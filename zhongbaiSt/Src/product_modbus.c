#include "product_modbus.h"
#include "commissioning.h"
#include "mc_config.h"
#include "mc_api.h"
#include "parameters_conversion.h"

#define SOFTWARE_VERSION_MAJOR 0U
#define SOFTWARE_VERSION_MINOR 2U
#define SOFTWARE_VERSION_PATCH 7U
#define SOFTWARE_VERSION_BUILD 0U
#include "usart.h"
#include <string.h>

/* RS485 DE/RE direction control on PA4 (same wiring as reference board) */
#define DE_RE_Pin                 GPIO_PIN_4
#define DE_RE_GPIO_Port           GPIOA
#define RS485_RX_ENABLE()         HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET)
#define RS485_TX_ENABLE()         HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_SET)

#define MB_COIL_COUNT 10U
#define MB_DISCRETE_COUNT 16U
#define MB_INPUT_REG_COUNT 14U
#define MB_HOLD_REG_COUNT 103U
#define MB_EX_ILLEGAL_FUNCTION 1U
#define MB_EX_ILLEGAL_ADDRESS 2U
#define MB_EX_ILLEGAL_VALUE 3U

uint8_t modbusRxWindow[MODBUS_RX_WINDOW_SIZE];
volatile uint16_t modbusRxLength;
volatile uint32_t modbusRxOverflowCount;
volatile uint32_t modbusCrcErrorCount;
volatile uint32_t modbusFrameErrorCount;

static uint8_t coils[MB_COIL_COUNT];
static uint16_t holdingRegs[MB_HOLD_REG_COUNT];
static uint8_t response[256];

#define PRODUCT_LEVEL_RAMP_MS 500U
#define PRODUCT_LEVEL_ACC_RPM_S 32000U
#define PRODUCT_LEVEL_DEC_RPM_S 10000U
#define PRODUCT_LEVEL_IQ_RAW 10922
#define MB_HREG_ENGINEERING_COMMAND 5U
#define MB_HREG_LAST_COMMAND_RESULT 19U
#define MB_EXT_ID_KP 33U
#define MB_EXT_ID_KI 34U
#define MB_EXT_IQ_KP 35U
#define MB_EXT_IQ_KI 36U
#define MB_EXT_SPEED_KP 37U
#define MB_EXT_SPEED_KI 38U
#define MB_EXT_MODE 40U
#define MB_EXT_PROFILE 41U
#define MB_EXT_RPM 42U
#define MB_EXT_IQ_RAW 43U
#define MB_EXT_RAMP_MS 44U
#define MB_EXT_ACC 45U
#define MB_EXT_DEC 46U
#define MB_EXT_RUN_MS 47U
#define MB_EXT_RUN_COMMAND 48U
#define MB_SNAPSHOT_BASE 64U
#define MB_SNAPSHOT_COUNT 39U

static const uint16_t productLevelRpm[15] = {
  1000U, 2000U, 3000U, 4000U, 5000U, 6000U, 7000U, 8000U,
  9000U, 10000U, 11000U, 12000U, 13000U, 14000U, 15000U
};
static int16_t appliedMotorLevel;
static uint8_t directMotionActive;
static void productMotionTask(void);
static uint8_t startExtendedMotion(void);
static void updatePrivateSnapshot(void);
static void clearPowerOnBreakTransient(void);

extern uint8_t RecData;
extern int MotorLevel, TIM2HeatLevel, TIM3HeatLevel;
extern int TotolSwitch, HeatBarSwitch, HeatRollSwitch;

uint16_t ProductModbus_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint8_t bit;
  while(length-- != 0U)
  {
    crc ^= *data++;
    for(bit = 0U; bit < 8U; bit++)
      crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
  }
  return crc;
}

/**
  * @brief  Transmit bytes on the half-duplex RS485 bus.
  * @note   Direction is driven by the DE pin (PA4): high enables the bus
  *         driver, low re-enables the receiver. The pending interrupt
  *         reception is suspended before DE goes high and re-armed only after
  *         the final stop bit, and the receiver is flushed in between, so the
  *         driver echo can never leak into the receive window.
  * @note   HAL_UART_Transmit() waits for the UART TC flag before returning, so
  *         the driver is not disabled until the final stop bit has left PA2.
  */
static void writeHolding(uint16_t address, uint16_t value)
{
  switch(address)
  {
    case MB_HREG_TOTAL_SWITCH:
      TotolSwitch = value != 0U;
      coils[MB_HREG_TOTAL_SWITCH] = (uint8_t)TotolSwitch;
      holdingRegs[MB_HREG_TOTAL_SWITCH] = (uint16_t)TotolSwitch;
      if(TotolSwitch == 0)
      {
        holdingRegs[MB_HREG_HEAT_BAR_LEVEL] = 0U;
        holdingRegs[MB_HREG_HEAT_ROLL_LEVEL] = 0U;
        holdingRegs[MB_HREG_MOTOR_LEVEL] = 0U;
        TIM2HeatLevel = 0;
        TIM3HeatLevel = 0;
        MotorLevel = 0;
        HeatBarSwitch = 0;
        HeatRollSwitch = 0;
        if(appliedMotorLevel != 0 || directMotionActive != 0U)
          Commissioning_RemoteStop();
        appliedMotorLevel = 0;
        directMotionActive = 0U;
      }
      break;
    case MB_HREG_HEAT_BAR_LEVEL:
      holdingRegs[address] = TotolSwitch != 0 ? value : 0U;
      TIM2HeatLevel = (int)holdingRegs[address];
      HeatBarSwitch = TotolSwitch != 0 && TIM2HeatLevel != 0;
      break;
    case MB_HREG_HEAT_ROLL_LEVEL:
#if defined(F1_TEST_BOARD)
      holdingRegs[address] = TotolSwitch != 0 ? value : 0U;
      TIM3HeatLevel = (int)holdingRegs[address];
      HeatRollSwitch = TotolSwitch != 0 && TIM3HeatLevel != 0;
#else
      /* F1_POWER_V004 has no electrical connection from PB14 to a heater.
       * Keep the G4 register writable, but report and command this unavailable
       * channel as off instead of echoing a false active state. */
      holdingRegs[address] = 0U;
      TIM3HeatLevel = 0;
      HeatRollSwitch = 0;
#endif
      break;
    case MB_HREG_MOTOR_LEVEL:
      holdingRegs[address] = TotolSwitch != 0 ? value : 0U;
      MotorLevel = (int)(int16_t)holdingRegs[address];
      if(MotorLevel == 0 && directMotionActive != 0U)
      {
        Commissioning_RemoteStop();
        directMotionActive = 0U;
      }
      break;
    case MB_HREG_ENGINEERING_COMMAND:
      holdingRegs[address] = 0U;
      holdingRegs[MB_HREG_LAST_COMMAND_RESULT] = 1U;
      if(value == 2U)
      {
        Commissioning_RemoteStop();
        MotorLevel = 0;
        holdingRegs[MB_HREG_MOTOR_LEVEL] = 0U;
        appliedMotorLevel = 0;
        directMotionActive = 0U;
      }
      else if(value == 3U)
      {
        if(!MC_AcknowledgeFaultMotor1())
          holdingRegs[MB_HREG_LAST_COMMAND_RESULT] = 2U;
      }
      else if(value == 4U)
      {
        if(!Commissioning_RemoteRampStop(holdingRegs[MB_EXT_DEC]))
          holdingRegs[MB_HREG_LAST_COMMAND_RESULT] = 2U;
      }
      break;
    default:
      holdingRegs[address] = value;
      if(address == MB_EXT_RUN_COMMAND && value == 1U)
      {
        holdingRegs[MB_HREG_LAST_COMMAND_RESULT] =
            startExtendedMotion() != 0U ? 1U : 2U;
        holdingRegs[MB_EXT_RUN_COMMAND] = 0U;
      }
      break;
  }
}

static uint8_t validHoldingValue(uint16_t address, uint16_t value)
{
  if(address >= MB_SNAPSHOT_BASE) return 0U;
  switch(address)
  {
    case MB_HREG_TOTAL_SWITCH:
      return value <= 1U;
    case MB_HREG_HEAT_BAR_LEVEL:
    case MB_HREG_HEAT_ROLL_LEVEL:
      return value <= 13U;
    case MB_HREG_MOTOR_LEVEL:
      return (int16_t)value >= -15 && (int16_t)value <= 15;
    case MB_HREG_ENGINEERING_COMMAND:
      return value == 0U || value == 2U || value == 3U || value == 4U;
    case MB_EXT_MODE: case MB_EXT_PROFILE: case MB_EXT_RUN_COMMAND:
      return value <= 1U;
    case MB_EXT_RPM:
      return ((int16_t)value >= -20000 && (int16_t)value <= 20000 &&
              (int16_t)value != 0);
    case MB_EXT_IQ_RAW:
      return value >= 200U && value <= 10922U;
    case MB_EXT_RAMP_MS:
      return value >= 250U && value <= 10000U;
    case MB_EXT_ACC: case MB_EXT_DEC:
      return value >= 100U && value <= 60000U;
    case MB_EXT_RUN_MS:
      return value == 0U || (value >= 250U && value <= 60000U);
    default:
      return 1U;
  }
}

static uint8_t readDiscrete(uint16_t address)
{
  if(address == 0U) return TotolSwitch != 0;
  if(address == 1U) return HeatBarSwitch != 0;
  if(address == 2U) return HeatRollSwitch != 0;
  return 0U;
}

static uint16_t readInput(uint16_t address)
{
  int32_t speed;
  int32_t phaseCurrent;
  if(address == MB_IREG_ACTUAL_RPM)
  {
    speed = (int16_t)MC_GetMecSpeedAverageMotor1() * 6;
    return (uint16_t)(speed < 0 ? -speed : speed);
  }
  if(address == MB_IREG_MOTOR_STATE) return (uint16_t)MC_GetSTMStateMotor1();
  if(address == MB_IREG_PHASE_CURRENT_MA)
  {
    phaseCurrent = MC_GetPhaseCurrentAmplitudeMotor1();
    if(phaseCurrent < 0) phaseCurrent = -phaseCurrent;
    return (uint16_t)((phaseCurrent * (int32_t)(MAX_CURRENT * 1000.0) + 16384) / 32768);
  }
  if(address == MB_IREG_TOTAL_SWITCH) return (uint16_t)(TotolSwitch != 0);
  if(address == MB_IREG_HEAT_BAR_SWITCH) return (uint16_t)(HeatBarSwitch != 0);
  if(address == MB_IREG_HEAT_ROLL_SWITCH) return (uint16_t)(HeatRollSwitch != 0);
  if(address == MB_IREG_HEAT_BAR_LEVEL) return (uint16_t)TIM2HeatLevel;
  if(address == MB_IREG_HEAT_ROLL_LEVEL) return (uint16_t)TIM3HeatLevel;
  if(address == MB_IREG_MOTOR_LEVEL) return (uint16_t)(int16_t)MotorLevel;
  if(address == MB_IREG_VERSION_HIGH)
    return (uint16_t)(((uint16_t)SOFTWARE_VERSION_MAJOR << 8) | SOFTWARE_VERSION_MINOR);
  if(address == MB_IREG_VERSION_LOW)
    return (uint16_t)(((uint16_t)SOFTWARE_VERSION_PATCH << 8) | SOFTWARE_VERSION_BUILD);
  return 0U;
}

static void writeCoil(uint16_t address, uint8_t on)
{
  coils[address] = on != 0U;
  if(address == 0U) writeHolding(MB_HREG_TOTAL_SWITCH, on);
}

static uint16_t exceptionResponse(uint8_t function, uint8_t exception)
{
  response[0] = MODBUS_SLAVE_ADDRESS;
  response[1] = (uint8_t)(function | 0x80U);
  response[2] = exception;
  return 3U;
}

static uint16_t processRequest(const uint8_t *r, uint16_t length, uint8_t broadcast)
{
  uint8_t fn = r[1];
  uint16_t address = (uint16_t)(((uint16_t)r[2] << 8) | r[3]);
  uint16_t quantity, value, i, out = 0U;
  if(length < 8U) return 0U;
  switch(fn)
  {
    case 0x01U: case 0x02U:
      quantity = (uint16_t)(((uint16_t)r[4] << 8) | r[5]);
      if(quantity == 0U || quantity > 2000U) out = exceptionResponse(fn, MB_EX_ILLEGAL_VALUE);
      else if(address >= (fn == 0x01U ? MB_COIL_COUNT : MB_DISCRETE_COUNT) ||
              quantity > (uint16_t)((fn == 0x01U ? MB_COIL_COUNT : MB_DISCRETE_COUNT) - address))
        out = exceptionResponse(fn, MB_EX_ILLEGAL_ADDRESS);
      else
      {
        response[0] = MODBUS_SLAVE_ADDRESS; response[1] = fn; response[2] = (uint8_t)((quantity + 7U) / 8U);
        memset(&response[3], 0, response[2]);
        for(i = 0U; i < quantity; i++)
        {
          uint8_t state = fn == 0x01U ? coils[address + i] : readDiscrete(address + i);
          if(state != 0U) response[3U + i / 8U] |= (uint8_t)(1U << (i % 8U));
        }
        out = (uint16_t)(3U + response[2]);
      }
      break;
    case 0x03U: case 0x04U:
      quantity = (uint16_t)(((uint16_t)r[4] << 8) | r[5]);
      if(quantity == 0U || quantity > 125U) out = exceptionResponse(fn, MB_EX_ILLEGAL_VALUE);
      else if(address >= (fn == 0x03U ? MB_HOLD_REG_COUNT : MB_INPUT_REG_COUNT) ||
              quantity > (uint16_t)((fn == 0x03U ? MB_HOLD_REG_COUNT : MB_INPUT_REG_COUNT) - address))
        out = exceptionResponse(fn, MB_EX_ILLEGAL_ADDRESS);
      else
      {
        response[0] = MODBUS_SLAVE_ADDRESS; response[1] = fn; response[2] = (uint8_t)(quantity * 2U);
        for(i = 0U; i < quantity; i++)
        {
          value = fn == 0x03U ? holdingRegs[address + i] : readInput(address + i);
          response[3U + i * 2U] = (uint8_t)(value >> 8); response[4U + i * 2U] = (uint8_t)value;
        }
        out = (uint16_t)(3U + quantity * 2U);
      }
      break;
    case 0x05U:
      value = (uint16_t)(((uint16_t)r[4] << 8) | r[5]);
      if(address >= MB_COIL_COUNT) out = exceptionResponse(fn, MB_EX_ILLEGAL_ADDRESS);
      else if(value != 0U && value != 0xFF00U) out = exceptionResponse(fn, MB_EX_ILLEGAL_VALUE);
      else { writeCoil(address, value == 0xFF00U); memcpy(response, r, 6U); out = 6U; }
      break;
    case 0x06U:
      value = (uint16_t)(((uint16_t)r[4] << 8) | r[5]);
      if(address >= MB_HOLD_REG_COUNT) out = exceptionResponse(fn, MB_EX_ILLEGAL_ADDRESS);
      else if(!validHoldingValue(address, value)) out = exceptionResponse(fn, MB_EX_ILLEGAL_VALUE);
      else { writeHolding(address, value); memcpy(response, r, 6U); out = 6U; }
      break;
    case 0x0FU:
      quantity = (uint16_t)(((uint16_t)r[4] << 8) | r[5]);
      if(quantity == 0U || quantity > 1968U || r[6] != (uint8_t)((quantity + 7U) / 8U)) out = exceptionResponse(fn, MB_EX_ILLEGAL_VALUE);
      else if(address >= MB_COIL_COUNT || quantity > (uint16_t)(MB_COIL_COUNT - address)) out = exceptionResponse(fn, MB_EX_ILLEGAL_ADDRESS);
      else { for(i = 0U; i < quantity; i++) writeCoil(address + i, (uint8_t)((r[7U + i / 8U] >> (i % 8U)) & 1U)); memcpy(response, r, 6U); out = 6U; }
      break;
    case 0x10U:
      quantity = (uint16_t)(((uint16_t)r[4] << 8) | r[5]);
      if(quantity == 0U || quantity > 123U || r[6] != (uint8_t)(quantity * 2U)) out = exceptionResponse(fn, MB_EX_ILLEGAL_VALUE);
      else if(address >= MB_HOLD_REG_COUNT || quantity > (uint16_t)(MB_HOLD_REG_COUNT - address)) out = exceptionResponse(fn, MB_EX_ILLEGAL_ADDRESS);
      else
      {
        for(i = 0U; i < quantity; i++)
        {
          value = (uint16_t)(((uint16_t)r[7U + i * 2U] << 8) | r[8U + i * 2U]);
          if(!validHoldingValue((uint16_t)(address + i), value)) break;
        }
        if(i != quantity) out = exceptionResponse(fn, MB_EX_ILLEGAL_VALUE);
        else
        {
          for(i = 0U; i < quantity; i++)
            writeHolding((uint16_t)(address + i), (uint16_t)(((uint16_t)r[7U + i * 2U] << 8) | r[8U + i * 2U]));
          memcpy(response, r, 6U); out = 6U;
        }
      }
      break;
    default: out = exceptionResponse(fn, MB_EX_ILLEGAL_FUNCTION); break;
  }
  return broadcast != 0U ? 0U : out;
}

static uint8_t supportedFunction(uint8_t fn)
{
  return fn == 1U || fn == 2U || fn == 3U || fn == 4U || fn == 5U || fn == 6U || fn == 15U || fn == 16U;
}

static uint16_t requestLength(const uint8_t *data, uint16_t available)
{
  if(available < 2U) return 0U;
  if(data[1] >= 1U && data[1] <= 6U) return 8U;
  if(data[1] == 15U || data[1] == 16U) return available >= 7U ? (uint16_t)(9U + data[6]) : 0U;
  return 0U;
}

static void consumeRxBytes(uint16_t count)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if(count > modbusRxLength) count = modbusRxLength;
  modbusRxLength = (uint16_t)(modbusRxLength - count);
  if(modbusRxLength != 0U) memmove(modbusRxWindow, &modbusRxWindow[count], modbusRxLength);
  if(primask == 0U) __enable_irq();
}

static uint8_t runMotion(int16_t rpm, uint16_t rampMs, uint16_t acc,
                         uint16_t dec, uint16_t runMs, uint8_t profile)
{
  CommissioningRemoteConfig c;
  c.mode = 0U; c.profile = profile; c.rpm = rpm;
  c.iqRaw = PRODUCT_LEVEL_IQ_RAW; c.rampMs = rampMs;
  c.accRpmS = acc; c.decRpmS = dec; c.runMs = runMs;
  c.idKp = 3321; c.idKi = 122; c.iqKp = 3321; c.iqKi = 122;
  c.speedKp = 200; c.speedKi = 25;
  return Commissioning_RemoteRun(&c) ? 1U : 0U;
}

static uint8_t startExtendedMotion(void)
{
  CommissioningRemoteConfig c;
  c.mode=(uint8_t)holdingRegs[MB_EXT_MODE]; c.profile=(uint8_t)holdingRegs[MB_EXT_PROFILE];
  c.rpm=(int16_t)holdingRegs[MB_EXT_RPM]; c.iqRaw=(int16_t)holdingRegs[MB_EXT_IQ_RAW];
  c.rampMs=holdingRegs[MB_EXT_RAMP_MS]; c.accRpmS=holdingRegs[MB_EXT_ACC];
  c.decRpmS=holdingRegs[MB_EXT_DEC]; c.runMs=holdingRegs[MB_EXT_RUN_MS];
  c.idKp=(int16_t)holdingRegs[MB_EXT_ID_KP]; c.idKi=(int16_t)holdingRegs[MB_EXT_ID_KI];
  c.iqKp=(int16_t)holdingRegs[MB_EXT_IQ_KP]; c.iqKi=(int16_t)holdingRegs[MB_EXT_IQ_KI];
  c.speedKp=(int16_t)holdingRegs[MB_EXT_SPEED_KP]; c.speedKi=(int16_t)holdingRegs[MB_EXT_SPEED_KI];
  if(!Commissioning_RemoteRun(&c)) return 0U;
  directMotionActive=1U; appliedMotorLevel=0; return 1U;
}

static void productMotionTask(void)
{
  int16_t requested = (TotolSwitch != 0) ? (int16_t)MotorLevel : 0;
  State_t state = MC_GetSTMStateMotor1();
  if(directMotionActive != 0U)
  {
    if(state == IDLE) directMotionActive = 0U;
    return;
  }
  if(requested == 0)
  {
    if(appliedMotorLevel != 0) Commissioning_RemoteStop();
    appliedMotorLevel=0; return;
  }
  if(appliedMotorLevel != 0 && ((appliedMotorLevel < 0) != (requested < 0)))
  {
    Commissioning_RemoteStop(); appliedMotorLevel=0; return;
  }
  if(state == IDLE || (state == RUN && appliedMotorLevel != requested))
  {
    int16_t magnitude = requested < 0 ? (int16_t)-requested : requested;
    int16_t rpm = (int16_t)productLevelRpm[magnitude - 1];
    if(requested < 0) rpm = (int16_t)-rpm;
    if(runMotion(rpm, PRODUCT_LEVEL_RAMP_MS, PRODUCT_LEVEL_ACC_RPM_S,
                 PRODUCT_LEVEL_DEC_RPM_S, 0U, 1U)) appliedMotorLevel=requested;
  }
}

static void updatePrivateSnapshot(void)
{
  CommissioningRemoteStatus motion;
  qd_t iq = MC_GetIqdMotor1();
  qd_t ref = MC_GetIqdrefMotor1();
  int32_t rpm = (int32_t)MC_GetMecSpeedAverageMotor1() * 6;
  int32_t phaseCurrent = MC_GetPhaseCurrentAmplitudeMotor1();
  uint16_t flags;

  Commissioning_GetRemoteStatus(&motion);
  if(phaseCurrent < 0) phaseCurrent = -phaseCurrent;
  phaseCurrent = (phaseCurrent * (int32_t)(MAX_CURRENT * 1000.0) + 16384) / 32768;
  flags = (uint16_t)((TotolSwitch != 0 ? 1U : 0U) |
      ((TIM1->BDTR & TIM_BDTR_MOE) != 0U ? 2U : 0U) |
      (HeatBarSwitch != 0 ? 4U : 0U) | (HeatRollSwitch != 0 ? 8U : 0U) |
      ((motion.flags & 1U) != 0U ? 16U : 0U) |
      ((motion.flags & 2U) != 0U ? 32U : 0U) |
      (directMotionActive != 0U ? 64U : 0U) |
      ((motion.flags & 4U) != 0U ? 128U : 0U) |
      (HAL_GPIO_ReadPin(M1_OCP_GPIO_Port, M1_OCP_Pin) == GPIO_PIN_SET ? 256U : 0U) |
      ((motion.flags & 8U) != 0U ? 512U : 0U));

  holdingRegs[64] = (uint16_t)MC_GetSTMStateMotor1();
  holdingRegs[65] = flags;
  holdingRegs[66] = motion.phase;
  holdingRegs[67] = (uint16_t)motion.targetRpm;
  holdingRegs[68] = (uint16_t)motion.referenceRpm;
  holdingRegs[69] = (uint16_t)(int16_t)rpm;
  holdingRegs[70] = (uint16_t)motion.observerRpm;
  holdingRegs[71] = motion.busVoltageV;
  holdingRegs[72] = (uint16_t)phaseCurrent;
  holdingRegs[73] = MC_GetCurrentFaultsMotor1();
  holdingRegs[74] = MC_GetOccurredFaultsMotor1();
  holdingRegs[75] = (uint16_t)iq.d;
  holdingRegs[76] = (uint16_t)iq.q;
  holdingRegs[77] = (uint16_t)ref.d;
  holdingRegs[78] = (uint16_t)ref.q;
  holdingRegs[79] = (uint16_t)Commissioning_VqBeforeLimit;
  holdingRegs[80] = (uint16_t)Commissioning_VdBeforeLimit;
  holdingRegs[81] = (uint16_t)Commissioning_VqApplied;
  holdingRegs[82] = (uint16_t)Commissioning_VdApplied;
  holdingRegs[83] = motion.reportFlags;
  holdingRegs[84] = motion.startMs;
  holdingRegs[85] = motion.loopMs;
  holdingRegs[86] = motion.bandMs;
  holdingRegs[87] = motion.settleMs;
  holdingRegs[88] = motion.reachedMs;
  holdingRegs[89] = (uint16_t)motion.reachedRpm;
  holdingRegs[90] = (uint16_t)motion.iqLimitMs;
  holdingRegs[91] = (uint16_t)(motion.iqLimitMs >> 16);
  holdingRegs[92] = (uint16_t)Commissioning_VoltageLimitTicks;
  holdingRegs[93] = (uint16_t)(Commissioning_VoltageLimitTicks >> 16);
  holdingRegs[94] = (uint16_t)modbusCrcErrorCount;
  holdingRegs[95] = (uint16_t)(modbusCrcErrorCount >> 16);
  holdingRegs[96] = (uint16_t)modbusFrameErrorCount;
  holdingRegs[97] = (uint16_t)(modbusFrameErrorCount >> 16);
  holdingRegs[98] = (uint16_t)modbusRxOverflowCount;
  holdingRegs[99] = (uint16_t)(modbusRxOverflowCount >> 16);
  holdingRegs[100] = holdingRegs[MB_HREG_LAST_COMMAND_RESULT];
  holdingRegs[101] = (uint16_t)(int16_t)MotorLevel;
  holdingRegs[102] = motion.stopReason;
}

void ProductModbus_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Configure PA4 as RS485 DE/RE push-pull output (was DAC1_OUT1 debug pin) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = DE_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DE_RE_GPIO_Port, &GPIO_InitStruct);
  RS485_RX_ENABLE();

  modbusRxLength = 0U;
  holdingRegs[MB_EXT_ID_KP] = 3321U; holdingRegs[MB_EXT_ID_KI] = 122U;
  holdingRegs[MB_EXT_IQ_KP] = 3321U; holdingRegs[MB_EXT_IQ_KI] = 122U;
  holdingRegs[MB_EXT_SPEED_KP] = 200U; holdingRegs[MB_EXT_SPEED_KI] = 25U;
  holdingRegs[MB_EXT_MODE] = 0U; holdingRegs[MB_EXT_PROFILE] = 1U;
  holdingRegs[MB_EXT_RPM] = 10000U; holdingRegs[MB_EXT_IQ_RAW] = PRODUCT_LEVEL_IQ_RAW;
  holdingRegs[MB_EXT_RAMP_MS] = PRODUCT_LEVEL_RAMP_MS;
  holdingRegs[MB_EXT_ACC] = PRODUCT_LEVEL_ACC_RPM_S;
  holdingRegs[MB_EXT_DEC] = PRODUCT_LEVEL_DEC_RPM_S;
  holdingRegs[MB_EXT_RUN_MS] = 0U;
  appliedMotorLevel = 0; directMotionActive = 0U;
  Commissioning_Init();
  (void)RS485_StartReceiveDMA();
  HAL_Delay(1000U);
  ProductModbus_ReportVersion(); HAL_Delay(100U);
  ProductModbus_ReportVersion(); HAL_Delay(100U);
  ProductModbus_ReportVersion();
  clearPowerOnBreakTransient();
}

/* The IPM fault output can be low while its supply and the external pull-up
 * settle during an MCU reset. TIM1 correctly latches that edge as BREAK_IN,
 * although the product has never enabled PWM. Clear only this exact boot-only
 * condition; persistent or mixed faults, and every break after startup, remain
 * latched for explicit engineering acknowledgement. */
static void clearPowerOnBreakTransient(void)
{
  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U &&
      HAL_GPIO_ReadPin(M1_OCP_GPIO_Port, M1_OCP_Pin) == GPIO_PIN_SET &&
      MC_GetCurrentFaultsMotor1() == MC_NO_ERROR &&
      MC_GetOccurredFaultsMotor1() == MC_BREAK_IN &&
      MC_GetSTMStateMotor1() == FAULT_OVER)
  {
    (void)MC_AcknowledgeFaultMotor1();
  }
}

void ProductModbus_ReportVersion(void)
{
  uint8_t versionFrame[8] = {
    0x76U, SOFTWARE_VERSION_MAJOR, SOFTWARE_VERSION_MINOR,
    SOFTWARE_VERSION_PATCH, SOFTWARE_VERSION_BUILD, 0U, 0U, 0x6EU
  };
  uint16_t crc = ProductModbus_CRC16(versionFrame, 5U);

  versionFrame[5] = (uint8_t)crc;
  versionFrame[6] = (uint8_t)(crc >> 8);
  (void)RS485_Transmit(versionFrame, sizeof(versionFrame), 100U);
}

void ProductModbus_RxByteISR(uint8_t value)
{
  if(modbusRxLength < MODBUS_RX_WINDOW_SIZE) modbusRxWindow[modbusRxLength++] = value;
  else modbusRxOverflowCount++;
}

void ProductModbus_RxBytesISR(const uint8_t *data, uint16_t length)
{
  uint16_t i;
  for(i = 0U; i < length; i++) ProductModbus_RxByteISR(data[i]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if !defined(F1_PRODUCT_MODBUS)
  if(huart->Instance == USART2)
  {
    ProductModbus_RxByteISR(RecData);
    (void)HAL_UART_Receive_IT(&huart2, &RecData, 1U);
  }
#else
  (void)huart;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    modbusFrameErrorCount++;
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
#if defined(F1_PRODUCT_MODBUS)
    (void)RS485_StartReceiveDMA();
#else
    (void)HAL_UART_Receive_IT(&huart2, &RecData, 1U);
#endif
  }
}

void ProductModbus_Task(void)
{
  uint16_t available, frameLength, crc, replyLength;
  uint8_t address;

  Commissioning_Task();
  productMotionTask();
  holdingRegs[17] = (uint16_t)MC_GetSTMStateMotor1();
  holdingRegs[18] = (uint16_t)((TotolSwitch != 0 ? 1U : 0U) |
      ((TIM1->BDTR & TIM_BDTR_MOE) != 0U ? 2U : 0U) |
      (HeatBarSwitch != 0 ? 4U : 0U) | (HeatRollSwitch != 0 ? 8U : 0U));
  holdingRegs[21] = (uint16_t)(appliedMotorLevel < 0 ?
      productLevelRpm[-appliedMotorLevel - 1] :
      (appliedMotorLevel > 0 ? productLevelRpm[appliedMotorLevel - 1] : 0U));
  { int32_t rpm=(int32_t)MC_GetMecSpeedAverageMotor1()*6;
    holdingRegs[22]=(uint16_t)(rpm<0?-rpm:rpm); holdingRegs[23]=(uint16_t)(int16_t)rpm; }
  holdingRegs[26]=MC_GetCurrentFaultsMotor1(); holdingRegs[27]=MC_GetOccurredFaultsMotor1();
  { qd_t iq=MC_GetIqdMotor1(), ref=MC_GetIqdrefMotor1();
    holdingRegs[50]=(uint16_t)iq.d; holdingRegs[51]=(uint16_t)iq.q;
    holdingRegs[52]=(uint16_t)ref.d; holdingRegs[53]=(uint16_t)ref.q; }
  holdingRegs[54]=(uint16_t)Commissioning_VqBeforeLimit;
  holdingRegs[55]=(uint16_t)Commissioning_VdBeforeLimit;
  holdingRegs[56]=(uint16_t)Commissioning_VqApplied;
  holdingRegs[57]=(uint16_t)Commissioning_VdApplied;
  holdingRegs[58]=(uint16_t)Commissioning_VoltageLimitTicks;
  holdingRegs[59]=(uint16_t)(Commissioning_VoltageLimitTicks >> 16);
  updatePrivateSnapshot();
  if(modbusRxLength == 0U) return;

  while(1)
  {
    available = modbusRxLength;
    if(available < 2U) return;
    address = modbusRxWindow[0];
    if((address != MODBUS_SLAVE_ADDRESS && address != 0U) || !supportedFunction(modbusRxWindow[1]))
    {
      modbusFrameErrorCount++; consumeRxBytes(1U); continue;
    }
    frameLength = requestLength(modbusRxWindow, available);
    if(frameLength == 0U)
    {
      if((modbusRxWindow[1] == 15U || modbusRxWindow[1] == 16U) && available < 7U)
        return;
      modbusFrameErrorCount++; consumeRxBytes(1U); continue;
    }
    if(frameLength > MODBUS_RX_WINDOW_SIZE)
    {
      modbusFrameErrorCount++; consumeRxBytes(1U); continue;
    }
    if(available < frameLength)
    {
      return;
    }
    crc = (uint16_t)(modbusRxWindow[frameLength - 2U] | ((uint16_t)modbusRxWindow[frameLength - 1U] << 8));
    if(crc != ProductModbus_CRC16(modbusRxWindow, (uint16_t)(frameLength - 2U)))
    {
      modbusCrcErrorCount++; consumeRxBytes(1U); continue;
    }
    replyLength = processRequest(modbusRxWindow, frameLength, address == 0U);
    consumeRxBytes(frameLength);
    if(replyLength != 0U)
    {
      crc = ProductModbus_CRC16(response, replyLength);
      response[replyLength++] = (uint8_t)crc; response[replyLength++] = (uint8_t)(crc >> 8);
      (void)RS485_Transmit(response, replyLength, 100U);
    }
  }
}
