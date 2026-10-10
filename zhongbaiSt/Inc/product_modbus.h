#ifndef PRODUCT_MODBUS_H
#define PRODUCT_MODBUS_H

#include "main.h"

#define MODBUS_SLAVE_ADDRESS       3U
#define MODBUS_RX_BUFFER_SIZE    256U
#define MODBUS_DMA_RX_SIZE        256U

/* SOFTWARE_VERSION_* are supplied by the validated compiler via file. */

#define MB_HREG_TOTAL_SWITCH       0U
#define MB_HREG_HEAT_BAR_LEVEL     2U
#define MB_HREG_HEAT_ROLL_LEVEL    3U
#define MB_HREG_MOTOR_LEVEL        4U

#define MB_IREG_ACTUAL_RPM         0U
#define MB_IREG_MOTOR_STATE        1U
#define MB_IREG_PHASE_CURRENT_MA   2U
#define MB_IREG_TOTAL_SWITCH       3U
#define MB_IREG_HEAT_BAR_SWITCH    4U
#define MB_IREG_HEAT_ROLL_SWITCH   5U
#define MB_IREG_HEAT_BAR_LEVEL     6U
#define MB_IREG_HEAT_ROLL_LEVEL    7U
#define MB_IREG_MOTOR_LEVEL        8U
#define MB_IREG_VERSION_HIGH      11U
#define MB_IREG_VERSION_LOW       12U

extern volatile uint32_t modbusRxOverflowCount;
extern volatile uint32_t modbusCrcErrorCount;
extern volatile uint32_t modbusFrameErrorCount;

/* AT32-style RX hand-off: the ISR copies each completed IDLE-delimited burst
 * into a bounded slot; the task parses the slot copy byte-wise. */
void ProductModbus_RxIdleCallback(const uint8_t *data, uint16_t length);

void ProductModbus_Init(void);
void ProductModbus_Task(void);
uint8_t ProductModbus_DebugMotor(int16_t rpm);
uint8_t ProductModbus_DebugHeat(uint8_t channel, uint16_t level);
void ProductModbus_ReportVersion(void);
uint16_t ProductModbus_CRC16(const uint8_t *data, uint16_t length);

#endif
