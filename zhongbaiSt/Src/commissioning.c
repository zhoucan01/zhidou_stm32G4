#include "commissioning.h"

#include "main.h"
#include "mc_api.h"
#include "mc_config.h"
#include "pid_regulator.h"
#include "revup_ctrl.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "usart.h"
#if defined(F1_CN2_ASCII)
#include "cn2_ascii.h"
#endif

#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define FW_NAME "F1 Zhongbai Lab"
#define FW_VERSION "v1.5-exp"
#if defined(F1_CN2_ASCII_M1) || defined(F1_PRODUCT_MODBUS_M1)
#define MOTOR_PROFILE "M1"
#else
#define MOTOR_PROFILE "M3"
#endif
#define RX_LINE_SIZE 96u
#define ARM_BUS_MIN_V 250u
#define ARM_BUS_MAX_V 380u
#define OPEN_RPM_MIN 60
#define OPEN_RPM_MAX 2000
#if defined(F1_PRODUCT_MODBUS_M1)
#define CL_RPM_MIN 1000
#elif defined(F1_CN2_ASCII_M1)
#define CL_RPM_MIN 2000
#else
#define CL_RPM_MIN 2500
#endif
#define CL_RPM_MAX 20000
#define CL_START_RPM 3500
#define FAST_START_RPM 2500
#define FAST_HANDOFF_RPM 1800
#define STANDARD_HANDOFF_RPM 2500
#define TRANSITION_MS_MAX 200u
#define CL_RAMP_MS_MIN 250u
#define SPEED_RATE_MIN 100u
#define SPEED_RATE_MAX 60000u
#define SPEED_RAMP_MS_MAX 60000u
#define DEC_SETTLE_GRACE_MS 500u
#define IQ_RAW_MIN 200
#define IQ_RAW_MAX 10922
#define RUN_MS_MIN 250u
#define RUN_MS_MAX 60000u
#define RAMP_MS_MIN 250u
#define RAMP_MS_MAX 10000u
#define START_GRACE_MS 15000u
#define TARGET_BAND_MS 200u
#define BREAK_RETRY_WINDOW_MS 50u
#define BREAK_RETRY_QUIET_MS 500u
#define BREAK_RETRY_WAIT_MS 2000u

/* A product STOP may clear faults which cannot damage the power stage once
 * their live condition has disappeared. Hardware protection and internal
 * execution faults remain latched for engineering inspection. */
#define PRODUCT_RECOVERABLE_FAULTS (MC_UNDER_VOLT | MC_START_UP | MC_SPEED_FDBK)

#if defined(F1_TEST_BOARD)
#define BOARD_PROFILE "F1_TEST"
#else
#define BOARD_PROFILE "F1_POWER_V004"
#endif

typedef enum { MODE_CL = 0, MODE_OPEN = 1 } MotionMode;

typedef struct
{
  MotionMode mode;
  int16_t direction;
  int16_t rpm;
  int16_t iqRaw;
  uint16_t rampMs;
  uint16_t accRpmS;
  uint16_t decRpmS;
  uint16_t runMs;
  bool fastStart;
  int16_t idKp;
  int16_t idKi;
  int16_t iqKp;
  int16_t iqKi;
  int16_t speedKp;
  int16_t speedKi;
} MotionConfig;

static volatile char rxLine[RX_LINE_SIZE];
static volatile uint8_t rxLength;
static volatile bool lineReady;
static volatile bool rxOverflow;
static MotionConfig active;
static MotionConfig lastRun;
static bool runRequested;
static bool breakRetryPending;
static bool breakRetryUsed;
static bool breakRetryAcknowledged;
static uint32_t breakRetryAt;
static uint32_t breakClearSince;
static uint32_t runDeadline;
static uint32_t startDeadline;
static bool holdStarted;
static bool decelerating;
static uint32_t decelDeadline;
static uint16_t lastAccRampMs;
static uint16_t lastDecRampMs;
static bool loopSeen;
static uint32_t runCommandAt;
static uint32_t pwmAt;
static bool pwmSeen;
static uint32_t loopAt;
static uint32_t firstBandAt;
static uint32_t bandSince;
static uint32_t settleAt;
static uint32_t reachedAt;
static uint32_t reportEndAt;
static uint32_t lastTimingAt;
static uint32_t iqLimitMs;
static int32_t reachedRpm;
static bool firstBandSeen;
static bool inBand;
static bool settleSeen;
static bool targetReached;
static bool reportAvailable;
static const char *reportReason;
static bool productFaultClearRequested;
static uint16_t transitionDurationMs;
static uint16_t observerStartRpm;
static volatile bool transitionStartEvent;
static volatile bool transitionEndEvent;
static volatile uint32_t transitionStartAt;
static volatile uint32_t transitionEndAt;
static volatile int16_t transitionStartForcedUnit;
static volatile int16_t transitionStartObservedUnit;
static volatile int16_t transitionEndForcedUnit;
static volatile int16_t transitionEndObservedUnit;
static volatile bool fallbackStartEvent;
static volatile bool fallbackHandoffEvent;
static volatile uint32_t fallbackStartAt;
static volatile uint32_t fallbackHandoffAt;
static volatile int16_t fallbackStartForcedUnit;
static volatile int16_t fallbackStartObservedUnit;
static volatile int16_t fallbackHandoffForcedUnit;
static volatile int16_t fallbackHandoffObservedUnit;

volatile int16_t Commissioning_VqBeforeLimit;
volatile int16_t Commissioning_VdBeforeLimit;
volatile int16_t Commissioning_VqApplied;
volatile int16_t Commissioning_VdApplied;
volatile uint32_t Commissioning_VoltageLimitTicks;

static void Send(const char *text)
{
#if defined(F1_PRODUCT_MODBUS)
  (void)text;
#elif defined(F1_CN2_ASCII)
  Cn2Ascii_Transmit((const uint8_t *)text, (uint16_t)strlen(text));
#else
  (void)RS485_Transmit((const uint8_t *)text, (uint16_t)strlen(text), 200u);
#endif
}

static void Reply(const char *format, ...)
{
  char out[256];
  va_list args;
  va_start(args, format);
  (void)vsnprintf(out, sizeof(out), format, args);
  va_end(args);
  Send(out);
}

static uint16_t BusVoltage(void)
{
  return VBS_GetAvBusVoltage_V(&RealBusVoltageSensorParamsM1._Super);
}

static bool IsStopped(void)
{
  State_t state = MC_GetSTMStateMotor1();
  return !runRequested && ((state == IDLE) || (state == FAULT_OVER));
}

static void FormatElapsed(char *out, size_t size, bool available, uint32_t eventAt)
{
  if (available) (void)snprintf(out, size, "%lu", (unsigned long)(eventAt - runCommandAt));
  else (void)snprintf(out, size, "NA");
}

static uint16_t RateToDurationMs(uint32_t speedChangeRpm, uint16_t rateRpmS)
{
  uint32_t duration = (speedChangeRpm * 1000u + rateRpmS - 1u) / rateRpmS;
  if (duration == 0u) duration = 1u;
  if (duration > SPEED_RAMP_MS_MAX) duration = SPEED_RAMP_MS_MAX;
  return (uint16_t)duration;
}

static void Report(void)
{
  char start[16], loop[16], band[16], settle[16], reached[16];
  uint32_t endAt;
  if (!reportAvailable) { Send("REPORT NONE\r\n"); return; }
  endAt = runRequested ? HAL_GetTick() : reportEndAt;
  FormatElapsed(start, sizeof(start), pwmSeen, pwmAt);
  FormatElapsed(loop, sizeof(loop), loopSeen, loopAt);
  FormatElapsed(band, sizeof(band), firstBandSeen, firstBandAt);
  FormatElapsed(settle, sizeof(settle), settleSeen, settleAt);
  FormatElapsed(reached, sizeof(reached), targetReached, reachedAt);
  Reply("REPORT MODE=%s PROFILE=%s TARGET=%d RESULT=%s STOP=%s START_MS=%s LOOP_MS=%s BAND_MS=%s "
        "SETTLE_MS=%s REACHED_MS=%s ELAPSED_MS=%lu REACHED_RPM=%ld IQ_LIMIT_MS=%lu\r\n",
        lastRun.mode == MODE_CL ? "CL" : "OPEN", lastRun.fastStart ? "FAST" : "STANDARD",
        lastRun.direction * lastRun.rpm,
        targetReached ? "EST_TARGET_REACHED" : "UNREACHED",
        runRequested ? "ACTIVE" : reportReason,
        start, loop, band, settle, reached,
        (unsigned long)(endAt - runCommandAt), (long)reachedRpm,
        (unsigned long)iqLimitMs);
}

static void StopNow(const char *reason)
{
  (void)MC_StopMotor1();
  if (!runRequested) { Send("OK STOP already stopped\r\n"); return; }
  reportEndAt = HAL_GetTick();
  reportReason = reason;
  runRequested = false;
  breakRetryPending = false;
  holdStarted = false;
  decelerating = false;
  Reply("EVENT STOP REASON=%s\r\n", reason);
  Report();
}

static void Defaults(MotionConfig *cfg)
{
  cfg->mode = MODE_OPEN;
  cfg->direction = 1;
  cfg->rpm = 120;
  cfg->iqRaw = 900;
  cfg->rampMs = 2000u;
  cfg->accRpmS = 10000u;
  cfg->decRpmS = 10000u;
  cfg->runMs = 10000u;
  cfg->fastStart = false;
  cfg->idKp = PID_GetKP(&PIDIdHandle_M1);
  cfg->idKi = PID_GetKI(&PIDIdHandle_M1);
  cfg->iqKp = PID_GetKP(&PIDIqHandle_M1);
  cfg->iqKi = PID_GetKI(&PIDIqHandle_M1);
  cfg->speedKp = PID_GetKP(&PIDSpeedHandle_M1);
  cfg->speedKi = PID_GetKI(&PIDSpeedHandle_M1);
}

static bool Validate(const MotionConfig *cfg)
{
  int16_t minRpm = (cfg->mode == MODE_OPEN) ? OPEN_RPM_MIN : CL_RPM_MIN;
  int16_t maxRpm = (cfg->mode == MODE_OPEN) ? OPEN_RPM_MAX : CL_RPM_MAX;
  return (cfg->direction == 1 || cfg->direction == -1) &&
         cfg->rpm >= minRpm && cfg->rpm <= maxRpm &&
         cfg->iqRaw >= IQ_RAW_MIN && cfg->iqRaw <= IQ_RAW_MAX &&
         cfg->rampMs >= RAMP_MS_MIN && cfg->rampMs <= RAMP_MS_MAX &&
         (cfg->mode != MODE_CL || cfg->rampMs >= CL_RAMP_MS_MIN) &&
         cfg->accRpmS >= SPEED_RATE_MIN && cfg->accRpmS <= SPEED_RATE_MAX &&
         cfg->decRpmS >= SPEED_RATE_MIN && cfg->decRpmS <= SPEED_RATE_MAX &&
         ((cfg->runMs >= RUN_MS_MIN && cfg->runMs <= RUN_MS_MAX) ||
#if defined(F1_PRODUCT_MODBUS)
          cfg->runMs == 0u ||
#endif
          false) &&
         cfg->idKp > 0 && cfg->idKi >= 0 && cfg->iqKp > 0 && cfg->iqKi >= 0 &&
         cfg->speedKp > 0 && cfg->speedKi >= 0;
}

static void ApplyConfig(void)
{
  uint8_t i;
  int16_t speedUnit = (int16_t)(active.rpm / 6);
  uint16_t phaseMs = (uint16_t)(active.rampMs / 5u);
  int16_t startRpm = active.fastStart ? FAST_START_RPM : CL_START_RPM;
  /* Use the generated ST startup topology: VSS performs the complete forced
     ramp, then a converged STO takes over with a zero-duration transition.
     Keeping only this path removes the dynamic-blend/fallback state race. */
  transitionDurationMs = 0u;
#if defined(F1_CN2_ASCII_M1) || defined(F1_PRODUCT_MODBUS_M1)
  observerStartRpm = active.fastStart ? FAST_HANDOFF_RPM : STANDARD_HANDOFF_RPM;
#else
  observerStartRpm = STANDARD_HANDOFF_RPM;
#endif

  PID_SetKP(&PIDIdHandle_M1, active.idKp);
  PID_SetKI(&PIDIdHandle_M1, active.idKi);
  PID_SetKP(&PIDIqHandle_M1, active.iqKp);
  PID_SetKI(&PIDIqHandle_M1, active.iqKi);
  PID_SetKP(&PIDSpeedHandle_M1, active.speedKp);
  PID_SetKI(&PIDSpeedHandle_M1, active.speedKi);
  PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, active.iqRaw);
  PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, (int16_t)-active.iqRaw);

  for (i = 0u; i < 5u; ++i)
  {
    int16_t startSpeedUnit = active.mode == MODE_OPEN ? speedUnit : (int16_t)(startRpm / 6);
    RUC_SetPhaseDurationms(&RevUpControlM1, i, phaseMs);
    RUC_SetPhaseFinalMecSpeedUnit(&RevUpControlM1, i,
        (int16_t)(((int32_t)startSpeedUnit * (int32_t)(i + 1u)) / 5));
    RUC_SetPhaseFinalTorque(&RevUpControlM1, i,
        (active.mode == MODE_CL && i == 0u && active.iqRaw > PHASE1_FINAL_CURRENT)
            ? PHASE1_FINAL_CURRENT : active.iqRaw);
  }
  RevUpControlM1.bFirstAccelerationStage = active.mode == MODE_OPEN ? 5u : 2u;
  RevUpControlM1.hMinStartUpValidSpeed = (uint16_t)(observerStartRpm / 6u);
  STO_SetMinStartUpValidSpeedUnit(&STO_PLL_M1, (uint16_t)(observerStartRpm / 6u));
  VirtualSpeedSensorM1.hTransitionSteps =
      (int16_t)(((uint32_t)TF_REGULATION_RATE * transitionDurationMs) / 1000u);
}

static void QueueStartRamp(void)
{
  int16_t speedUnit = (int16_t)((active.direction * active.rpm) / 6);
  ApplyConfig();
  if (active.mode == MODE_OPEN)
    MC_ProgramSpeedRampMotor1(speedUnit, active.rampMs);
  else
  {
    uint32_t remainingRpm = active.rpm > STANDARD_HANDOFF_RPM
        ? (uint32_t)active.rpm - STANDARD_HANDOFF_RPM : 0u;
    MC_ProgramSpeedRampMotor1(speedUnit, RateToDurationMs(remainingRpm, active.accRpmS));
  }
}

static void ArmRunTracking(void)
{
  Commissioning_VoltageLimitTicks = 0u;
  runRequested = true;
  breakRetryPending = false;
  breakRetryUsed = false;
  breakRetryAcknowledged = false;
  holdStarted = false;
  decelerating = false;
  pwmSeen = false;
  loopSeen = false;
  firstBandSeen = false;
  inBand = false;
  settleSeen = false;
  targetReached = false;
  reachedRpm = 0;
  iqLimitMs = 0;
  lastAccRampMs = 0u;
  lastDecRampMs = 0u;
  runCommandAt = HAL_GetTick();
  lastRun = active;
  lastTimingAt = runCommandAt;
  reportReason = "ACTIVE";
  reportAvailable = true;
  transitionStartEvent = false;
  transitionEndEvent = false;
  startDeadline = runCommandAt + active.rampMs + START_GRACE_MS;
}

bool Commissioning_RemoteRun(const CommissioningRemoteConfig *config)
{
  MotionConfig next;
  State_t state;
  int16_t signedRpm;
  if (config == NULL) return false;
  Defaults(&next);
  next.mode = config->mode == 0u ? MODE_CL : MODE_OPEN;
  next.fastStart = config->profile != 0u;
  next.direction = config->rpm < 0 ? -1 : 1;
  next.rpm = config->rpm < 0 ? (int16_t)-config->rpm : config->rpm;
  next.iqRaw = config->iqRaw;
  next.rampMs = config->rampMs;
  next.accRpmS = config->accRpmS;
  next.decRpmS = config->decRpmS;
  next.runMs = config->runMs;
  next.idKp = config->idKp; next.idKi = config->idKi;
  next.iqKp = config->iqKp; next.iqKi = config->iqKi;
  next.speedKp = config->speedKp; next.speedKi = config->speedKi;
  if (!Validate(&next)) return false;
  state = MC_GetSTMStateMotor1();
  if (state == RUN)
  {
    uint32_t current = MC_GetMecSpeedReferenceMotor1();
    uint32_t delta;
    if (active.mode != next.mode || active.direction != next.direction) return false;
    if ((int32_t)current < 0) current = (uint32_t)-(int32_t)current;
    current *= 6u;
    delta = next.rpm > current ? (uint32_t)next.rpm - current : current - (uint32_t)next.rpm;
    active = next;
    ApplyConfig();
    signedRpm = (int16_t)(active.direction * (active.rpm / 6));
    MC_ProgramSpeedRampMotor1(signedRpm, RateToDurationMs(delta, active.accRpmS));
    return true;
  }
  if (state != IDLE) return false;
  active = next;
  QueueStartRamp();
  if (!MC_StartMotor1()) return false;
  ArmRunTracking();
  return true;
}

bool Commissioning_RemoteRampStop(uint16_t decRpmS)
{
  int32_t directionRpm;
  if (MC_GetSTMStateMotor1() != RUN || active.mode != MODE_CL ||
      decelerating || decRpmS < SPEED_RATE_MIN || decRpmS > SPEED_RATE_MAX) return false;
  directionRpm = (int32_t)active.direction * MC_GetMecSpeedReferenceMotor1() * 6;
  if (directionRpm <= CL_RPM_MIN)
  {
    StopNow("RAMPSTOP_DONE");
    return true;
  }
  active.decRpmS = decRpmS;
  lastDecRampMs = RateToDurationMs((uint32_t)(directionRpm - CL_RPM_MIN), decRpmS);
  MC_ProgramSpeedRampMotor1((int16_t)(active.direction * (CL_RPM_MIN / 6)), lastDecRampMs);
  decelerating = true;
  decelDeadline = HAL_GetTick() + lastDecRampMs + DEC_SETTLE_GRACE_MS;
  return true;
}

void Commissioning_RemoteStop(void)
{
  productFaultClearRequested = true;
  StopNow("COMMAND");
}

static uint16_t RemoteElapsed(bool available, uint32_t eventAt)
{
  uint32_t elapsed;
  if (!available) return UINT16_MAX;
  elapsed = eventAt - runCommandAt;
  return elapsed > UINT16_MAX - 1u ? UINT16_MAX - 1u : (uint16_t)elapsed;
}

static uint16_t RemoteStopReason(void)
{
  if (runRequested) return 0u;
  if (!reportAvailable) return 1u;
  if (!strcmp(reportReason, "COMMAND")) return 2u;
  if (!strcmp(reportReason, "TIMEOUT")) return 3u;
  if (!strcmp(reportReason, "FAULT")) return 4u;
  if (!strcmp(reportReason, "START_TIMEOUT")) return 5u;
  if (!strcmp(reportReason, "UNEXPECTED_STATE")) return 6u;
  if (!strcmp(reportReason, "RAMPSTOP_DONE")) return 7u;
  if (!strcmp(reportReason, "RAMPSTOP_LIMIT")) return 8u;
  if (!strcmp(reportReason, "BREAK_RETRY_ABORT")) return 9u;
  if (!strcmp(reportReason, "BREAK_ACK_FAILED")) return 10u;
  if (!strcmp(reportReason, "BREAK_RESTART_REJECTED")) return 11u;
  return 255u;
}

void Commissioning_GetRemoteStatus(CommissioningRemoteStatus *status)
{
  State_t state;
  if (status == NULL) return;
  state = MC_GetSTMStateMotor1();
  status->flags = (uint16_t)((runRequested ? 1u : 0u) |
      ((state == RUN && active.mode == MODE_CL && STO_PLL_M1.IsAlgorithmConverged) ? 2u : 0u) |
      (breakRetryPending ? 4u : 0u) | (breakRetryUsed ? 8u : 0u));
  status->phase = !runRequested ? 0u :
      (breakRetryPending ? 4u : (decelerating ? 3u : (holdStarted ? 2u : 1u)));
  status->targetRpm = runRequested ? (int16_t)(active.direction * active.rpm) : 0;
  status->referenceRpm = (runRequested && state == RUN)
      ? (int16_t)((int32_t)MC_GetMecSpeedReferenceMotor1() * 6) : 0;
  status->observerRpm = (int16_t)((int32_t)SPD_GetAvrgMecSpeedUnit(&STO_PLL_M1._Super) * 6);
  status->busVoltageV = BusVoltage();
  status->reportFlags = (uint16_t)((pwmSeen ? 1u : 0u) | (loopSeen ? 2u : 0u) |
      (firstBandSeen ? 4u : 0u) | (settleSeen ? 8u : 0u) |
      (targetReached ? 16u : 0u));
  status->startMs = RemoteElapsed(pwmSeen, pwmAt);
  status->loopMs = RemoteElapsed(loopSeen, loopAt);
  status->bandMs = RemoteElapsed(firstBandSeen, firstBandAt);
  status->settleMs = RemoteElapsed(settleSeen, settleAt);
  status->reachedMs = RemoteElapsed(targetReached, reachedAt);
  status->reachedRpm = (int16_t)reachedRpm;
  status->iqLimitMs = iqLimitMs;
  status->stopReason = RemoteStopReason();
}

static void ShowConfig(const char *tag, const MotionConfig *cfg)
{
  Reply("%s MODE=%s PROFILE=%s DIR=%s RPM=%d IQRAW=%d RAMP_MS=%u ACC=%u DEC=%u RUN_MS=%u "
        "IDKP=%d IDKI=%d IQKP=%d IQKI=%d SPKP=%d SPKI=%d\r\n",
        tag, cfg->mode == MODE_OPEN ? "OPEN" : "CL", cfg->fastStart ? "FAST" : "STANDARD",
        cfg->direction > 0 ? "CW" : "CCW", cfg->rpm, cfg->iqRaw,
        cfg->rampMs, cfg->accRpmS, cfg->decRpmS, cfg->runMs, cfg->idKp, cfg->idKi,
        cfg->iqKp, cfg->iqKi, cfg->speedKp, cfg->speedKi);
}

static void Status(void)
{
  State_t state = MC_GetSTMStateMotor1();
  qd_t iq = MC_GetIqdMotor1();
  qd_t ref = MC_GetIqdrefMotor1();
  uint16_t faultNow = MC_GetCurrentFaultsMotor1();
  uint16_t faultPast = MC_GetOccurredFaultsMotor1();
  int32_t rpm = (int32_t)MC_GetMecSpeedAverageMotor1() * 6;
  int32_t refRpm = (runRequested && state == RUN)
      ? (int32_t)MC_GetMecSpeedReferenceMotor1() * 6 : 0;
  int32_t obsRpm = (int32_t)SPD_GetAvrgMecSpeedUnit(&STO_PLL_M1._Super) * 6;
  bool observerActive = state == RUN && active.mode == MODE_CL;
  Reply("STATUS STATE=%u RUN_REQ=%u PHASE=%s MODE=%s SRC=%s SETABLE=%u OUT=%u "
        "VBUS=%u RPM=%ld TARGET_RPM=%d REF_RPM=%ld OBS_RPM=%ld OBS_VALID=%u "
        "ID=%d IQ=%d IDREF=%d IQREF=%d FAULT=0x%04X OCCURRED=0x%04X BKIN=%u\r\n",
        (unsigned)state, runRequested ? 1u : 0u,
        !runRequested ? "IDLE" : (decelerating ? "DECEL" : (holdStarted ? "HOLD" : "START")),
        active.mode == MODE_OPEN ? "OPEN" : "CL",
        observerActive ? "STO" : "VSS", IsStopped() ? 1u : 0u,
        (TIM1->BDTR & TIM_BDTR_MOE) != 0u ? 1u : 0u,
        BusVoltage(), (long)rpm, active.rpm, (long)refRpm, (long)obsRpm,
        STO_PLL_M1.IsAlgorithmConverged ? 1u : 0u,
        iq.d, iq.q, ref.d, ref.q,
        faultNow, faultPast,
        HAL_GPIO_ReadPin(M1_OCP_GPIO_Port, M1_OCP_Pin) == GPIO_PIN_SET ? 1u : 0u);
}

static void ObserverStatus(void)
{
  Reply("OBS CONVERGED=%u VARIANCE_OK=%u BEMF_OK=%u OBS_RPM=%ld "
        "BEMF_OBS=%ld BEMF_EST=%ld ANGLE=%d\r\n",
        STO_PLL_M1.IsAlgorithmConverged ? 1u : 0u,
        STO_PLL_IsVarianceTight(&STO_M1) ? 1u : 0u,
        STO_PLL_IsBemfConsistent(&STO_PLL_M1) ? 1u : 0u,
        (long)SPD_GetAvrgMecSpeedUnit(&STO_PLL_M1._Super) * 6,
        (long)STO_PLL_GetObservedBemfLevel(&STO_PLL_M1),
        (long)STO_PLL_GetEstimatedBemfLevel(&STO_PLL_M1),
        SPD_GetElAngle(&STO_PLL_M1._Super));
}

static void Baseline(void)
{
  int32_t sumA = 0, sumB = 0;
  int16_t minA = INT16_MAX, minB = INT16_MAX;
  int16_t maxA = INT16_MIN, maxB = INT16_MIN;
  unsigned i;
  if (!IsStopped()) { Send("ERR STOP first\r\n"); return; }
  for (i = 0u; i < 128u; ++i)
  {
    ab_t v = MC_GetIabMotor1();
    sumA += v.a; sumB += v.b;
    if (v.a < minA) minA = v.a;
    if (v.a > maxA) maxA = v.a;
    if (v.b < minB) minB = v.b;
    if (v.b > maxB) maxB = v.b;
    HAL_Delay(1u);
  }
  Reply("BASELINE GATES=OFF N=128 VBUS=%u IA_AVG=%ld IA_MIN=%d IA_MAX=%d "
        "IB_AVG=%ld IB_MIN=%d IB_MAX=%d BKIN=%u FAULT=0x%04X\r\n",
        BusVoltage(), (long)(sumA / 128), minA, maxA,
        (long)(sumB / 128), minB, maxB,
        HAL_GPIO_ReadPin(M1_OCP_GPIO_Port, M1_OCP_Pin) == GPIO_PIN_SET ? 1u : 0u,
        MC_GetCurrentFaultsMotor1());
}

static bool SetValue(MotionConfig *cfg, const char *name, long value)
{
  if (value < 0 || value > UINT16_MAX) return false;
  if (strcmp(name, "RUN_MS") != 0 && strcmp(name, "ACC") != 0 &&
      strcmp(name, "DEC") != 0 && value > INT16_MAX) return false;
  if (!strcmp(name, "RPM")) cfg->rpm = (int16_t)value;
  else if (!strcmp(name, "IQRAW")) cfg->iqRaw = (int16_t)value;
  else if (!strcmp(name, "RAMP_MS")) cfg->rampMs = (uint16_t)value;
  else if (!strcmp(name, "ACC")) cfg->accRpmS = (uint16_t)value;
  else if (!strcmp(name, "DEC")) cfg->decRpmS = (uint16_t)value;
  else if (!strcmp(name, "RUN_MS")) cfg->runMs = (uint16_t)value;
  else if (!strcmp(name, "IDKP")) cfg->idKp = (int16_t)value;
  else if (!strcmp(name, "IDKI")) cfg->idKi = (int16_t)value;
  else if (!strcmp(name, "IQKP")) cfg->iqKp = (int16_t)value;
  else if (!strcmp(name, "IQKI")) cfg->iqKi = (int16_t)value;
  else if (!strcmp(name, "SPKP")) cfg->speedKp = (int16_t)value;
  else if (!strcmp(name, "SPKI")) cfg->speedKi = (int16_t)value;
  else return false;
  return true;
}

static void ProcessLine(char *line)
{
  char *scan;
  char *cmd;
  char *arg1;
  char *arg2;

  /* The commissioning console is operator-facing: accept either case and
     harmless surrounding whitespace for every command and argument. */
  for (scan = line; *scan != '\0'; ++scan)
  {
    *scan = (char)toupper((unsigned char)*scan);
  }
  cmd = strtok(line, " \t");
  arg1 = strtok(NULL, " \t");
  arg2 = strtok(NULL, " \t");
  if (cmd == NULL) return;
  if ((!strcmp(cmd, "ACC") || !strcmp(cmd, "DEC")) && arg1 && !arg2)
  {
    arg2 = arg1;
    arg1 = cmd;
    cmd = "SET";
  }

  if (!strcmp(cmd, "PING")) Send("PONG\r\n");
  else if (!strcmp(cmd, "INFO"))
    Reply("INFO F1ZL %s %s %s STM32G431 MCSDK5.4.8 %s %s\r\n",
          FW_VERSION, MOTOR_PROFILE, BOARD_PROFILE, __DATE__, __TIME__);
  else if (!strcmp(cmd, "HELP"))
    Send("CMD INFO|STATUS|OBS|DIAG|REPORT|SAFETY|CFG|MODE|PROFILE|DIR|SET|ACC|DEC|DFLT|BASELINE|RUN|RAMPSTOP|STOP|FAULT|ACK\r\n");
  else if (!strcmp(cmd, "STATUS")) Status();
  else if (!strcmp(cmd, "OBS")) ObserverStatus();
  else if (!strcmp(cmd, "DIAG"))
    Reply("DIAG VQ_PRE=%d VD_PRE=%d VQ_OUT=%d VD_OUT=%d VMODULE_MAX=%u VLIMIT_TICKS=%lu\r\n",
          Commissioning_VqBeforeLimit, Commissioning_VdBeforeLimit,
          Commissioning_VqApplied, Commissioning_VdApplied,
          CircleLimitationM1.MaxModule, (unsigned long)Commissioning_VoltageLimitTicks);
  else if (!strcmp(cmd, "REPORT")) Report();
  else if (!strcmp(cmd, "CFG")) ShowConfig("ACTIVE", &active);
  else if (!strcmp(cmd, "SAFETY"))
    Reply("SAFETY BUS_ARM=%u..%uV OPEN_RPM=%d..%d CL_RPM=%d..%d IQRAW=%d..%d "
          "CL_START_MS_MIN=%u ACC_DEC_RPM_S=%u..%u RUN_MS_MAX=%u PB14=UNCONNECTED HEATER=DISABLED\r\n",
          ARM_BUS_MIN_V, ARM_BUS_MAX_V, OPEN_RPM_MIN, OPEN_RPM_MAX,
          CL_RPM_MIN, CL_RPM_MAX, IQ_RAW_MIN, IQ_RAW_MAX,
          CL_RAMP_MS_MIN, SPEED_RATE_MIN, SPEED_RATE_MAX, RUN_MS_MAX);
  else if (!strcmp(cmd, "FAULT")) Status();
  else if (!strcmp(cmd, "BASELINE")) Baseline();
  else if (!strcmp(cmd, "STOP")) StopNow("COMMAND");
  else if (!strcmp(cmd, "RAMPSTOP"))
  {
    int16_t targetUnit;
    int32_t currentRefRpm;
    if (!runRequested || active.mode != MODE_CL || MC_GetSTMStateMotor1() != RUN)
    {
      Send("ERR RAMPSTOP requires CL RUN\r\n");
      return;
    }
    if (decelerating) { Send("ERR RAMPSTOP already active\r\n"); return; }
    targetUnit = (int16_t)(active.direction * (CL_RPM_MIN / 6));
    currentRefRpm = (int32_t)active.direction * MC_GetMecSpeedReferenceMotor1() * 6;
    if (currentRefRpm <= CL_RPM_MIN) { StopNow("RAMPSTOP_DONE"); return; }
    lastDecRampMs = RateToDurationMs((uint32_t)(currentRefRpm - CL_RPM_MIN), active.decRpmS);
    MC_ProgramSpeedRampMotor1(targetUnit, lastDecRampMs);
    decelerating = true;
    decelDeadline = HAL_GetTick() + lastDecRampMs + DEC_SETTLE_GRACE_MS;
    Reply("OK RAMPSTOP TARGET_RPM=%d DEC=%u RPM/S RAMP_MS=%u; STOP immediate\r\n",
          active.direction * (int16_t)((CL_RPM_MIN / 6) * 6), active.decRpmS,
          lastDecRampMs);
  }
  else if (!strcmp(cmd, "ACK"))
  {
    if (breakRetryPending) Send("ERR retry pending; STOP first\r\n");
    else if (MC_AcknowledgeFaultMotor1()) Send("OK ACK\r\n"); else Send("ERR ACK\r\n");
  }
  else if (!strcmp(cmd, "DFLT"))
  {
    if (!IsStopped()) Send("ERR STOP first\r\n");
    else { Defaults(&active); ShowConfig("ACTIVE", &active); }
  }
  else if (!strcmp(cmd, "MODE") && arg1)
  {
    if (!IsStopped()) Send("ERR STOP first\r\n");
    else if (!strcmp(arg1, "OPEN")) { active.mode = MODE_OPEN; active.fastStart = false; Send("OK MODE OPEN active\r\n"); }
    else if (!strcmp(arg1, "CL"))
    {
      active.mode = MODE_CL;
      active.iqRaw = NOMINAL_CURRENT;
      if (active.rampMs < CL_RAMP_MS_MIN) active.rampMs = CL_RAMP_MS_MIN;
      Send("OK MODE CL active; IQRAW=10922 old default; SET RPM/RAMP_MS/ACC then RUN\r\n");
    }
    else Send("ERR MODE OPEN\r\n");
  }
  else if (!strcmp(cmd, "PROFILE") && arg1)
  {
    if (!IsStopped()) Send("ERR STOP first\r\n");
    else if (!strcmp(arg1, "FAST"))
    {
      active.mode = MODE_CL;
      active.fastStart = true;
      active.rpm = 10000;
      active.iqRaw = NOMINAL_CURRENT;
      active.rampMs = 1000u;
      active.accRpmS = 10000u;
      active.decRpmS = 10000u;
      active.runMs = 20000u;
      ShowConfig("ACTIVE", &active);
    }
    else if (!strcmp(arg1, "STANDARD"))
    {
      active.mode = MODE_CL;
      active.fastStart = false;
      active.rpm = 3000;
      active.iqRaw = NOMINAL_CURRENT;
      active.rampMs = 3000u;
      active.accRpmS = 100u;
      active.decRpmS = 3000u;
      active.runMs = 20000u;
      ShowConfig("ACTIVE", &active);
    }
    else Send("ERR PROFILE STANDARD|FAST\r\n");
  }
  else if (!strcmp(cmd, "DIR") && arg1)
  {
    if (!IsStopped()) Send("ERR STOP first\r\n");
    else if (!strcmp(arg1, "CW")) { active.direction = 1; Send("OK DIR CW active\r\n"); }
    else if (!strcmp(arg1, "CCW")) { active.direction = -1; Send("OK DIR CCW active\r\n"); }
    else Send("ERR DIR CW|CCW\r\n");
  }
  else if (!strcmp(cmd, "SET") && arg1 && arg2)
  {
    char *end = NULL;
    long value;
    MotionConfig candidate = active;
    value = strtol(arg2, &end, 0);
    if (end == arg2 || *end != '\0' || !SetValue(&candidate, arg1, value)) Send("ERR SET name/value\r\n");
    else if (!Validate(&candidate)) Send("ERR RANGE; configuration unchanged\r\n");
    else if (runRequested || !IsStopped())
    {
      if (!strcmp(arg1, "RPM") && runRequested && active.mode == MODE_CL &&
          active.fastStart && MC_GetSTMStateMotor1() == RUN && !decelerating)
      {
        int32_t fromRpm = (int32_t)active.direction * MC_GetMecSpeedReferenceMotor1() * 6;
        uint32_t deltaRpm = (uint32_t)labs((long)(candidate.rpm - fromRpm));
        bool accelerating = candidate.rpm >= fromRpm;
        uint16_t rate = accelerating ? active.accRpmS : active.decRpmS;
        uint16_t duration = RateToDurationMs(deltaRpm, rate);
        MC_ProgramSpeedRampMotor1((int16_t)(active.direction * candidate.rpm / 6), duration);
        active.rpm = candidate.rpm;
        lastRun.rpm = candidate.rpm;
        firstBandSeen = false;
        inBand = false;
        settleSeen = false;
        targetReached = false;
        reachedRpm = 0;
        Reply("OK SET RPM=%d active; %s FROM_RPM=%ld RATE=%u RPM/S RAMP_MS=%u\r\n",
              active.rpm, accelerating ? "ACC" : "DEC", (long)fromRpm, rate, duration);
      }
      else Send("ERR STOP first\r\n");
    }
    else { active = candidate; Reply("OK SET %s=%ld active\r\n", arg1, value); }
  }
  else if (!strcmp(cmd, "COMMIT"))
  {
    if (!IsStopped()) Send("ERR STOP first\r\n");
    else Send("OK COMMIT no longer required; settings already active\r\n");
  }
  else if (!strcmp(cmd, "ARM") && arg1 && !strcmp(arg1, "M1"))
  {
    Send("OK ARM M3 no longer required; RUN checks bus and faults\r\n");
  }
  else if (!strcmp(cmd, "RUN"))
  {
    uint16_t bus = BusVoltage();
    if (!IsStopped()) { Send("ERR STOP first\r\n"); return; }
    if (!Validate(&active)) { Send("ERR CONFIG invalid\r\n"); return; }
    if (MC_GetCurrentFaultsMotor1() != 0u) { Send("ERR active fault\r\n"); return; }
    if (bus < ARM_BUS_MIN_V || bus > ARM_BUS_MAX_V)
    {
      Reply("ERR VBUS=%u expected %u..%u\r\n", bus, ARM_BUS_MIN_V, ARM_BUS_MAX_V);
      return;
    }
    Reply("READY RUN MODE=%s PROFILE=%s DIR=%s RPM=%d; PWM still off\r\n",
          active.mode == MODE_OPEN ? "OPEN" : "CL",
          active.fastStart ? "FAST" : "STANDARD",
          active.direction > 0 ? "CW" : "CCW", active.rpm);
    QueueStartRamp();
    if (!MC_StartMotor1()) Send("ERR START rejected\r\n");
    else
    {
      ArmRunTracking();
      Reply("OK RUN MODE=%s PROFILE=%s DIR=%s RPM=%d IQRAW=%d STARTUP_MS=%u BLEND_MS=%u OBS_START_RPM=%u ACC=%u RPM/S TIMEOUT_MS=%u\r\n",
            active.mode == MODE_OPEN ? "OPEN" : "CL",
            active.fastStart ? "FAST" : "STANDARD",
            active.direction > 0 ? "CW" : "CCW", active.rpm,
            active.iqRaw, active.rampMs, transitionDurationMs, observerStartRpm,
            active.accRpmS, active.runMs);
    }
  }
  else Send("ERR COMMAND; HELP\r\n");
}

void Commissioning_Init(void)
{
  Defaults(&active);
  runRequested = false;
  holdStarted = false;
  reportAvailable = false;
  Send("\r\nF1 ZHONGBAI LAB " FW_VERSION " " MOTOR_PROFILE "\r\nSAFE START: outputs off; INFO then STATUS/SAFETY/CFG\r\n");
}

void Commissioning_RxByteISR(uint8_t value)
{
  /* The host sends CRLF. Do not mark the command ready on CR because replying
     while the USB/RS485 adapter is still transmitting LF causes bus contention. */
  if (value == '\r')
  {
    return;
  }
  if (value == '\n')
  {
    if (rxLength != 0u && !lineReady)
    {
      rxLine[rxLength] = '\0';
      lineReady = true;
    }
    return;
  }
  if (lineReady) return;
  if (rxLength < RX_LINE_SIZE - 1u) rxLine[rxLength++] = (char)value;
  else rxOverflow = true;
}

void Commissioning_Task(void)
{
  char local[RX_LINE_SIZE];
  if (productFaultClearRequested)
  {
    State_t state = MC_GetSTMStateMotor1();
    uint16_t currentFault = MC_GetCurrentFaultsMotor1();
    uint16_t occurredFault = MC_GetOccurredFaultsMotor1();
    if (state == IDLE)
      productFaultClearRequested = false;
    else if (currentFault == 0u && state == FAULT_OVER)
    {
      if ((occurredFault & (uint16_t)~PRODUCT_RECOVERABLE_FAULTS) == 0u)
      {
        if (MC_AcknowledgeFaultMotor1()) productFaultClearRequested = false;
      }
      else productFaultClearRequested = false;
    }
  }
  if (transitionStartEvent)
  {
    transitionStartEvent = false;
    Reply("EVENT BLEND_START MS=%lu FORCED_RPM=%ld OBS_RPM=%ld BLEND_MS=%u\r\n",
          (unsigned long)(transitionStartAt - runCommandAt),
          (long)transitionStartForcedUnit * 6,
          (long)transitionStartObservedUnit * 6,
          transitionDurationMs);
  }
  if (transitionEndEvent)
  {
    transitionEndEvent = false;
    Reply("EVENT BLEND_END MS=%lu FORCED_RPM=%ld OBS_RPM=%ld\r\n",
          (unsigned long)(transitionEndAt - runCommandAt),
          (long)transitionEndForcedUnit * 6,
          (long)transitionEndObservedUnit * 6);
  }
  if (fallbackStartEvent)
  {
    fallbackStartEvent = false;
    Reply("EVENT FAST_BLEND_REJECT MS=%lu FORCED_RPM=%ld OBS_RPM=%ld; ST_FALLBACK=ACTIVE\r\n",
          (unsigned long)(fallbackStartAt - runCommandAt),
          (long)fallbackStartForcedUnit * 6,
          (long)fallbackStartObservedUnit * 6);
  }
  if (fallbackHandoffEvent)
  {
    fallbackHandoffEvent = false;
    Reply("EVENT ST_FALLBACK_HANDOFF MS=%lu FORCED_RPM=%ld OBS_RPM=%ld TRANSITION_MS=0 POLICY=UNCONDITIONAL\r\n",
          (unsigned long)(fallbackHandoffAt - runCommandAt),
          (long)fallbackHandoffForcedUnit * 6,
          (long)fallbackHandoffObservedUnit * 6);
  }
  if (runRequested && breakRetryPending)
  {
    uint32_t now = HAL_GetTick();
    State_t state = MC_GetSTMStateMotor1();
    uint16_t currentFault = MC_GetCurrentFaultsMotor1();
    uint16_t occurredFault = MC_GetOccurredFaultsMotor1();
    uint16_t bus = BusVoltage();
    bool breakClear = HAL_GPIO_ReadPin(M1_OCP_GPIO_Port, M1_OCP_Pin) == GPIO_PIN_SET;
    bool outputOff = (TIM1->BDTR & TIM_BDTR_MOE) == 0u;
    bool busReady = bus >= ARM_BUS_MIN_V && bus <= ARM_BUS_MAX_V;
    if ((uint32_t)(now - breakRetryAt) >= BREAK_RETRY_WAIT_MS ||
        (occurredFault & (uint16_t)~MC_BREAK_IN) != 0u || !busReady || !outputOff)
    {
      StopNow("BREAK_RETRY_ABORT");
    }
    else if (!breakRetryAcknowledged)
    {
      if (!breakClear || currentFault != 0u || state != FAULT_OVER)
        breakClearSince = 0u;
      else
      {
        if (breakClearSince == 0u) breakClearSince = now;
        if ((uint32_t)(now - breakClearSince) >= BREAK_RETRY_QUIET_MS)
        {
          if (!MC_AcknowledgeFaultMotor1()) StopNow("BREAK_ACK_FAILED");
          else
          {
            breakRetryAcknowledged = true;
            Send("EVENT BREAK_ACK; waiting for IDLE\r\n");
          }
        }
      }
    }
    else if (state == IDLE && breakClear && currentFault == 0u && occurredFault == 0u)
    {
      QueueStartRamp();
      if (!MC_StartMotor1()) StopNow("BREAK_RESTART_REJECTED");
      else
      {
        breakRetryPending = false;
        runCommandAt = now;
        startDeadline = now + active.rampMs + START_GRACE_MS;
        lastTimingAt = now;
        pwmSeen = false;
        loopSeen = false;
        firstBandSeen = false;
        inBand = false;
        settleSeen = false;
        targetReached = false;
        reachedRpm = 0;
        iqLimitMs = 0;
        Send("EVENT BREAK_RETRY RUN attempt=2\r\n");
      }
    }
  }
  else if (runRequested)
  {
    uint32_t now = HAL_GetTick();
    State_t state = MC_GetSTMStateMotor1();
    if (!pwmSeen && (TIM1->BDTR & TIM_BDTR_MOE) != 0u)
    {
      pwmSeen = true;
      pwmAt = now;
    }
    if (active.mode == MODE_CL && state == RUN && !decelerating)
    {
      int32_t rpm = (int32_t)MC_GetMecSpeedAverageMotor1() * 6;
      int32_t referenceRpm = (int32_t)MC_GetMecSpeedReferenceMotor1() * 6;
      int32_t targetRpm = (int32_t)active.direction * active.rpm;
      int32_t tolerance = active.rpm / 20;
      bool valid = STO_PLL_M1.IsAlgorithmConverged;
      bool speedInBand;
      if (tolerance < 100) tolerance = 100;
      speedInBand = valid && labs((long)(rpm - targetRpm)) <= tolerance;
      if (!loopSeen)
      {
        loopSeen = true;
        loopAt = now;
        Reply("EVENT LOOP MS=%lu RPM=%ld\r\n",
              (unsigned long)(now - runCommandAt), (long)rpm);
      }
      if (!settleSeen && labs((long)(referenceRpm - targetRpm)) <= tolerance)
      {
        settleSeen = true;
        settleAt = now;
      }
      if (speedInBand)
      {
        if (!firstBandSeen) { firstBandSeen = true; firstBandAt = now; }
        if (!inBand) { inBand = true; bandSince = now; }
        if (!targetReached && settleSeen && (uint32_t)(now - bandSince) >= TARGET_BAND_MS)
        {
          targetReached = true;
          reachedAt = now;
          reachedRpm = rpm;
          Reply("EVENT TARGET_REACHED MS=%lu RPM=%ld BAND_MS=%u\r\n",
                (unsigned long)(now - runCommandAt), (long)rpm, TARGET_BAND_MS);
        }
      }
      else inBand = false;
      if (abs(MC_GetIqdrefMotor1().q) >= (active.iqRaw * 98) / 100)
        iqLimitMs += now - lastTimingAt;
    }
    lastTimingAt = now;
    if (MC_GetCurrentFaultsMotor1() != 0u || state == FAULT_NOW || state == FAULT_OVER)
    {
      uint16_t occurredFault = MC_GetOccurredFaultsMotor1();
      if (!breakRetryUsed && active.mode == MODE_CL && !holdStarted && !loopSeen &&
          (uint32_t)(now - runCommandAt) <= BREAK_RETRY_WINDOW_MS &&
          occurredFault == MC_BREAK_IN &&
          (TIM1->BDTR & TIM_BDTR_MOE) == 0u)
      {
        breakRetryPending = true;
        breakRetryUsed = true;
        breakRetryAcknowledged = false;
        breakRetryAt = now;
        breakClearSince = 0u;
        Reply("EVENT BREAK_RETRY_PENDING MS=%lu; output off, wait for clear\r\n",
              (unsigned long)(now - runCommandAt));
      }
      else StopNow("FAULT");
    }
    else if (!holdStarted && state == RUN)
    {
      if (active.mode == MODE_CL)
      {
        int32_t referenceRpm = (int32_t)MC_GetMecSpeedReferenceMotor1() * 6;
        int32_t targetRpm = (int32_t)active.direction * active.rpm;
        uint32_t deltaRpm = (uint32_t)labs((long)(targetRpm - referenceRpm));
        lastAccRampMs = RateToDurationMs(deltaRpm, active.accRpmS);
        MC_ProgramSpeedRampMotor1((int16_t)(targetRpm / 6), lastAccRampMs);
        Reply("EVENT ACC_RAMP FROM_RPM=%ld TARGET_RPM=%ld ACC=%u RPM/S RAMP_MS=%u\r\n",
              (long)referenceRpm, (long)targetRpm, active.accRpmS, lastAccRampMs);
      }
      holdStarted = true;
      runDeadline = now + active.runMs;
    }
    else if (!holdStarted && (int32_t)(now - startDeadline) >= 0)
      StopNow("START_TIMEOUT");
    else if (holdStarted && state != RUN)
      StopNow("UNEXPECTED_STATE");
    else if (holdStarted && active.runMs != 0u && (int32_t)(now - runDeadline) >= 0)
      StopNow("TIMEOUT");
    else if (decelerating)
    {
      int32_t directionRpm = (int32_t)active.direction * MC_GetMecSpeedAverageMotor1() * 6;
      int32_t directionRefRpm = (int32_t)active.direction * MC_GetMecSpeedReferenceMotor1() * 6;
      if (directionRefRpm <= CL_RPM_MIN && directionRpm >= 0 &&
          directionRpm <= CL_RPM_MIN + CL_RPM_MIN / 20)
        StopNow("RAMPSTOP_DONE");
      else if ((int32_t)(now - decelDeadline) >= 0)
        StopNow("RAMPSTOP_LIMIT");
    }
  }
  if (rxOverflow)
  {
    __disable_irq(); rxLength = 0u; lineReady = false; rxOverflow = false; __enable_irq();
    Send("ERR LINE too long\r\n");
  }
  if (lineReady)
  {
    __disable_irq();
    memcpy(local, (const void *)rxLine, rxLength + 1u);
    rxLength = 0u; lineReady = false;
    __enable_irq();
    ProcessLine(local);
  }
}

bool Commissioning_OpenLoopActive(void)
{
  return runRequested && active.mode == MODE_OPEN;
}

int16_t Commissioning_OpenLoopIq(void)
{
  return (int16_t)(active.direction * active.iqRaw);
}

uint16_t Commissioning_TransitionDurationMs(void)
{
  return transitionDurationMs;
}

uint16_t Commissioning_ObserverStartRpm(void)
{
  return observerStartRpm;
}

void Commissioning_TransitionStarted(int16_t forcedSpeedUnit, int16_t observedSpeedUnit)
{
  transitionStartForcedUnit = forcedSpeedUnit;
  transitionStartObservedUnit = observedSpeedUnit;
  transitionStartAt = HAL_GetTick();
  transitionStartEvent = true;
}

void Commissioning_TransitionCompleted(int16_t forcedSpeedUnit, int16_t observedSpeedUnit)
{
  transitionEndForcedUnit = forcedSpeedUnit;
  transitionEndObservedUnit = observedSpeedUnit;
  transitionEndAt = HAL_GetTick();
  transitionEndEvent = true;
}

void Commissioning_FallbackStarted(int16_t forcedSpeedUnit, int16_t observedSpeedUnit)
{
  fallbackStartForcedUnit = forcedSpeedUnit;
  fallbackStartObservedUnit = observedSpeedUnit;
  fallbackStartAt = HAL_GetTick();
  fallbackStartEvent = true;
}

void Commissioning_FallbackHandoff(int16_t forcedSpeedUnit, int16_t observedSpeedUnit)
{
  fallbackHandoffForcedUnit = forcedSpeedUnit;
  fallbackHandoffObservedUnit = observedSpeedUnit;
  fallbackHandoffAt = HAL_GetTick();
  fallbackHandoffEvent = true;
}
