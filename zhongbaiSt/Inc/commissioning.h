#ifndef F1_COMMISSIONING_H
#define F1_COMMISSIONING_H

#include <stdbool.h>
#include <stdint.h>

void Commissioning_Init(void);
void Commissioning_Task(void);
void Commissioning_RxByteISR(uint8_t value);

typedef struct
{
  uint16_t mode;       /* 0=CL, 1=OPEN */
  uint16_t profile;    /* 0=STANDARD, 1=FAST */
  int16_t rpm;         /* signed direction */
  int16_t iqRaw;
  uint16_t rampMs;
  uint16_t accRpmS;
  uint16_t decRpmS;
  uint16_t runMs;
  int16_t idKp, idKi, iqKp, iqKi, speedKp, speedKi;
} CommissioningRemoteConfig;

typedef struct
{
  uint16_t flags;       /* bit0 run, bit1 observer valid, bit2 retry pending, bit3 retry used */
  uint16_t phase;       /* 0 idle, 1 start, 2 hold, 3 decel, 4 break retry */
  int16_t targetRpm;
  int16_t referenceRpm;
  int16_t observerRpm;
  uint16_t busVoltageV;
  uint16_t reportFlags; /* bit0 PWM, bit1 loop, bit2 band, bit3 settled, bit4 reached */
  uint16_t startMs;
  uint16_t loopMs;
  uint16_t bandMs;
  uint16_t settleMs;
  uint16_t reachedMs;
  int16_t reachedRpm;
  uint32_t iqLimitMs;
  uint16_t stopReason;
} CommissioningRemoteStatus;

/* Binary product-protocol bridge. It reuses all settable Motion Lab fields
 * without emitting ASCII on USART2. */
bool Commissioning_RemoteRun(const CommissioningRemoteConfig *config);
bool Commissioning_RemoteRampStop(uint16_t decRpmS);
void Commissioning_RemoteStop(void);
void Commissioning_GetRemoteStatus(CommissioningRemoteStatus *status);

/* Hooks used by the retained MCSDK state machine for bounded forced motion. */
bool Commissioning_OpenLoopActive(void);
int16_t Commissioning_OpenLoopIq(void);
uint16_t Commissioning_TransitionDurationMs(void);
uint16_t Commissioning_ObserverStartRpm(void);
void Commissioning_TransitionStarted(int16_t forcedSpeedUnit, int16_t observedSpeedUnit);
void Commissioning_TransitionCompleted(int16_t forcedSpeedUnit, int16_t observedSpeedUnit);
void Commissioning_FallbackStarted(int16_t forcedSpeedUnit, int16_t observedSpeedUnit);
void Commissioning_FallbackHandoff(int16_t forcedSpeedUnit, int16_t observedSpeedUnit);

/* Read-only FOC voltage diagnostics, written by the high-frequency current loop. */
extern volatile int16_t Commissioning_VqBeforeLimit;
extern volatile int16_t Commissioning_VdBeforeLimit;
extern volatile int16_t Commissioning_VqApplied;
extern volatile int16_t Commissioning_VdApplied;
extern volatile uint32_t Commissioning_VoltageLimitTicks;

#endif
