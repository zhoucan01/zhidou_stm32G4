#include "usrdug.h"
#include "product_modbus.h"
#include "usart.h"
#include "mc_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

extern int HeatPowerTable[30][2];
extern int TotolSwitch, HeatBarSwitch, HeatRollSwitch, TIM2HeatLevel, TIM3HeatLevel;
extern volatile uint32_t HeatZeroCount, HeatTimerStartCount, HeatTimerExpireCount;

/* Single producer (USART1 ISR), single consumer (main loop). */
#define RX_SIZE 256U
static uint8_t rx[RX_SIZE];
static volatile uint16_t rxHead, rxTail;
static volatile uint8_t rxLost;
static char command[96];
static uint16_t commandLength;
static uint8_t discardLine;
static uint32_t lastByteMs;

void UsrDug_Init(void)
{
  rxHead = rxTail = 0U;
  rxLost = 0U; commandLength = 0U; discardLine = 0U;
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
  __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void UsrDug_RxByteISR(uint8_t byte)
{
  uint16_t next = (uint16_t)((rxHead + 1U) % RX_SIZE);
  if(next == rxTail || byte == 0U) { rxLost = 1U; return; }
  rx[rxHead] = byte;
  __DMB(); rxHead = next;
}

void UsrDug_Task(void)
{
  if(rxLost)
  {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    rxTail = rxHead; rxLost = 0U;
    if(mask == 0U) __enable_irq();
    commandLength = 0U; discardLine = 1U;
    lastByteMs = HAL_GetTick();
  }
  while(rxTail != rxHead)
  {
    uint8_t byte = rx[rxTail];
    __DMB(); rxTail = (uint16_t)((rxTail + 1U) % RX_SIZE);
    if(HAL_GetTick() - lastByteMs > 1000U) { commandLength = 0U; discardLine = 0U; }
    lastByteMs = HAL_GetTick();
    if(byte == '\r' || byte == '\n')
    {
      if(discardLine)
      {
        const char *error = "NB+ERR,line-overflow-or-rx-error\r\n";
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)error, (uint16_t)strlen(error), 100U);
      }
      else if(commandLength != 0U)
      {
        command[commandLength] = '\0'; UsrDug_HandleLine(command);
      }
      commandLength = 0U; discardLine = 0U;
    }
    else if(!discardLine)
    {
      if(commandLength < sizeof(command) - 1U) command[commandLength++] = (char)byte;
      else { discardLine = 1U; commandLength = 0U; }
    }
  }
}

static char *trim(char *s)
{
  char *end;
  while(*s == ' ' || *s == '\t') s++;
  end = s + strlen(s);
  while(end > s && (end[-1] == ' ' || end[-1] == '\t')) --end;
  *end = '\0';
  return s;
}

static uint8_t number(char *s, long *value)
{
  char *end;
  s = trim(s);
  errno = 0;
  *value = strtol(s, &end, 10);
  return end != s && *end == '\0' && errno != ERANGE;
}

void UsrDug_HandleLine(char *line)
{
  char reply[224], *key, *val, *eq, *p;
  const char *error = "syntax";
  long a, b, c;
  uint8_t ok = 0U;
  line = trim(line);
  for(p = line; *p; p++)
    if(*p >= 'A' && *p <= 'Z') *p = (char)(*p + ('a' - 'A'));
  if(strncmp(line, "nb+", 3U) != 0) goto done;
  eq = strchr(line + 3, '=');
  if(eq == NULL) goto done;
  *eq = '\0'; key = trim(line + 3); val = trim(eq + 1);
  if(strcmp(key, "table") == 0)
  {
    char *second = strchr(val, ',');
    char *third;
    uint8_t setArr, setPsc;
    uint32_t mask;
    if(second == NULL) goto done;
    *second++ = '\0'; third = strchr(second, ',');
    if(third != NULL) *third++ = '\0';
    second = trim(second);
    setArr = *second != '\0';
    setPsc = third != NULL;
    if(!number(val, &a) || (!setArr && !setPsc)) goto done;
    if(setArr && !number(second, &b)) goto done;
    if(setPsc && !number(third, &c)) goto done;
    error = "table-range";
    /* Only initialized, reachable levels; PSC and shared TIM3 ARR are 16-bit. */
    if(a < 1 || a > 30 || (setArr && (b < 1 || b > 65535)) ||
       (setPsc && (c < 0 || c > 65535))) goto done;
    mask = __get_PRIMASK(); __disable_irq();
    if(setArr) HeatPowerTable[a - 1][0] = (int)b;
    if(setPsc) HeatPowerTable[a - 1][1] = (int)c;
    if(mask == 0U) __enable_irq();
    if(setArr && setPsc)
      snprintf(reply, sizeof(reply), "NB+table=%ld,%ld,%ld,OK\r\n", a, b, c);
    else if(setArr)
      snprintf(reply, sizeof(reply), "NB+table=%ld,%ld,OK\r\n", a, b);
    else
      snprintf(reply, sizeof(reply), "NB+table=%ld,,%ld,OK\r\n", a, c);
    ok = 1U;
  }
  else
  {
    if(strcmp(key, "motor") == 0 && strcmp(val, "?") == 0)
    {
      snprintf(reply, sizeof(reply), "NB+motor,state=%u,fault=0x%04X,history=0x%04X,rpm=%ld\r\n",
        (unsigned)MC_GetSTMStateMotor1(), (unsigned)MC_GetCurrentFaultsMotor1(),
        (unsigned)MC_GetOccurredFaultsMotor1(), (long)MC_GetMecSpeedAverageMotor1() * 6L);
      ok = 1U; goto done;
    }
    if((strcmp(key, "heat1") == 0 || strcmp(key, "heat2") == 0) && strcmp(val, "?") == 0)
    {
      uint32_t mask = __get_PRIMASK();
      int level, enabled, total;
      uint32_t odr, cr1, dier;
      TIM_TypeDef *timer = key[4] == '1' ? TIM2 : TIM3;
      __disable_irq();
      level = key[4] == '1' ? TIM2HeatLevel : TIM3HeatLevel;
      enabled = key[4] == '1' ? HeatBarSwitch : HeatRollSwitch;
      total = TotolSwitch; odr = GPIOB->ODR;
      cr1 = timer->CR1; dier = timer->DIER;
      if(mask == 0U) __enable_irq();
      snprintf(reply, sizeof(reply),
        "NB+%s,level=%d,total=%d,on=%d,out=%u,zero=%u,timer=%u,irq=%u,ms=%lu,zc=%lu,start1=%lu,fire1=%lu\r\n",
        key, level, total, enabled,
        (unsigned)((odr & (key[4] == '1' ? HeatBar_Pin : HeatRoll_Pin)) != 0U),
        (unsigned)(HAL_GPIO_ReadPin(Zero_GPIO_Port, Zero_Pin) != GPIO_PIN_RESET),
        (unsigned)((cr1 & TIM_CR1_CEN) != 0U),
        (unsigned)((dier & TIM_DIER_UIE) != 0U), (unsigned long)HAL_GetTick(),
        (unsigned long)HeatZeroCount, (unsigned long)HeatTimerStartCount,
        (unsigned long)HeatTimerExpireCount);
      ok = 1U; goto done;
    }
    if(!number(val, &a)) goto done;
    if(strcmp(key, "motor") == 0)
    {
      error = "motor-range-or-state";
      if(a < -20000 || a > 20000 || (a != 0 && a > -1000 && a < 1000))
      { error = "motor-range"; goto done; }
      if(!ProductModbus_DebugMotor((int16_t)a))
      {
        snprintf(reply, sizeof(reply), "NB+ERR,motor-rejected,state=%u,fault=0x%04X,history=0x%04X\r\n",
          (unsigned)MC_GetSTMStateMotor1(), (unsigned)MC_GetCurrentFaultsMotor1(),
          (unsigned)MC_GetOccurredFaultsMotor1());
        ok = 1U; goto done;
      }
    }
    else if(strcmp(key, "heat1") == 0 || strcmp(key, "heat2") == 0)
    {
      error = "heat-range-or-unavailable";
      if(a < 0 || a > 30 || !ProductModbus_DebugHeat((uint8_t)(key[4] - '0'), (uint16_t)a)) goto done;
    }
    else { error = "unknown-command"; goto done; }
    snprintf(reply, sizeof(reply), "NB+%s=%ld,OK\r\n", key, a);
    ok = 1U;
  }
done:
  if(!ok) snprintf(reply, sizeof(reply), "NB+ERR,%s\r\n", error);
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)reply, (uint16_t)strlen(reply), 100U);
}
