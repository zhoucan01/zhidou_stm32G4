#ifndef USRDUG_H
#define USRDUG_H
#include <stdint.h>
/* Main-loop only. A complete NB command excludes its CR/LF terminator. */
void UsrDug_HandleLine(char *line);
void UsrDug_Init(void);
void UsrDug_RxByteISR(uint8_t byte);
void UsrDug_Task(void);
#endif
