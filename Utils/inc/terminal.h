#ifndef _TERMINAL_H
#define _TERMINAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "terminal_if.h"



typedef struct
{
   const char *cmd;
   const char *description;
   void (*cmdFunc)(sTerminalInterface_t *termIf, uint8_t argc, char **argv);
}sTermEntry_t;

void terminal_handleCommand(sTerminalInterface_t *termIf, char *cmd);
//HAL_StatusTypeDef terminal_setCommand(char *cmd, sTerminalInterface_t *termIf);
uint8_t terminal_run();
bool terminal_isActive();

extern const sTermEntry_t hEntry;
extern const sTermEntry_t helpEntry;
extern const sTermEntry_t rebootEntry;
extern const sTermEntry_t bootEntry;
extern const sTermEntry_t sleepEntry;

#ifdef __cplusplus
}
#endif

#endif
