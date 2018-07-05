#ifndef KSES_UTILITIES_TERMINAL_IF_H_
#define KSES_UTILITIES_TERMINAL_IF_H_

typedef struct {
	void (*printf)(const char *format, ...);
	void (*sleep)(void);
}sTerminalInterface_t;

#endif /* KSES_UTILITIES_TERMINAL_IF_H_ */
