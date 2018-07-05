#include <stdio.h>

#include "terminal.h"
#include "commands.h"

const sTermEntry_t *term_entries[] =
{
      &hEntry,
      &helpEntry,
      &rebootEntry,
      &bootEntry,
      0
};
