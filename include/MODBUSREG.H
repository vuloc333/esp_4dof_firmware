#include "Arduino.h"

#define inreg 50
#define outreg 50

#define incoil 100
#define outcoil 100


struct ModbusReg {
  uint16_t inregs[inreg];
  uint16_t outregs[outreg];
  bool incoils[incoil];
  bool outcoils[outcoil];
};