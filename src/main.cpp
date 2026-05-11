#include "Arduino.h"
#include "Modcom.h"
#include "ModbusReg.h"

ModbusReg Mdata;
Modcom mc(Mdata, 0);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mc.begin();
}

void loop() {
  // for (int i = 0; i < outreg; i++) { 
  //   Mdata.outregs[i] = i;
  // }
  for (int i = 0; i < outreg; i++) { 
    Mdata.outregs[i] = Mdata.inregs[i];
    //Mdata.outregs[i] = i;
  }

  for (int i = 0; i < outcoil; i++) {
    Mdata.outcoils[i] = Mdata.incoils[i];
  }

  mc.poll();
  yield();
}