#include "Arduino.h"
#include "Modcom.h"
#include "ModbusReg.h"
#include <math.h>
#include "Calc4dof.h"
#include "Storage.h"

ModbusReg Mdata;
Modcom mc(Mdata, 0);
int16_t Parameter[20];
StorageManager storage;
TaskHandle_t TaskModbus;

bool flag0 = false;
bool flag1 = false;
int long timer1 = 0;

float myAngles[4];
float myPos[3];

void ParameterSave() {
  for (int i = 0; i < sizeof(Parameter) / sizeof(Parameter[0]); i++) Parameter[i] = Mdata.inregs[i];
  
  flag1 = Mdata.incoils[0] && !flag0;

  Mdata.outcoils[0] = flag1;

  if(flag1) {
    //for (int i = 0; i < inreg; i++) Mdata.inregs[i] = i;
    storage.save(Parameter, sizeof(Parameter) / sizeof(Parameter[0]));
  }
}
void ParameterLoad() {
  storage.load(Parameter, sizeof(Parameter) / sizeof(Parameter[0]));
  for (int i = 0; i < sizeof(Parameter) / sizeof(Parameter[0]); i++) Mdata.outregs[i] = Parameter[i];
}

long t = 0;

void ModbusTask(void* parameter) {
  Calc4dof myArm(100.0, 100.0, 30.0, 50.0);
  while (1) {
    float tx = Mdata.inregs[10] / 100.0;
    float ty = Mdata.inregs[11] / 100.0;
    float tz = Mdata.inregs[12] / 100.0;
    // 2. Tính toán IK
    //if (myArm.computeIK(tx, ty, tz)) {
    bool IKvalid = myArm.computeIK(150.0, 0.0, 100.0);
    if (IKvalid) {
            // Lấy góc ra mảng 1 chiều
            myArm.getAnglesArray(myAngles);
            myArm.getFKArray(myPos);
            // Ví dụ: Đổ thẳng vào thanh ghi Modbus
            for(int i = 0; i < 4; i++) {
              Mdata.outregs[20 + i] = (int16_t)(myAngles[i] * 100);
              if (i < 3) Mdata.outregs[30 + i] = (int16_t)(myPos[i] * 100);
            }
        }

    if(millis() - t > 1000) {
      t = millis();
      Serial.print("Target: ");
      Serial.print(myAngles[0]); Serial.print(", ");
      Serial.print(myAngles[1]); Serial.print(", ");
      Serial.print(myAngles[2]); Serial.print(", ");
      Serial.print(IKvalid); Serial.println();
    }
    ParameterSave();
      
    for (int i = 0; i < 3; i++)  Mdata.outregs[i] = Mdata.inregs[i];

    for (int i = 0; i < 3; i++) Mdata.outcoils[i] = Mdata.incoils[i];
    
    mc.poll();
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void InitThread() {
  xTaskCreatePinnedToCore(
    ModbusTask,   
    "Task_Modbus", 
    10000,          
    NULL,          
    1,              
    &TaskModbus,    
    0             
  );
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mc.begin();
  ParameterLoad();
  InitThread();
}

void loop() {

}

