#define RateJoint1 1
#define RateJoint2 1
#define RateJoint3 1
#define RateJoint4 1

#include "Modcom.h"
#include "Calc4dof.h"
#include "Storage.h"
#include "JointController.h"

//Modbus
ModbusReg Mdata;
Modcom mc(Mdata, 0);
//Storage
int16_t Parameter[20];
StorageManager storage;
//Threading core0
TaskHandle_t TaskModbus;
//Stepper control robot 4dof
FastAccelStepperEngine engine = FastAccelStepperEngine();
//Angle to step conversion: Steps per Degree
JointController J1(RateJoint1), J2(RateJoint2), J3(RateJoint3), J4(RateJoint4);

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

void ModbusTask(void* parameter) {
  Calc4dof myArm(100.0, 100.0, 30.0, 50.0);
  while (1) {
    long startTime = micros();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // 10ms

    float tx = Mdata.inregs[10] / 100.0;
    float ty = Mdata.inregs[11] / 100.0;
    float tz = Mdata.inregs[12] / 100.0;
    // 2. Tính toán IK
    //if (myArm.computeIK(tx, ty, tz)) {
    Mdata.outregs[25] = myArm.computeIK(tx, ty, tz);

    myArm.getAnglesArray(myAngles);
    myArm.getFKArray(myPos);
    // Ví dụ: Đổ thẳng vào thanh ghi Modbus
    for(int i = 0; i < 4; i++) {
      Mdata.outregs[20 + i] = (int16_t)(myAngles[i] * 100);
      if (i < 3) Mdata.outregs[30 + i] = (int16_t)(myPos[i] * 100);
            }
    ParameterSave();
      
    for (int i = 0; i < 3; i++)  Mdata.outregs[i] = Mdata.inregs[i];

    for (int i = 0; i < 3; i++) Mdata.outcoils[i] = Mdata.incoils[i];
    
    mc.poll();
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    Mdata.outregs[26] = micros() - startTime;
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
  engine.init();
  J1.begin(engine, 32, 33);
  J2.begin(engine, 25, 26);
  J3.begin(engine, 27, 14);
  J4.begin(engine, 13, 19);
  
  // Set tốc độ/gia tốc cho từng khớp nếu muốn khác nhau
  J1.setProfile(500, 100);
  J2.setProfile(500, 100);
  J3.setProfile(500, 100);
  J4.setProfile(500, 100);
}

void loop() {
  long startTime = micros();
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // 10ms
  // 1. Cấu hình tốc độ/gia tốc cho từng khớp (có thể thay đổi động)
  
  // 2. Nạp góc vào là xong, đéo cần quan tâm step hay tính toán gì nữa
  J1.setTargetAngle(Mdata.outregs[20] / 100.0);
  J2.setTargetAngle(Mdata.outregs[21] / 100.0);
  J3.setTargetAngle(Mdata.outregs[22] / 100.0);
  J4.setTargetAngle(Mdata.outregs[23] / 100.0);
  //Current Angle Feedback
  Mdata.outregs[35] = (int16_t)(J1.getCurrentAngle() * 100);
  Mdata.outregs[36] = (int16_t)(J2.getCurrentAngle() * 100);
  Mdata.outregs[37] = (int16_t)(J3.getCurrentAngle() * 100);
  Mdata.outregs[38] = (int16_t)(J4.getCurrentAngle() * 100);

  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  Mdata.outregs[27] = micros() - startTime;
}

