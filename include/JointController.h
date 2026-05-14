#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <Arduino.h>
#include "FastAccelStepper.h"

class JointController {
public:
    // Khởi tạo chỉ cần tỉ số truyền (Steps per Degree)
    JointController(float stepsPerDeg);
    
    // Mày sẽ gán chân pin ở đây, tại main.cpp
    void begin(FastAccelStepperEngine &engine, uint8_t stepPin, uint8_t dirPin);
    
    // Nạp góc trực tiếp
    void setTargetAngle(float angle);
    
    // Cấu hình mượt mà
    void setProfile(uint32_t speed, uint32_t accel);
    
    bool isMoving();

    float getCurrentAngle();
private:
    FastAccelStepper* _stepper = nullptr;
    float _stepsPerDeg;
};

#endif