#include "JointController.h"

JointController::JointController(float stepsPerDeg) : _stepsPerDeg(stepsPerDeg) {}

void JointController::begin(FastAccelStepperEngine &engine, uint8_t stepPin, uint8_t dirPin) {
    _stepper = engine.stepperConnectToPin(stepPin);
    if (_stepper) {
        _stepper->setDirectionPin(dirPin);
        _stepper->setAutoEnable(true); // Tự động quản lý chân Enable nếu có
    }
    else {
        Serial.print("Error: Could not connect stepper to pin ");
        Serial.println(stepPin);
    }
}

void JointController::setTargetAngle(float angle) {
    if (_stepper) {
        // Công thức đổi từ độ sang xung chuyên nghiệp
        long targetSteps = (long)(angle * _stepsPerDeg);
        _stepper->moveTo(targetSteps);
    } else {
        Serial.println("Error: Stepper not initialized");
    }
}

void JointController::setProfile(uint32_t speed, uint32_t accel) {
    if (_stepper) {
        _stepper->setSpeedInHz(speed);
        _stepper->setAcceleration(accel);
    } else {
        Serial.println("Error: Stepper not initialized");
    }
}

bool JointController::isMoving() {
    return (_stepper && _stepper->isRunning());
}

float JointController::getCurrentAngle() {
    if (_stepper != nullptr) {
        long currentSteps = _stepper->getCurrentPosition(); 
        return (float)currentSteps / _stepsPerDeg;
    }
    return -999.0f;
}