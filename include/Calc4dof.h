#ifndef _CALC_4DOF_H
#define _CALC_4DOF_H

#include "Arduino.h"
#include "NocKinematics.h"
#include <math.h>

class Calc4dof {
public:
    // Khởi tạo
    Calc4dof(float l1, float l2, float l3, float baseH);

    // Tính toán IK
    bool computeIK(float tx, float ty, float tz);

    // Xuất góc ra mảng 1 chiều (4 phần tử: Base, Shoulder, Elbow, Wrist)
    void getAnglesArray(float outArray[4]);

    // Xuất tọa độ FK ra mảng 1 chiều (3 phần tử: X, Y, Z)
    void getFKArray(float outArray[3]);

private:
    float _lengths[3];
    float _baseH;
    NocKinematics::FABRIK _fabrik;
    float _currentAngles[4]; // Lưu trữ nội bộ dạng mảng
};

#endif