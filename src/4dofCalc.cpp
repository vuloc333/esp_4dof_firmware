#include "Calc4dof.h"

Calc4dof::Calc4dof(float l1, float l2, float l3, float baseH) : _baseH(baseH) {
    _lengths[0] = l1; 
    _lengths[1] = l2; 
    _lengths[2] = l3;
    // Khởi tạo FABRIK sau khi đã có chiều dài xương
    _fabrik = new NocKinematics::FABRIK(_lengths, NumberJoints); 
    _fabrik->setTolerance(0.1f);
    _fabrik->setMaxIterations(20);
    _fabrik->setBasePosition(NocCore::Vector3(0, 0, 0));
}

int Calc4dof::computeIK(float tx, float ty, float tz) {
    long startTime = micros();
    //Reset pos
    NocCore::Vector3 resetPos(0, 0, tz + tx + ty); // Đặt lại vị trí ban đầu để tránh lỗi khi target quá xa
    _fabrik->solve(resetPos);
    //Calc Pos
    NocCore::Vector3 localTarget(tx, ty, tz - _baseH);
    bool solved = _fabrik->solve(localTarget);
    //Get Angle
    if (solved) {
        NocCore::Vector3 p[4];
        for (int i = 0; i < 4; i++) p[i] = _fabrik->getJointPosition(i);

        // Tính toán và lưu thẳng vào mảng nội bộ
        _currentAngles[0] = atan2(p[1].y_val, p[1].x_val) * 180.0 / PI; // Base

        float r[4], z[4];
        for (int i = 0; i < 4; i++) {
            r[i] = sqrt(p[i].x_val * p[i].x_val + p[i].y_val * p[i].y_val);
            z[i] = p[i].z_val;
        }

        _currentAngles[1] = atan2(z[1] - z[0], r[1] - r[0]) * 180.0 / PI; // Shoulder
        float aP1P2 = atan2(z[2] - z[1], r[2] - r[1]) * 180.0 / PI;
        _currentAngles[2] = aP1P2 - _currentAngles[1];                  // Elbow
        float aP2P3 = atan2(z[3] - z[2], r[3] - r[2]) * 180.0 / PI;
        _currentAngles[3] = aP2P3 - aP1P2;                             // Wrist
    }
    return micros() - startTime; // Trả về thời gian tính toán IK
}

void Calc4dof::getAnglesArray(float outArray[4]) {
    for(int i = 0; i < 4; i++) outArray[i] = _currentAngles[i];
}

void Calc4dof::getFKArray(float outArray[3]) {
    float r1 = _currentAngles[0] * PI / 180.0;
    float r2 = _currentAngles[1] * PI / 180.0;
    float r3 = _currentAngles[2] * PI / 180.0;
    float r4 = _currentAngles[3] * PI / 180.0;

    float a2 = r2, a23 = r2 + r3, a234 = r2 + r3 + r4;
    float R = _lengths[0] * cos(a2) + _lengths[1] * cos(a23) + _lengths[2] * cos(a234);
    
    outArray[0] = R * cos(r1); // X
    outArray[1] = R * sin(r1); // Y
    outArray[2] = _baseH + _lengths[0] * sin(a2) + _lengths[1] * sin(a23) + _lengths[2] * sin(a234); // Z
}