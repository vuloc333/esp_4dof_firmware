#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// --- THÔNG SỐ CƠ KHÍ (Sửa theo thực tế để hết sai lệch góc) ---
// Công thức: (Số xung mỗi vòng của động cơ * Vi bước của Driver * Tỉ số truyền hộp số) / 360
// Ví dụ: Động cơ 1.8 độ (200 xung), Vi bước 1/16, không hộp số => (200 * 16) / 360 = 8.8889
#define RATE_JOINT1 8.8889f
#define RATE_JOINT2 8.8889f
#define RATE_JOINT3 8.8889f
#define RATE_JOINT4 8.8889f

// --- CẤU HÌNH PIN (GPIO) ---
#define J1_STEP_PIN 32
#define J1_DIR_PIN  33
#define J2_STEP_PIN 25
#define J2_DIR_PIN  26
#define J3_STEP_PIN 27
#define J3_DIR_PIN  14
#define J4_STEP_PIN 13
#define J4_DIR_PIN  19

// --- THÔNG SỐ VẬN HÀNH ---
#define MODBUS_PORT     2    // Nên dùng Serial2 (Pin 16, 17) để tránh Reset khi cắm USB
#define SLAVE_ID        1
#define LOOP_MS         10   // Chu kỳ vòng lặp
#define DEFAULT_SPEED   4000
#define DEFAULT_ACCEL   1000

// --- THÔNG SỐ ROBOT (Dùng để tính IK/FK) ---
#define L1 100.0f // Chiều dài xương 1 (mm)
#define L2 100.0f // Chiều dài xương 2 (mm)
#define L3 30.0f // Chiều dài xương 3 (mm)
#define BASE_HEIGHT 50.0f // Chiều cao từ mặt đất đến gốc khớp 1 (mm)

//Modbus Register Map
#define REG_TARGET_ANGLE_BASE 20 // 20-23: Target angles (Base, Shoulder, Elbow, Wrist) * 100
#define REG_CURRENT_ANGLE_BASE 35 // 35-38: Current angles feedback (Base

#endif