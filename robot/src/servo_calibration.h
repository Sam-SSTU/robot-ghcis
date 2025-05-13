#ifndef SERVO_CALIBRATION_H
#define SERVO_CALIBRATION_H

#include <Arduino.h>
#include <Servo.h>

class ServoCalibrator {
private:
    Servo* servos[3];
    const int servoCount = 3;
    
    // 当前校准的舵机索引
    int currentServo = 0;
    
    // 存储原点位置
    int originPositions[3] = {90, 90, 90};
    
    // 存储限位
    struct ServoLimits {
        int min;
        int max;
    } servoLimits[3];

public:
    ServoCalibrator(Servo* servo1, Servo* servo2, Servo* servo3) {
        servos[0] = servo1;
        servos[1] = servo2;
        servos[2] = servo3;
        
        // 初始化限位
        for(int i = 0; i < 3; i++) {
            servoLimits[i].min = 0;
            servoLimits[i].max = 180;
        }
    }

    // 设置当前舵机的原点
    void setOrigin() {
        originPositions[currentServo] = servos[currentServo]->read();
    }

    // 设置当前舵机的最小限位
    void setMinLimit() {
        servoLimits[currentServo].min = servos[currentServo]->read();
    }

    // 设置当前舵机的最大限位
    void setMaxLimit() {
        servoLimits[currentServo].max = servos[currentServo]->read();
    }

    // 切换到下一个舵机
    bool nextServo() {
        currentServo++;
        if(currentServo >= servoCount) {
            currentServo = 0;
            return false;  // 所有舵机都已校准完成
        }
        return true;  // 还有舵机需要校准
    }

    // 获取当前舵机位置
    int getCurrentPosition() {
        return servos[currentServo]->read();
    }

    // 移动当前舵机
    void moveCurrentServo(int step) {
        int currentPos = servos[currentServo]->read();
        int newPos = currentPos + step;
        
        // 确保在0-180度范围内
        newPos = constrain(newPos, 0, 180);
        
        servos[currentServo]->write(newPos);
    }

    // 获取当前校准的舵机索引
    int getCurrentServoIndex() {
        return currentServo;
    }

    // 打印校准数据
    void printCalibrationData() {
        Serial.println("\n=== 校准数据 ===");
        for(int i = 0; i < 3; i++) {
            Serial.print("舵机 ");
            Serial.print(i + 1);
            Serial.println(" 的校准数据:");
            Serial.print("  原点位置: ");
            Serial.println(originPositions[i]);
            Serial.print("  最小限位: ");
            Serial.println(servoLimits[i].min);
            Serial.print("  最大限位: ");
            Serial.println(servoLimits[i].max);
            Serial.println("-------------------");
        }
    }
};

#endif // SERVO_CALIBRATION_H 