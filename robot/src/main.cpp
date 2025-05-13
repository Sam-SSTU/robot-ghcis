#include <Arduino.h>
#include <Servo.h>
#include "servo_calibration.h"

/* -----------------------------------------------------------
 *  舵机校准程序 - 使用内部上拉电阻和非阻塞去抖动
 *  引脚: D3 = 确认, D11 = 上, D12 = 下
 *  接线: 按钮一端接引脚，另一端接GND（无需额外电阻）
 *  逻辑: 按下 = LOW, 释放 = HIGH
 * -----------------------------------------------------------
 */

// 定义舵机引脚
const int SERVO1_PIN = 10;
const int SERVO2_PIN = 9;
const int SERVO3_PIN = 8;

// 定义按钮引脚
const byte CONFIRM_PIN = 3;    // 确认按钮
const byte UP_PIN = 11;        // 向上按钮
const byte DOWN_PIN = 12;      // 向下按钮

const unsigned long DEBOUNCE_MS = 5;   // 按钮抖动消除阈值
const int SERVO_STEP = 10;             // 每次调整的角度

// 按钮结构体
struct Button {
    const char* name;
    byte pin;
    bool stableState;      // 最新确认的稳定电平
    bool lastReading;      // 上一次立即读取的电平
    unsigned long lastChangeTime;  // 发现电平改变的时间戳
    bool wasPressed;       // 之前是否处于按下状态
};

Button buttons[] = {
    { "CONFIRM", CONFIRM_PIN, HIGH, HIGH, 0, false },
    { "UP",      UP_PIN,      HIGH, HIGH, 0, false },
    { "DOWN",    DOWN_PIN,    HIGH, HIGH, 0, false }
};
const byte BTN_COUNT = sizeof(buttons) / sizeof(buttons[0]);

// 创建舵机对象
Servo servo1;
Servo servo2;
Servo servo3;

// 创建校准器
ServoCalibrator* calibrator;

// 校准状态
enum CalibrationState {
    WAITING_START,  // 等待开始校准
    ORIGIN,        // 设置原点
    MIN_LIMIT,     // 设置最小限位
    MAX_LIMIT      // 设置最大限位
};

CalibrationState currentState = WAITING_START;

// 函数声明
void handleConfirmButton();

void setup() {
    Serial.begin(9600);
    Serial.println(F("== 舵机校准程序启动 =="));

    // 初始化按钮引脚为上拉输入模式
    pinMode(CONFIRM_PIN, INPUT_PULLUP);
    pinMode(UP_PIN, INPUT_PULLUP);
    pinMode(DOWN_PIN, INPUT_PULLUP);

    // 初始化舵机
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);

    // 创建校准器
    calibrator = new ServoCalibrator(&servo1, &servo2, &servo3);

    // 所有舵机先回到0度
    Serial.println("正在将所有舵机归零...");
    servo1.write(0);
    delay(1000);
    servo2.write(0);
    delay(1000);
    servo3.write(0);
    delay(1000);
    
    Serial.println("所有舵机已归零");
    Serial.println("请固定好舵盘，然后按确认按钮开始校准");
}

void loop() {
    unsigned long now = millis();

    // 处理所有按钮
    for (byte i = 0; i < BTN_COUNT; ++i) {
        bool reading = digitalRead(buttons[i].pin);

        // 若瞬时读取改变 → 记录时间戳
        if (reading != buttons[i].lastReading) {
            buttons[i].lastReading = reading;
            buttons[i].lastChangeTime = now;
        }

        // 若电平保持 DEBOUNCE_MS 毫秒 → 认定为稳定
        if ((now - buttons[i].lastChangeTime) >= DEBOUNCE_MS &&
            reading != buttons[i].stableState) {

            buttons[i].stableState = reading;
            bool isPressed = (reading == LOW);

            // 检测按钮释放
            if (!isPressed && buttons[i].wasPressed) {
                // 按钮释放时的动作
                if (i == 1) {  // UP按钮
                    calibrator->moveCurrentServo(SERVO_STEP);
                    Serial.print(F("上调 "));
                    Serial.print(SERVO_STEP);
                    Serial.println(F(" 度"));
                }
                else if (i == 2) {  // DOWN按钮
                    calibrator->moveCurrentServo(-SERVO_STEP);
                    Serial.print(F("下调 "));
                    Serial.print(SERVO_STEP);
                    Serial.println(F(" 度"));
                }
                else if (i == 0) {  // CONFIRM按钮
                    handleConfirmButton();
                }
            }
            buttons[i].wasPressed = isPressed;
        }
    }

    // 定期打印状态
    static unsigned long lastStatusTime = 0;
    if (now - lastStatusTime > 200) {  // 每200ms打印一次
        lastStatusTime = now;
        Serial.print("舵机:");
        Serial.print(calibrator->getCurrentServoIndex() + 1);
        Serial.print(" 位置:");
        Serial.print(calibrator->getCurrentPosition());
        Serial.print(" 状态:");
        switch(currentState) {
            case WAITING_START: Serial.println("等待开始"); break;
            case ORIGIN: Serial.println("设置原点"); break;
            case MIN_LIMIT: Serial.println("设置最小限位"); break;
            case MAX_LIMIT: Serial.println("设置最大限位"); break;
        }
    }
}

// 处理确认按钮
void handleConfirmButton() {
    switch (currentState) {
        case WAITING_START:
            currentState = ORIGIN;
            Serial.println("\n校准说明：");
            Serial.println("1. 使用上/下按钮调整舵机位置（每次10度）");
            Serial.println("2. 按确认按钮保存当前位置");
            Serial.println("3. 每个舵机需要设置：原点、最小限位、最大限位");
            Serial.println("\n开始校准舵机 1 的原点位置");
            break;

        case ORIGIN:
            calibrator->setOrigin();
            currentState = MIN_LIMIT;
            Serial.println("原点已设置");
            Serial.println("请设置最小限位");
            break;

        case MIN_LIMIT:
            calibrator->setMinLimit();
            currentState = MAX_LIMIT;
            Serial.println("最小限位已设置");
            Serial.println("请设置最大限位");
            break;

        case MAX_LIMIT:
            calibrator->setMaxLimit();
            currentState = ORIGIN;
            Serial.println("最大限位已设置");

            // 检查是否还有下一个舵机需要校准
            if (!calibrator->nextServo()) {
                // 所有舵机都已校准完成
                Serial.println("\n所有舵机校准完成！");
                calibrator->printCalibrationData();
                while(1); // 校准完成，停止程序
            } else {
                // 开始下一个舵机的校准
                int nextServo = calibrator->getCurrentServoIndex() + 1;
                Serial.print("\n开始校准舵机 ");
                Serial.print(nextServo);
                Serial.println(" 的原点位置");
            }
            break;
    }
}