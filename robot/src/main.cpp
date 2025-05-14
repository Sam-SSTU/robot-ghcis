#include <Arduino.h>
#include <Servo.h>

/* -----------------------------------------------------------
 *  14方向舵机控制程序
 *  - 每个方向对应特定的舵机角度组合
 *  - 按下按钮4切换方向
 * -----------------------------------------------------------
 */

// 定义舵机引脚
const int SERVO1_PIN = 10;
const int SERVO2_PIN = 9;
const int SERVO3_PIN = 8;

// 定义按钮引脚
const byte CONFIRM_PIN = 4;    // 确认按钮

// 创建舵机对象
Servo servo1;
Servo servo2;
Servo servo3;

// 定义方向结构体
struct ServoPosition {
    const char* name;  // 方向名称
    int servo1;        // 舵机1角度
    int servo2;        // 舵机2角度
    int servo3;        // 舵机3角度
};

// 定义所有方向的舵机角度
const ServoPosition POSITIONS[] = {
    {"上    ", 180, 0, 60},     // 上
    {"下    ", 0, 180, 60},     // 下
    {"左    ", 20, 10, 180},      // 左
    {"右    ", 180, 180, 0},    // 右
    {"左上  ", 180, 0, 180},    // 左上
    {"右上  ", 180, 0, 0},      // 右上
    {"左下  ", 0, 180, 180},    // 左下
    {"右下  ", 0, 180, 0}       // 右下
};

const int POSITION_COUNT = sizeof(POSITIONS) / sizeof(POSITIONS[0]);
int currentPosition = 0;  // 当前位置索引

// 移动所有舵机到指定角度
void moveToPosition(const ServoPosition& pos) {
    Serial.print("移动到方向: ");
    Serial.println(pos.name);
    Serial.print("舵机角度: ");
    Serial.print(pos.servo1);
    Serial.print(", ");
    Serial.print(pos.servo2);
    Serial.print(", ");
    Serial.println(pos.servo3);
    
    servo1.write(pos.servo1);
    delay(300);  // 等待舵机1到位
    servo2.write(pos.servo2);
    delay(300);  // 等待舵机2到位
    servo3.write(pos.servo3);
    delay(300);  // 等待舵机3到位
    
    Serial.println("移动完成");
}

void setup() {
    Serial.begin(9600);
    Serial.println("14方向舵机控制程序启动");

    // 初始化按钮引脚为上拉输入
    pinMode(CONFIRM_PIN, INPUT_PULLUP);

    // 初始化舵机
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);

    // 初始化时移动到第一个位置
    moveToPosition(POSITIONS[0]);
}

void loop() {
    // 读取按钮状态（上拉模式，按下为LOW）
    if (digitalRead(CONFIRM_PIN) == LOW) {
        delay(50);  // 简单消抖
        // 确认按钮是否真的被按下
        if (digitalRead(CONFIRM_PIN) == LOW) {
            // 切换到下一个位置
            currentPosition = (currentPosition + 1) % POSITION_COUNT;
            moveToPosition(POSITIONS[currentPosition]);
            
            // 等待按钮释放
            while(digitalRead(CONFIRM_PIN) == LOW) {
                delay(10);
            }
            delay(50);  // 释放后的额外消抖
        }
    }
}