#include <Arduino.h>
#include <Servo.h>

/* -----------------------------------------------------------
 *  遥感控制舵机程序
 *  - 遥感控制：X轴-A4，Y轴-A5
 *  - 方向映射：
 *    将360度分成36个方向(每10度一个)
 *    基于8个基准方向计算所有其他方向的舵机角度
 *  - 遥感回中时，舵机自动回到中间位置
 * -----------------------------------------------------------
 */

// 定义舵机引脚
const int SERVO1_PIN = 10;
const int SERVO2_PIN = 9;
const int SERVO3_PIN = 8;

// 定义遥感引脚
const byte JOYSTICK_X_PIN = A4;  // 遥感X轴
const byte JOYSTICK_Y_PIN = A5;  // 遥感Y轴

// 遥感参数
const float DEADZONE = 0.2;      // 死区大小(0-1)
const int ANGLE_SEGMENTS = 36;   // 将360度分成36份，每份10度
const int SERVO_UPDATE_RATE = 50; // 舵机更新间隔(ms)

// 遥感读数变量
int joystickX;
int joystickY;

// 创建舵机对象
Servo servo1;
Servo servo2;
Servo servo3;

// 定义舵机中立位置
const int SERVO1_CENTER = 180;
const int SERVO2_CENTER = 180;
const int SERVO3_CENTER = 180;

// 定义基准方向的舵机角度
struct ServoAngles {
    int servo1;
    int servo2;
    int servo3;
};

// 8个基准方向的舵机角度 (以45度为间隔)
const ServoAngles BASE_DIRECTIONS[] = {
    {180, 0, 60},    // 上 (0度)
    {180, 0, 0},     // 右上 (45度)
    {180, 180, 0},   // 右 (90度)
    {0, 180, 0},     // 右下 (135度)
    {0, 180, 60},    // 下 (180度)
    {0, 180, 180},   // 左下 (225度)
    {20, 10, 180},   // 左 (270度)
    {180, 0, 180}    // 左上 (315度)
};

// 计算两个基准方向之间的角度的舵机位置
ServoAngles interpolateDirection(float angle) {
    // 找出最近的两个基准方向
    int baseSector = (int)(angle / 45.0);
    int nextSector = (baseSector + 1) % 8;
    
    // 计算插值比例
    float blend = (angle - (baseSector * 45.0)) / 45.0;
    
    // 创建插值结果
    ServoAngles result;
    
    // 插值计算三个舵机的角度
    result.servo1 = BASE_DIRECTIONS[baseSector].servo1 * (1 - blend) + 
                    BASE_DIRECTIONS[nextSector].servo1 * blend;
    result.servo2 = BASE_DIRECTIONS[baseSector].servo2 * (1 - blend) + 
                    BASE_DIRECTIONS[nextSector].servo2 * blend;
    result.servo3 = BASE_DIRECTIONS[baseSector].servo3 * (1 - blend) + 
                    BASE_DIRECTIONS[nextSector].servo3 * blend;
    
    return result;
}

// 将舵机移动到指定角度
void moveServos(int servo1Angle, int servo2Angle, int servo3Angle) {
    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);
    
    // 调试输出
    Serial.print("舵机角度: ");
    Serial.print(servo1Angle);
    Serial.print(", ");
    Serial.print(servo2Angle);
    Serial.print(", ");
    Serial.println(servo3Angle);
}

// 将舵机移动到中间位置
void moveToCenterPosition() {
    moveServos(SERVO1_CENTER, SERVO2_CENTER, SERVO3_CENTER);
    Serial.println("舵机回到中间位置");
}

// 将遥感值映射到角度并控制舵机
void mapJoystickToServos() {
    // 读取遥感值
    joystickX = analogRead(JOYSTICK_X_PIN);
    joystickY = analogRead(JOYSTICK_Y_PIN);

    // 将遥感值转换为-100到100的范围
    float x = map(joystickX, 0, 1023, -100, 100);
    float y = map(joystickY, 0, 1023, -100, 100);

    // 计算遥感的角度和强度
    float angle = atan2(y, x) * 180 / PI;
    if (angle < 0) angle += 360;  // 转换到0-360度范围
    float strength = sqrt(x*x + y*y) / 100.0;
    
    // 调试输出遥感信息
    Serial.print("遥感: X=");
    Serial.print(x);
    Serial.print(", Y=");
    Serial.print(y);
    Serial.print(", 强度=");
    Serial.print(strength);
    Serial.print(", 角度=");
    Serial.println(angle);
    
    // 如果遥感在死区内，移动到中间位置
    if (strength < DEADZONE) {
        moveToCenterPosition();
        return;
    }
    
    // 计算当前角度所在的10度分段
    int segment = (int)(angle / 10.0); // 0-35的分段
    
    // 使用插值计算对应的舵机角度
    ServoAngles servoPos = interpolateDirection(angle);

    // 移动舵机
    moveServos(servoPos.servo1, servoPos.servo2, servoPos.servo3);
    
    // 额外的调试信息
    Serial.print("分段: ");
    Serial.println(segment);
}

void setup() {
    Serial.begin(9600);
    Serial.println("遥感36方向舵机控制程序启动");
    
    // 初始化遥感引脚为输入
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);

    // 初始化舵机
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    
    // 初始化时将舵机移动到中间位置
    moveToCenterPosition();
    
    // 输出基准方向信息
    Serial.println("基准方向舵机角度:");
    for (int i = 0; i < 8; i++) {
        Serial.print(i * 45);
        Serial.print("度: (");
        Serial.print(BASE_DIRECTIONS[i].servo1);
        Serial.print(", ");
        Serial.print(BASE_DIRECTIONS[i].servo2);
        Serial.print(", ");
        Serial.print(BASE_DIRECTIONS[i].servo3);
        Serial.println(")");
    }
}

void loop() {
    mapJoystickToServos();
    delay(SERVO_UPDATE_RATE);
}