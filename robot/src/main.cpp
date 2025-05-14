#include <Arduino.h>
#include <Servo.h>

/* -----------------------------------------------------------
 *  遥感和按钮控制舵机程序
 *  - 遥感控制：X轴-A4，Y轴-A5 (比例速率控制)
 *  - 按钮控制：
 *    上按钮(D12) - 所有舵机角度增加 (长按连续)
 *    下按钮(D11) - 所有舵机角度减少 (长按连续)
 *    复位按钮(D2) - 所有舵机到最小角度 (单次)
 *    回中按钮(A3) - 所有舵机到预设中心点 (单次)
 *  - 使用结构体和时间戳进行按钮消抖
 *  - 遥感无自动回中，控制舵机移动速率
 * -----------------------------------------------------------
 */

// 定义舵机引脚
const int SERVO1_PIN = 10;
const int SERVO2_PIN = 9;
const int SERVO3_PIN = 8;

// 定义遥感引脚
const byte JOYSTICK_X_PIN = A4;  // 遥感X轴
const byte JOYSTICK_Y_PIN = A5;  // 遥感Y轴

// 定义按钮引脚
const byte UP_PIN = 12;        // 向上按钮 (物理上的"上"按钮，增加角度)
const byte DOWN_PIN = 11;      // 向下按钮 (物理上的"下"按钮，减少角度)
const byte RESET_PIN = 2;      // 复位按钮 (舵机到最小角度)
const byte CENTER_JOY_PIN = A3; // 手动回中按钮 (舵机到预设中心)

// 控制参数
const float DEADZONE = 0.2;      // 遥感死区大小(0-1)
const int SERVO_UPDATE_RATE = 30; // 主循环延迟(ms)
const unsigned long DEBOUNCE_MS = 5; // 按钮消抖时间
const int ANGLE_STEP = 5;       // 按钮控制的舵机角度步长 (减小以便长按时更平滑)
const float JOYSTICK_SENSITIVITY = 0.05f; // 遥感速率控制灵敏度

// 每个舵机的角度范围限制
const int MIN_ANGLE_1 = 0;
const int MAX_ANGLE_1 = 180;
const int MIN_ANGLE_2 = 0;
const int MAX_ANGLE_2 = 180;
const int MIN_ANGLE_3 = 0;
const int MAX_ANGLE_3 = 180;

// 遥感读数变量
int joystickX;
int joystickY;

// 创建舵机对象
Servo servo1;
Servo servo2;
Servo servo3;

// 当前舵机角度 (使用浮点数以实现平滑速率控制)
float currentServo1Pos = 180.0f; 
float currentServo2Pos = 180.0f;
float currentServo3Pos = 180.0f;

// 定义舵机中立位置 (手动回中按钮A3使用)
const int SERVO1_CENTER = 180;
const int SERVO2_CENTER = 180;
const int SERVO3_CENTER = 180;

// Button struct for debouncing
struct Button {
  const char*   name;
  byte          pin;
  bool          stableState;    // Debounced state
  bool          lastReading;    // Previous raw reading
  unsigned long lastChangeTime; // Time of last unstable reading
  bool          actionTakenOnPress; // Flag to ensure single action for non-continuous buttons
};

Button buttons[] = {
  { "UP",       UP_PIN,         HIGH, HIGH, 0, false },
  { "DOWN",     DOWN_PIN,       HIGH, HIGH, 0, false },
  { "RESET",    RESET_PIN,      HIGH, HIGH, 0, false },
  { "CENTER",   CENTER_JOY_PIN, HIGH, HIGH, 0, false }
};

// 定义基准方向的舵机角度 (遥感用 - 代表全速偏转时的目标方向)
struct ServoAngles {
    int servo1;
    int servo2;
    int servo3;
};

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

// 函数声明
void moveServos(float s1, float s2, float s3); // Now accepts floats
void moveToCenterPosition();
void servoAnglesIncrease(); // Renamed for clarity, UP button increases angles
void servoAnglesDecrease(); // Renamed for clarity, DOWN button decreases angles
void resetToMinPosition();
ServoAngles interpolateDirection(float angle);
void mapJoystickToServos();
void handleButtons();

ServoAngles interpolateDirection(float angle) {
    int baseSector = (int)(angle / 45.0f);
    if (baseSector < 0) baseSector = 0; 
    if (baseSector > 7) baseSector = 7;
    int nextSector = (baseSector + 1) % 8;
    float blend = (angle - (baseSector * 45.0f)) / 45.0f;
    if (blend < 0.0f) blend = 0.0f; 
    if (blend > 1.0f) blend = 1.0f;

    ServoAngles result;
    result.servo1 = round(BASE_DIRECTIONS[baseSector].servo1 * (1.0f - blend) + BASE_DIRECTIONS[nextSector].servo1 * blend);
    result.servo2 = round(BASE_DIRECTIONS[baseSector].servo2 * (1.0f - blend) + BASE_DIRECTIONS[nextSector].servo2 * blend);
    result.servo3 = round(BASE_DIRECTIONS[baseSector].servo3 * (1.0f - blend) + BASE_DIRECTIONS[nextSector].servo3 * blend);
    return result;
}

void moveServos(float s1, float s2, float s3) { // Accepts floats
    currentServo1Pos = constrain(s1, (float)MIN_ANGLE_1, (float)MAX_ANGLE_1);
    currentServo2Pos = constrain(s2, (float)MIN_ANGLE_2, (float)MAX_ANGLE_2);
    currentServo3Pos = constrain(s3, (float)MIN_ANGLE_3, (float)MAX_ANGLE_3);
    
    servo1.write(round(currentServo1Pos)); // Write rounded int to servo
    servo2.write(round(currentServo2Pos));
    servo3.write(round(currentServo3Pos));
    
    // Reduced frequency debug output
    static unsigned long lastServoDebugTime = 0;
    if (millis() - lastServoDebugTime > 200) { // Output every 200ms
        Serial.print("舵机目标(float): ");
        Serial.print(currentServo1Pos, 2); Serial.print(", ");
        Serial.print(currentServo2Pos, 2); Serial.print(", ");
        Serial.print(currentServo3Pos, 2); Serial.print(" -> Int: ");
        Serial.print(round(currentServo1Pos)); Serial.print(", ");
        Serial.print(round(currentServo2Pos)); Serial.print(", ");
        Serial.println(round(currentServo3Pos));
        lastServoDebugTime = millis();
    }
}

void moveToCenterPosition() {
    moveServos((float)SERVO1_CENTER, (float)SERVO2_CENTER, (float)SERVO3_CENTER);
    // Serial.println("按钮: 舵机回到预设中心位置."); // Reduce verbosity
}

// UP button (e.g., D12) makes angles INCREASE
void servoAnglesIncrease() {
    moveServos(currentServo1Pos + ANGLE_STEP, 
               currentServo2Pos + ANGLE_STEP, 
               currentServo3Pos + ANGLE_STEP);
    // Serial.println("按钮: 角度增加中..."); // Reduce verbosity for continuous hold
}

// DOWN button (e.g., D11) makes angles DECREASE
void servoAnglesDecrease() {
    moveServos(currentServo1Pos - ANGLE_STEP, 
               currentServo2Pos - ANGLE_STEP, 
               currentServo3Pos - ANGLE_STEP);
    // Serial.println("按钮: 角度减少中..."); // Reduce verbosity for continuous hold
}

void resetToMinPosition() { 
    moveServos((float)MIN_ANGLE_1, (float)MIN_ANGLE_2, (float)MIN_ANGLE_3);
    Serial.println("按钮: 已重置到最小角度.");
}

void mapJoystickToServos() {
    joystickX = analogRead(JOYSTICK_X_PIN);
    joystickY = analogRead(JOYSTICK_Y_PIN);
    float x_mapped = map(joystickX, 0, 1023, -100, 100);
    float y_mapped = map(joystickY, 0, 1023, -100, 100);
    float angle_rad = atan2(y_mapped, x_mapped);
    float angle_deg = angle_rad * 180.0f / PI;
    if (angle_deg < 0) angle_deg += 360.0f;
    float strength = sqrt(x_mapped*x_mapped + y_mapped*y_mapped); // Raw strength 0-~141

    if (strength / 100.0f < DEADZONE) { // Normalize strength for deadzone check
        // Joystick is in deadzone, servos hold current position.
        return; 
    }
    
    // Normalize strength: 0 at deadzone edge, 1 at full deflection (approx 100 for map output)
    float normalized_strength = (strength - (DEADZONE * 100.0f)) / (100.0f - (DEADZONE * 100.0f));
    normalized_strength = constrain(normalized_strength, 0.0f, 1.0f);

    ServoAngles targetDirectionPos = interpolateDirection(angle_deg);

    float delta1 = (targetDirectionPos.servo1 - currentServo1Pos) * normalized_strength * JOYSTICK_SENSITIVITY;
    float delta2 = (targetDirectionPos.servo2 - currentServo2Pos) * normalized_strength * JOYSTICK_SENSITIVITY;
    float delta3 = (targetDirectionPos.servo3 - currentServo3Pos) * normalized_strength * JOYSTICK_SENSITIVITY;

    moveServos(currentServo1Pos + delta1, 
               currentServo2Pos + delta2, 
               currentServo3Pos + delta3);

    // Optional: Reduced frequency joystick debug output
    static unsigned long lastJoyDebug = 0;
    if (millis() - lastJoyDebug > 250) { 
       // Serial.print("遥感: Str="); Serial.print(strength,1);
       // Serial.print(" NormStr="); Serial.print(normalized_strength,2);
       // Serial.print(" Ang="); Serial.print(angle_deg,1);
       // Serial.print(" TargetDir(S1)="); Serial.print(targetDirectionPos.servo1);
       // Serial.print(" Delta1="); Serial.println(delta1, 2);
       lastJoyDebug = millis();
    }
}

void handleButtons() {
  unsigned long now = millis();
  for (auto &btn : buttons) {
    bool reading = digitalRead(btn.pin);
    if (reading != btn.lastReading) {
      btn.lastReading = reading;
      btn.lastChangeTime = now;
      btn.actionTakenOnPress = false; // Reset flag when button state changes (press or release)
    }
    if ((now - btn.lastChangeTime) >= DEBOUNCE_MS && reading != btn.stableState) {
      btn.stableState = reading; 
      // For non-continuous buttons, actionTakenOnPress ensures single action per press
      // For continuous buttons (UP/DOWN), we don't use actionTakenOnPress, action happens if LOW
    }

    // After debouncing, if button is firmly pressed (stableState is LOW)
    if (btn.stableState == LOW) { 
        if (btn.pin == UP_PIN) {
          servoAnglesIncrease(); // Continuous action while held
        } else if (btn.pin == DOWN_PIN) {
          servoAnglesDecrease(); // Continuous action while held
        } else if (btn.pin == RESET_PIN) {
          if (!btn.actionTakenOnPress) {
            resetToMinPosition(); 
            btn.actionTakenOnPress = true;
          }
        } else if (btn.pin == CENTER_JOY_PIN) {
          if (!btn.actionTakenOnPress) {
            moveToCenterPosition();
            btn.actionTakenOnPress = true;
          }
        }
      } else { // Button is HIGH (released or not pressed)
          btn.actionTakenOnPress = false; // Reset flag when button is released
      } 
  }
}

void setup() {
    Serial.begin(9600); 
    Serial.println("遥感(速率)和按钮控制 - V5 (长按)");
    
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);
    
    pinMode(UP_PIN, INPUT_PULLUP);
    pinMode(DOWN_PIN, INPUT_PULLUP);
    pinMode(RESET_PIN, INPUT_PULLUP);
    pinMode(CENTER_JOY_PIN, INPUT_PULLUP); 

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    
    moveServos(currentServo1Pos, currentServo2Pos, currentServo3Pos);
    Serial.println("初始位置已设置."); // currentServoXPos are already initialized
}

void loop() {
    handleButtons();
    mapJoystickToServos();
    delay(SERVO_UPDATE_RATE);
}