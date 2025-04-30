#include <Servo.h>

// 定义舵机对象
Servo servo11;
Servo servo10;
Servo servo9;

// 舵机引脚
const int PIN_SERVO11 = 11;
const int PIN_SERVO10 = 10;
const int PIN_SERVO9 = 9;

// 遥感引脚
const int X_PIN = A0;  // X轴接A0
const int Y_PIN = A1;  // Y轴接A1

// 按钮引脚
const int BTN_4 = 4;   // 按钮4 - 全部向上
const int BTN_3 = 3;   // 按钮3 - 全部向下
const int BTN_2 = 2;   // 按钮2 - 复位

// 当前舵机角度
int angle11 = 0;
int angle10 = 0;
int angle9 = 0;

// 每次角度调整的步长
const int ANGLE_STEP = 20;

// 每个舵机的角度范围限制
const int MIN_ANGLE_11 = 0;
const int MAX_ANGLE_11 = 180;

const int MIN_ANGLE_10 = 0;
const int MAX_ANGLE_10 = 160;

const int MIN_ANGLE_9 = 10;
const int MAX_ANGLE_9 = 170;

void setup() {
  // 初始化舵机
  servo11.attach(PIN_SERVO11);
  servo10.attach(PIN_SERVO10);
  servo9.attach(PIN_SERVO9);

  // 设置按钮引脚为输入模式
  pinMode(BTN_4, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);

  // 初始化舵机位置
  servo11.write(angle11);
  servo10.write(angle10);
  servo9.write(angle9);

  Serial.begin(9600);
  Serial.println("Platform Control Ready!");
}

void loop() {
  // 读取遥感值
  int xValue = analogRead(X_PIN);
  int yValue = analogRead(Y_PIN);
  
  // 读取按钮状态
  int btn4State = digitalRead(BTN_4);
  int btn3State = digitalRead(BTN_3);
  int btn2State = digitalRead(BTN_2);
  
  // 遥感中间值和阈值
  const int threshold = 100;
  const int center = 512;
  
  // 处理按钮操作
  if (btn4State == LOW) {  // 全部向上
    moveUp();
  }
  else if (btn3State == LOW) {  // 全部向下
    moveDown();
  }
  else if (btn2State == LOW) {  // 复位
    resetPosition();
  }
  else {
    // 处理遥感控制
    // 右上区域 - 拉11号（减小角度）
    if (xValue > (center + threshold) && yValue < (center - threshold)) {
      if (angle11 > MIN_ANGLE_11) {
        angle11 = max(angle11 - ANGLE_STEP, MIN_ANGLE_11);
      }
    }
    // 右下区域 - 放11号（增加角度）
    else if (xValue > (center + threshold) && yValue > (center + threshold)) {
      if (angle11 < MAX_ANGLE_11) {
        angle11 = min(angle11 + ANGLE_STEP, MAX_ANGLE_11);
      }
    }
    
    // 左上区域 - 拉10号（减小角度）
    if (xValue < (center - threshold) && yValue < (center - threshold)) {
      if (angle10 > MIN_ANGLE_10) {
        angle10 = max(angle10 - ANGLE_STEP, MIN_ANGLE_10);
      }
    }
    // 左下区域 - 放10号（增加角度）
    else if (xValue < (center - threshold) && yValue > (center + threshold)) {
      if (angle10 < MAX_ANGLE_10) {
        angle10 = min(angle10 + ANGLE_STEP, MAX_ANGLE_10);
      }
    }
    
    // 上方区域中间 - 放9号（增加角度）
    if (abs(xValue - center) < threshold && yValue < (center - threshold)) {
      if (angle9 < MAX_ANGLE_9) {
        angle9 = min(angle9 + ANGLE_STEP, MAX_ANGLE_9);
      }
    }
    // 下方区域中间 - 收9号（减小角度）
    else if (abs(xValue - center) < threshold && yValue > (center + threshold)) {
      if (angle9 > MIN_ANGLE_9) {
        angle9 = max(angle9 - ANGLE_STEP, MIN_ANGLE_9);
      }
    }
    
    // 更新舵机位置
    updateServos();
  }
  
  // 打印调试信息
  printStatus(xValue, yValue);
  
  delay(100);  // 短暂延时，避免操作太快
}

void moveUp() {
  bool moved = false;

  if (angle11 < MAX_ANGLE_11) {
    angle11 = min(angle11 + ANGLE_STEP, MAX_ANGLE_11);
    moved = true;
  }
  if (angle10 < MAX_ANGLE_10) {
    angle10 = min(angle10 + ANGLE_STEP, MAX_ANGLE_10);
    moved = true;
  }
  if (angle9 < MAX_ANGLE_9) {
    angle9 = min(angle9 + ANGLE_STEP, MAX_ANGLE_9);
    moved = true;
  }

  if (moved) {
    servo11.write(angle11);
    servo10.write(angle10);
    servo9.write(angle9);
    Serial.println("Moving up...");
  } else {
    Serial.println("Max angle reached!");
  }
}

void moveDown() {
  bool moved = false;

  if (angle11 > MIN_ANGLE_11) {
    angle11 = max(angle11 - ANGLE_STEP, MIN_ANGLE_11);
    moved = true;
  }
  if (angle10 > MIN_ANGLE_10) {
    angle10 = max(angle10 - ANGLE_STEP, MIN_ANGLE_10);
    moved = true;
  }
  if (angle9 > MIN_ANGLE_9) {
    angle9 = max(angle9 - ANGLE_STEP, MIN_ANGLE_9);
    moved = true;
  }

  if (moved) {
    servo11.write(angle11);
    servo10.write(angle10);
    servo9.write(angle9);
    Serial.println("Moving down...");
  } else {
    Serial.println("Min angle reached!");
  }
}

void tiltToServo11() {
  if (angle11 < MAX_ANGLE_11) {
    angle11 = min(angle11 + ANGLE_STEP, MAX_ANGLE_11);
    servo11.write(angle11);
    servo10.write(angle10);
    servo9.write(angle9);
    Serial.println("Tilting to Servo11...");
  } else {
    Serial.println("Servo11 max angle reached!");
  }
}

void tiltToServo10() {
  if (angle10 < MAX_ANGLE_10) {
    angle10 = min(angle10 + ANGLE_STEP, MAX_ANGLE_10);
    servo11.write(angle11);
    servo10.write(angle10);
    servo9.write(angle9);
    Serial.println("Tilting to Servo10...");
  } else {
    Serial.println("Servo10 max angle reached!");
  }
}

void tiltToServo9() {
  if (angle9 < MAX_ANGLE_9) {
    angle9 = min(angle9 + ANGLE_STEP, MAX_ANGLE_9);
    servo11.write(angle11);
    servo10.write(angle10);
    servo9.write(angle9);
    Serial.println("Tilting to Servo9...");
  } else {
    Serial.println("Servo9 max angle reached!");
  }
}

void resetPosition() {
  angle11 = MIN_ANGLE_11;
  angle10 = MIN_ANGLE_10;
  angle9 = MIN_ANGLE_9;

  servo11.write(angle11);
  servo10.write(angle10);
  servo9.write(angle9);

  Serial.println("Reset to initial position.");
}

void updateServos() {
  servo11.write(angle11);
  servo10.write(angle10);
  servo9.write(angle9);
}

void printStatus(int xValue, int yValue) {
  Serial.print("X值: ");
  Serial.print(xValue);
  Serial.print(" Y值: ");
  Serial.print(yValue);
  Serial.print(" 舵机角度 - 11号: ");
  Serial.print(angle11);
  Serial.print(" 10号: ");
  Serial.print(angle10);
  Serial.print(" 9号: ");
  Serial.println(angle9);
}