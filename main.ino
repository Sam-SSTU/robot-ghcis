#include <Servo.h>

// 定义舵机对象
Servo servo11;
Servo servo10;
Servo servo9;

// 舵机引脚
const int PIN_SERVO11 = 11;
const int PIN_SERVO10 = 10;
const int PIN_SERVO9 = 9;

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
  servo11.attach(PIN_SERVO11);
  servo10.attach(PIN_SERVO10);
  servo9.attach(PIN_SERVO9);

  servo11.write(angle11);
  servo10.write(angle10);
  servo9.write(angle9);

  Serial.begin(9600);
  Serial.println("Platform Control Ready!");
  Serial.println("Commands: U (up), D (down), T1 (tilt to servo11), T2 (tilt to servo10), T3 (tilt to servo9), R (reset)");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "U") {
      moveUp();
    } else if (input == "D") {
      moveDown();
    } else if (input == "T1") {
      tiltToServo11();
    } else if (input == "T2") {
      tiltToServo10();
    } else if (input == "T3") {
      tiltToServo9();
    } else if (input == "R") {
      resetPosition();
    } else {
      Serial.println("Invalid command! Use: U, D, T1, T2, T3, R");
    }

    // 输出当前角度
    Serial.print("Angles: S11=");
    Serial.print(angle11);
    Serial.print(", S10=");
    Serial.print(angle10);
    Serial.print(", S9=");
    Serial.println(angle9);
  }
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