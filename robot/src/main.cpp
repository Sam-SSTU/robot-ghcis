#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>             // Included for completeness, software SPI is used

// TFT Pin Definitions (Software SPI)
#define TFT_SCLK  7  // SPI Clock
#define TFT_MOSI  6  // SPI Data (Master Out Slave In)
#define TFT_CS    5  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   A0 // Reset pin
#define TFT_BL    A2 // Backlight Control Pin (Connect to 5V or 3.3V through a resistor if always on, or to this pin for software control)

#define ST77XX_DARKGREY 0x7BEF // Define a dark grey color (16-bit RGB565)

// Initialize Adafruit ST7789 driver object for software SPI
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

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
const byte MOS_CONTROL_BUTTON_PIN = 4; // Renamed from RESET_PIN, this is for MOS control
const byte CENTER_JOY_PIN = A3; // 手动回中按钮 (舵机到预设中心)

// Define MOS管引脚
const byte MOS_PIN = 3;

// 控制参数
const float DEADZONE = 0.2;      // 遥感死区大小(0-1)
const int SERVO_UPDATE_RATE = 05; // 主循环延迟(ms)
const unsigned long DEBOUNCE_MS = 50; // 按钮消抖时间 (Increased from 5)
const int ANGLE_STEP = 3;       // 按钮控制的舵机角度步长 (减小以便长按时更平滑)
const float JOYSTICK_SENSITIVITY = 0.02f; // 遥感速率控制灵敏度

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
  { "MOS_CTRL", MOS_CONTROL_BUTTON_PIN, HIGH, HIGH, 0, false }, // Changed from "RESET" and RESET_PIN
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
void setupDisplay(); // New function for TFT setup
void updateDisplay_cooperative(); // New function for TFT update

// Display update timing and state variables (restored)
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 20; // Keep this short for responsiveness

// Variables for display state and selective updates
int prev_joystickX_display = -1;
int prev_joystickY_display = -1;
bool prev_button_states_display[sizeof(buttons)/sizeof(Button)]; 
String prev_joyX_str_display = "";
String prev_joyY_str_display = "";

// Enum for cooperative display update states (restored)
enum DisplayUpdateState {
    UPDATE_JOY_X_TEXT,
    UPDATE_JOY_Y_TEXT,
    UPDATE_JOY_GRAPHIC_BOX, // Draw the box once or if erased
    UPDATE_JOY_GRAPHIC_DOT,
    UPDATE_BUTTON_0, // Will iterate through buttons
    UPDATE_BUTTON_1,
    UPDATE_BUTTON_2,
    UPDATE_BUTTON_3,
    DISPLAY_UPDATE_COMPLETE
};
DisplayUpdateState currentDisplayState = UPDATE_JOY_X_TEXT;
bool initial_draw_complete = false; // Flag to ensure full draw once

void setupDisplay() {
  #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  #endif

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_WHITE);
  tft.setTextColor(ST77XX_BLACK);

  // Draw centered title
  tft.setTextSize(2);
  const char* title = "Robot Control";
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((320 - w) / 2, 5); // Center horizontally, 5 pixels from top
  tft.print(title);
  
  // Draw horizontal line under title
  tft.drawFastHLine(10, 25, 300, ST77XX_BLACK);
  
  Serial.println("LCD Initialized in Landscape Mode.");

  for (size_t i = 0; i < sizeof(buttons)/sizeof(Button); ++i) {
      prev_button_states_display[i] = !buttons[i].stableState;
  }
}

// Restored updateDisplay_cooperative function
void updateDisplay_cooperative() {
    if (millis() - lastDisplayUpdateTime < DISPLAY_UPDATE_INTERVAL && initial_draw_complete) {
        return; 
    }
    lastDisplayUpdateTime = millis();

    tft.setTextSize(1);
    switch (currentDisplayState) {
        case UPDATE_JOY_X_TEXT: {
            String current_joyX_str = "JoyX: " + String(joystickX);
            if (current_joyX_str != prev_joyX_str_display || !initial_draw_complete) {
                tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
                tft.fillRect(10, 8, 70, 12, ST77XX_WHITE); // Clear area
                tft.setCursor(10, 10);
                tft.print(current_joyX_str);
                prev_joyX_str_display = current_joyX_str;
            }
            currentDisplayState = UPDATE_JOY_Y_TEXT;
            break;
        }
        case UPDATE_JOY_Y_TEXT: {
            String current_joyY_str = "JoyY: " + String(joystickY);
            if (current_joyY_str != prev_joyY_str_display || !initial_draw_complete) {
                tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
                tft.fillRect(10, 20, 70, 12, ST77XX_WHITE); // Clear area
                tft.setCursor(10, 22);
                tft.print(current_joyY_str);
                prev_joyY_str_display = current_joyY_str;
            }
            currentDisplayState = UPDATE_JOY_GRAPHIC_BOX;
            break;
        }
        case UPDATE_JOY_GRAPHIC_BOX: { 
            if (!initial_draw_complete) { // Only draw initially or if explicitly needed
                 int joyBoxX = 10, joyBoxY = 40, joyBoxSize = 50;
                 tft.drawRect(joyBoxX, joyBoxY, joyBoxSize, joyBoxSize, ST77XX_BLACK);
            }
            currentDisplayState = UPDATE_JOY_GRAPHIC_DOT;
            break;
        }
        case UPDATE_JOY_GRAPHIC_DOT: {
            int joyBoxX = 10, joyBoxY = 40, joyBoxSize = 50;
            if (prev_joystickX_display != -1) {
                int prev_dotX = map(prev_joystickY_display, 0, 1023, joyBoxX + 2, joyBoxX + joyBoxSize - 3);
                int prev_dotY = map(prev_joystickX_display, 1023, 0, joyBoxY + 2, joyBoxY + joyBoxSize - 3);
                if (joystickX != prev_joystickX_display || joystickY != prev_joystickY_display || !initial_draw_complete) {
                    tft.fillCircle(prev_dotX, prev_dotY, 3, ST77XX_WHITE);
                }
            }
            if (joystickX != prev_joystickX_display || joystickY != prev_joystickY_display || !initial_draw_complete) {
                int current_dotX = map(joystickY, 0, 1023, joyBoxX + 2, joyBoxX + joyBoxSize - 3);
                int current_dotY = map(joystickX, 1023, 0, joyBoxY + 2, joyBoxY + joyBoxSize - 3);
                tft.fillCircle(current_dotX, current_dotY, 3, ST77XX_RED);
            }
            prev_joystickX_display = joystickX;
            prev_joystickY_display = joystickY;
            currentDisplayState = UPDATE_BUTTON_0;
            break;
        }
        case UPDATE_BUTTON_0:
        case UPDATE_BUTTON_1:
        case UPDATE_BUTTON_2:
        case UPDATE_BUTTON_3: {
            size_t button_idx = currentDisplayState - UPDATE_BUTTON_0;
            if (button_idx < sizeof(buttons)/sizeof(Button)) {
                int buttonDisplayStartX = 150, buttonDisplayStartY = 40;
                int buttonRectHeight = 18, buttonRectWidth = 55, lineSpacing = 25;
                int currentY = buttonDisplayStartY + (button_idx * lineSpacing);

                if (buttons[button_idx].stableState != prev_button_states_display[button_idx] || !initial_draw_complete) {
                    tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
                    tft.fillRect(buttonDisplayStartX, currentY, 40 + buttonRectWidth + 10, buttonRectHeight, ST77XX_WHITE);

                    tft.setCursor(buttonDisplayStartX, currentY + (buttonRectHeight/2) - 4);
                    tft.print(String(buttons[button_idx].name) + ":");
                    int rectX = buttonDisplayStartX + 40;
                    int16_t x1, y1; uint16_t w, h;

                    // Invert the display logic for pins 4, 11, and 12
                    bool isInvertedButton = (buttons[button_idx].pin == 4 || 
                                           buttons[button_idx].pin == 11 || 
                                           buttons[button_idx].pin == 12);
                    bool displayAsPressed = isInvertedButton ? 
                                          (buttons[button_idx].stableState == HIGH) : 
                                          (buttons[button_idx].stableState == LOW);

                    if (displayAsPressed) {
                        tft.fillRect(rectX, currentY, buttonRectWidth, buttonRectHeight, ST77XX_GREEN);
                        tft.setTextColor(ST77XX_WHITE);
                        tft.getTextBounds("ON", rectX, currentY, &x1, &y1, &w, &h);
                        tft.setCursor(rectX + (buttonRectWidth - w) / 2, currentY + (buttonRectHeight - h) / 2 + h);
                        tft.print("ON");
                    } else {
                        tft.fillRect(rectX, currentY, buttonRectWidth, buttonRectHeight, ST77XX_DARKGREY);
                        tft.setTextColor(ST77XX_WHITE);
                        tft.getTextBounds("OFF", rectX, currentY, &x1, &y1, &w, &h);
                        tft.setCursor(rectX + (buttonRectWidth - w) / 2, currentY + (buttonRectHeight - h) / 2 + h);
                        tft.print("OFF");
                    }
                    prev_button_states_display[button_idx] = buttons[button_idx].stableState;
                }
            }
            
            // Draw electromagnetic status box
            if (button_idx == 0 || !initial_draw_complete) {
                int statusBoxX = 10, statusBoxY = 150;
                int statusBoxWidth = 100, statusBoxHeight = 60;
                
                tft.drawRect(statusBoxX, statusBoxY, statusBoxWidth, statusBoxHeight, ST77XX_BLACK);
                tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
                tft.setCursor(statusBoxX + 5, statusBoxY + 5);
                tft.print("Magnet:");
                
                bool magnetState = digitalRead(MOS_PIN);
                tft.fillRect(statusBoxX + 5, statusBoxY + 25, 
                            statusBoxWidth - 10, statusBoxHeight - 30, 
                            magnetState ? ST77XX_GREEN : ST77XX_RED);
            }

            if (button_idx + 1 < sizeof(buttons)/sizeof(Button)) {
                currentDisplayState = (DisplayUpdateState)(UPDATE_BUTTON_0 + button_idx + 1);
            } else {
                currentDisplayState = DISPLAY_UPDATE_COMPLETE;
            }
            break;
        }
        case DISPLAY_UPDATE_COMPLETE: {
            initial_draw_complete = true; 
            currentDisplayState = UPDATE_JOY_X_TEXT; // Loop back for next full update cycle
            break;
        }
    } // end switch
}

void setup() {
    Serial.begin(9600);
    Serial.println("遥感(速率)和按钮控制 - V5 (长按) + LCD");

    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);
    
    pinMode(UP_PIN, INPUT_PULLUP);
    pinMode(DOWN_PIN, INPUT_PULLUP);
    pinMode(MOS_CONTROL_BUTTON_PIN, INPUT_PULLUP); // Changed from RESET_PIN
    pinMode(CENTER_JOY_PIN, INPUT_PULLUP);
    pinMode(MOS_PIN, OUTPUT); // MOS_PIN setup
    digitalWrite(MOS_PIN, LOW); // Set initial state for MOS_PIN

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    
    moveServos(currentServo1Pos, currentServo2Pos, currentServo3Pos);
    setupDisplay(); // Initialize the LCD Display
    Serial.println("初始位置已设置."); // currentServoXPos are already initialized
}

void loop() {
    handleButtons();
    mapJoystickToServos();
    updateDisplay_cooperative(); // Restore direct call to cooperative display update
    //delay(SERVO_UPDATE_RATE); // Re-enable delay with new, smaller SERVO_UPDATE_RATE
}

// Servo and Button logic functions (existing - ensure they are complete)
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

void moveServos(float s1, float s2, float s3) {
    currentServo1Pos = constrain(s1, (float)MIN_ANGLE_1, (float)MAX_ANGLE_1);
    currentServo2Pos = constrain(s2, (float)MIN_ANGLE_2, (float)MAX_ANGLE_2);
    currentServo3Pos = constrain(s3, (float)MIN_ANGLE_3, (float)MAX_ANGLE_3);
    
    servo1.write(round(currentServo1Pos));
    servo2.write(round(currentServo2Pos));
    servo3.write(round(currentServo3Pos));
    
    static unsigned long lastServoDebugTime = 0;
    if (millis() - lastServoDebugTime > 200) {
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
}

void servoAnglesIncrease() {
    moveServos(currentServo1Pos + ANGLE_STEP, 
               currentServo2Pos + ANGLE_STEP, 
               currentServo3Pos + ANGLE_STEP);
}

void servoAnglesDecrease() {
    moveServos(currentServo1Pos - ANGLE_STEP, 
               currentServo2Pos - ANGLE_STEP, 
               currentServo3Pos - ANGLE_STEP);
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
    float strength = sqrt(x_mapped*x_mapped + y_mapped*y_mapped);

    if (strength / 100.0f < DEADZONE) {
        return; 
    }
    
    float normalized_strength = (strength - (DEADZONE * 100.0f)) / (100.0f - (DEADZONE * 100.0f));
    normalized_strength = constrain(normalized_strength, 0.0f, 1.0f);

    ServoAngles targetDirectionPos = interpolateDirection(angle_deg);

    float delta1 = (targetDirectionPos.servo1 - currentServo1Pos) * normalized_strength * JOYSTICK_SENSITIVITY;
    float delta2 = (targetDirectionPos.servo2 - currentServo2Pos) * normalized_strength * JOYSTICK_SENSITIVITY;
    float delta3 = (targetDirectionPos.servo3 - currentServo3Pos) * normalized_strength * JOYSTICK_SENSITIVITY;

    moveServos(currentServo1Pos + delta1, 
               currentServo2Pos + delta2, 
               currentServo3Pos + delta3);

    static unsigned long lastJoyDebug = 0;
    if (millis() - lastJoyDebug > 250) {
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
    }
    if ((now - btn.lastChangeTime) >= DEBOUNCE_MS && reading != btn.stableState) {
      btn.stableState = reading; 
    }

    if (btn.stableState == LOW) { 
        if (btn.pin == UP_PIN) {
          servoAnglesIncrease();
        } else if (btn.pin == DOWN_PIN) {
          servoAnglesDecrease();
        } else if (btn.pin == MOS_CONTROL_BUTTON_PIN) { 
          if (!btn.actionTakenOnPress) {
            bool currentMosState = digitalRead(MOS_PIN);
            bool newMosState = !currentMosState;
            digitalWrite(MOS_PIN, newMosState);
            Serial.print("MOS_PIN (Pin 3) is now: ");
            Serial.println(newMosState ? "HIGH" : "LOW");
            btn.actionTakenOnPress = true;
          }
        } else if (btn.pin == CENTER_JOY_PIN) {
          if (!btn.actionTakenOnPress) {
            moveToCenterPosition();
            btn.actionTakenOnPress = true;
          }
        }
      } else { 
          btn.actionTakenOnPress = false;
      } 
  }
}
// End of existing servo and button logic