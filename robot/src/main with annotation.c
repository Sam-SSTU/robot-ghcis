#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_GFX.h>    // 核心图形库
#include <Adafruit_ST7789.h> // ST7789 特定硬件库
#include <SPI.h>             // 包含此库用于完整性，实际使用软件 SPI

// TFT 显示屏引脚定义 (软件 SPI)
#define TFT_SCLK  7  // SPI 时钟引脚
#define TFT_MOSI  6  // SPI 数据引脚 (主出从入)
#define TFT_CS    5  // SPI 片选控制引脚
#define TFT_DC    2  // SPI 数据/命令控制引脚
#define TFT_RST   A0 // 显示屏复位引脚
#define TFT_BL    A2 // 显示屏背光控制引脚 (如果常亮，可通过电阻连接到5V或3.3V；或连接此引脚以进行软件控制)

#define ST77XX_DARKGREY 0x7BEF // 定义深灰色 (16位 RGB565 颜色格式)

// 初始化 Adafruit ST7789 驱动对象 (用于软件 SPI)
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

/* -----------------------------------------------------------
 *  遥感和按钮控制舵机程序
 *  - 遥感控制：X轴-A4，Y轴-A5 (比例速率控制)
 *  - 按钮控制：
 *    上按钮(D12) - 所有舵机角度增加 (长按连续)
 *    下按钮(D11) - 所有舵机角度减少 (长按连续)
 *    MOS控制按钮(D4) - 控制电磁铁开关 (单次)
 *    回中按钮(A3) - 所有舵机到预设中心点 (单次)
 *  - 使用结构体和时间戳进行按钮消抖
 *  - 遥感无自动回中，控制舵机移动速率
 * -----------------------------------------------------------
 */

// 定义舵机连接的引脚
const int SERVO1_PIN = 10; // 舵机1引脚
const int SERVO2_PIN = 9;  // 舵机2引脚
const int SERVO3_PIN = 8;  // 舵机3引脚

// 定义遥感连接的引脚
const byte JOYSTICK_X_PIN = A4;  // 遥感X轴模拟输入引脚
const byte JOYSTICK_Y_PIN = A5;  // 遥感Y轴模拟输入引脚

// 定义按钮连接的引脚
const byte UP_PIN = 12;        // 向上按钮 (物理上的"上"按钮，用于增加舵机角度)
const byte DOWN_PIN = 11;      // 向下按钮 (物理上的"下"按钮，用于减少舵机角度)
const byte MOS_CONTROL_BUTTON_PIN = 4; // MOS管控制按钮引脚 (原RESET_PIN)
const byte CENTER_JOY_PIN = A3; // 手动回中按钮 (将舵机移动到预设中心位置)

// 定义MOS管控制引脚
const byte MOS_PIN = 3; // MOS管栅极驱动引脚

// 控制参数
const float DEADZONE = 0.2;      // 遥感死区大小 (0-1的比例，中心区域的无效范围)
const int SERVO_UPDATE_RATE = 05; // 主循环延迟时间 (毫秒)，影响舵机响应和屏幕刷新率
const unsigned long DEBOUNCE_MS = 50; // 按钮消抖时间 (毫秒)，防止按钮抖动引起的误触发 (从5毫秒增加)
const int ANGLE_STEP = 3;       // 按钮控制舵机时，每次增加或减少的角度步长 (减小以便长按时更平滑)
const float JOYSTICK_SENSITIVITY = 0.02f; // 遥感速率控制灵敏度，影响遥感输入转换为舵机速度的比例

// 每个舵机的角度范围限制 (0-180度)
const int MIN_ANGLE_1 = 0;   // 舵机1最小角度
const int MAX_ANGLE_1 = 180; // 舵机1最大角度
const int MIN_ANGLE_2 = 0;   // 舵机2最小角度
const int MAX_ANGLE_2 = 180; // 舵机2最大角度
const int MIN_ANGLE_3 = 0;   // 舵机3最小角度
const int MAX_ANGLE_3 = 180; // 舵机3最大角度

// 存储遥感读数的变量
int joystickX; // 存储遥感X轴的原始读数 (0-1023)
int joystickY; // 存储遥感Y轴的原始读数 (0-1023)

// 创建舵机对象
Servo servo1; // 舵机1对象
Servo servo2; // 舵机2对象
Servo servo3; // 舵机3对象

// 当前舵机角度 (使用浮点数以实现平滑的速率控制)
float currentServo1Pos = 180.0f; // 舵机1当前角度，初始化为180度
float currentServo2Pos = 180.0f; // 舵机2当前角度，初始化为180度
float currentServo3Pos = 180.0f; // 舵机3当前角度，初始化为180度

// 定义舵机中立位置 (用于手动回中按钮A3)
const int SERVO1_CENTER = 180; // 舵机1的中心位置
const int SERVO2_CENTER = 180; // 舵机2的中心位置
const int SERVO3_CENTER = 180; // 舵机3的中心位置

// 按钮结构体，用于处理按钮消抖和状态管理
struct Button {
  const char*   name;           // 按钮名称 (用于调试或显示)
  byte          pin;            // 按钮连接的引脚
  bool          stableState;    // 消抖后的稳定状态 (LOW表示按下，HIGH表示松开，因为使用INPUT_PULLUP)
  bool          lastReading;    // 上一次的原始读数
  unsigned long lastChangeTime; // 上一次读数发生变化的时间戳
  bool          actionTakenOnPress; // 标志位，确保非连续性按钮 (如MOS控制、回中) 按下时只执行一次动作
};

Button buttons[] = { // 按钮数组，包含所有按钮的配置
  { "UP",       UP_PIN,         HIGH, HIGH, 0, false }, // 向上按钮
  { "DOWN",     DOWN_PIN,       HIGH, HIGH, 0, false }, // 向下按钮
  { "MOS_CTRL", MOS_CONTROL_BUTTON_PIN, HIGH, HIGH, 0, false }, // MOS控制按钮 (原"RESET"按钮)
  { "CENTER",   CENTER_JOY_PIN, HIGH, HIGH, 0, false }  // 回中按钮
};

// 定义基准方向的舵机角度 (遥感用 - 代表摇杆在特定方向全速偏转时的舵机目标角度)
struct ServoAngles { // 存储一组三个舵机角度的结构体
    int servo1; // 舵机1的角度
    int servo2; // 舵机2的角度
    int servo3; // 舵机3的角度
};

const ServoAngles BASE_DIRECTIONS[] = { // 定义8个基本方向的目标舵机角度
    {180, 0, 60},    // 上 (对应遥感角度0度)
    {180, 0, 0},     // 右上 (对应遥感角度45度)
    {180, 180, 0},   // 右 (对应遥感角度90度)
    {0, 180, 0},     // 右下 (对应遥感角度135度)
    {0, 180, 60},    // 下 (对应遥感角度180度)
    {0, 180, 180},   // 左下 (对应遥感角度225度)
    {20, 10, 180},   // 左 (对应遥感角度270度)
    {180, 0, 180}    // 左上 (对应遥感角度315度)
};

// 函数声明
void moveServos(float s1, float s2, float s3); // 控制三个舵机移动到指定角度 (接受浮点数参数)
void moveToCenterPosition(); // 将所有舵机移动到预设的中心位置
void servoAnglesIncrease(); // 增加所有舵机的角度 (用于向上按钮)
void servoAnglesDecrease(); // 减少所有舵机的角度 (用于向下按钮)
void resetToMinPosition();  // 将所有舵机复位到最小角度 (此函数在当前代码中未被MOS_CONTROL_BUTTON_PIN调用)
ServoAngles interpolateDirection(float angle); // 根据遥感角度插值计算目标舵机角度
void mapJoystickToServos(); // 将遥感读数映射到舵机控制信号
void handleButtons();       // 处理所有按钮的输入和消抖
void setupDisplay();        // 初始化TFT显示屏的函数
void updateDisplay_cooperative(); // 分时合作式更新TFT显示屏内容的函数

// 显示更新相关的时间和状态变量 (已恢复)
unsigned long lastDisplayUpdateTime = 0; // 上一次显示更新的时间戳
const unsigned long DISPLAY_UPDATE_INTERVAL = 20; // 显示更新的最小间隔时间 (毫秒)，保持较小值以提高响应性

// 用于显示状态和选择性更新的变量
int prev_joystickX_display = -1; // 上一次显示的遥感X轴值
int prev_joystickY_display = -1; // 上一次显示的遥感Y轴值
bool prev_button_states_display[sizeof(buttons)/sizeof(Button)]; // 上一次显示的按钮状态数组
String prev_joyX_str_display = ""; // 上一次显示的遥感X轴文本
String prev_joyY_str_display = ""; // 上一次显示的遥感Y轴文本

// 用于分时合作式显示更新的状态枚举 (已恢复)
enum DisplayUpdateState {
    UPDATE_JOY_X_TEXT,      // 更新遥感X轴文本状态
    UPDATE_JOY_Y_TEXT,      // 更新遥感Y轴文本状态
    UPDATE_JOY_GRAPHIC_BOX, // 更新遥感图形的边框状态 (通常只绘制一次)
    UPDATE_JOY_GRAPHIC_DOT, // 更新遥感图形的点状态
    UPDATE_BUTTON_0,        // 更新第一个按钮的显示状态 (后续会迭代所有按钮)
    UPDATE_BUTTON_1,        // 更新第二个按钮的显示状态
    UPDATE_BUTTON_2,        // 更新第三个按钮的显示状态
    UPDATE_BUTTON_3,        // 更新第四个按钮的显示状态
    DISPLAY_UPDATE_COMPLETE // 显示更新完成状态
};
DisplayUpdateState currentDisplayState = UPDATE_JOY_X_TEXT; // 当前显示更新状态，初始为更新X轴文本
bool initial_draw_complete = false; // 标志位，表示初始的完整绘制是否已完成

// 初始化TFT显示屏的函数
void setupDisplay() {
  #ifdef TFT_BL // 如果定义了TFT_BL (背光控制引脚)
    pinMode(TFT_BL, OUTPUT);    // 将背光引脚设置输出模式
    digitalWrite(TFT_BL, HIGH); // 点亮背光
  #endif

  tft.init(240, 320); // 初始化ST7789驱动，设置屏幕分辨率为240x320
  tft.setRotation(1); // 设置屏幕旋转方向 (1 通常表示横向，具体取决于屏幕和库的配置)
  tft.fillScreen(ST77XX_WHITE); // 用白色填充整个屏幕作为背景
  tft.setTextColor(ST77XX_BLACK); // 设置文本颜色为黑色

  // 绘制居中的标题文本
  tft.setTextSize(2); // 设置文本大小为2
  const char* title = "Robot Control"; // 标题内容
  int16_t x1, y1; // 用于存储文本边界计算结果的变量
  uint16_t w, h;  // 用于存储文本宽度和高度的变量
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h); // 计算文本的边界框以获取宽度和高度
  tft.setCursor((320 - w) / 2, 5); // 设置光标位置，水平居中 (320是横向宽度)，垂直方向距离顶部5像素
  tft.print(title); // 打印标题
  
  // 在标题下方绘制一条水平线
  tft.drawFastHLine(10, 25, 300, ST77XX_BLACK); // 在 (10, 25) 位置绘制长度为300的黑色水平线
  
  Serial.println("LCD Initialized in Landscape Mode."); // 通过串口打印LCD初始化完成信息

  // 初始化上一次按钮状态的显示数组，确保首次更新时所有按钮都会被绘制
  for (size_t i = 0; i < sizeof(buttons)/sizeof(Button); ++i) {
      prev_button_states_display[i] = !buttons[i].stableState; // 将上一次显示状态设置为当前状态的相反值
  }
}

// Restored updateDisplay_cooperative function
// 分时合作式更新TFT显示屏内容的函数
// 这个函数通过一个状态机逐步更新屏幕的不同部分，避免单次占用过多处理时间，从而保持主循环的响应性。
void updateDisplay_cooperative() {
    // 如果距离上次更新时间未达到预设间隔，并且初始绘制已完成，则跳过本次更新
    if (millis() - lastDisplayUpdateTime < DISPLAY_UPDATE_INTERVAL && initial_draw_complete) {
        return; 
    }
    lastDisplayUpdateTime = millis(); // 更新上次显示时间戳

    tft.setTextSize(1); // 设置后续绘制文本的大小为1
    switch (currentDisplayState) { // 根据当前显示状态执行相应的更新操作
        case UPDATE_JOY_X_TEXT: { // 更新遥感X轴文本
            String current_joyX_str = "JoyX: " + String(joystickX); // 构建当前X轴文本
            // 如果当前文本与上次不同，或者尚未完成初始绘制，则更新显示
            if (current_joyX_str != prev_joyX_str_display || !initial_draw_complete) {
                tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 设置文本颜色为黑，背景为白 (用于擦除旧文本)
                tft.fillRect(10, 8, 70, 12, ST77XX_WHITE); // 用白色填充旧文本区域以清除
                tft.setCursor(10, 10); // 设置光标位置
                tft.print(current_joyX_str); // 打印新的X轴文本
                prev_joyX_str_display = current_joyX_str; // 保存当前文本作为下次比较的基准
            }
            currentDisplayState = UPDATE_JOY_Y_TEXT; // 切换到下一个显示状态：更新Y轴文本
            break;
        }
        case UPDATE_JOY_Y_TEXT: { // 更新遥感Y轴文本
            String current_joyY_str = "JoyY: " + String(joystickY); // 构建当前Y轴文本
            // 如果当前文本与上次不同，或者尚未完成初始绘制，则更新显示
            if (current_joyY_str != prev_joyY_str_display || !initial_draw_complete) {
                tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 设置文本颜色和背景色
                tft.fillRect(10, 20, 70, 12, ST77XX_WHITE); // 清除旧文本区域
                tft.setCursor(10, 22); // 设置光标位置
                tft.print(current_joyY_str); // 打印新的Y轴文本
                prev_joyY_str_display = current_joyY_str; // 保存当前文本
            }
            currentDisplayState = UPDATE_JOY_GRAPHIC_BOX; // 切换到下一个显示状态：更新遥感图形的边框
            break;
        }
        case UPDATE_JOY_GRAPHIC_BOX: { // 更新遥感图形的边框
            // 只有在初始绘制未完成时才绘制边框 (边框通常不需要频繁重绘)
            if (!initial_draw_complete) { 
                 int joyBoxX = 10, joyBoxY = 40, joyBoxSize = 50; // 定义遥感显示区域的位置和大小
                 tft.drawRect(joyBoxX, joyBoxY, joyBoxSize, joyBoxSize, ST77XX_BLACK); // 绘制黑色矩形边框
            }
            currentDisplayState = UPDATE_JOY_GRAPHIC_DOT; // 切换到下一个显示状态：更新遥感图形的点
            break;
        }
        case UPDATE_JOY_GRAPHIC_DOT: { // 更新遥感图形的点 (代表摇杆位置)
            int joyBoxX = 10, joyBoxY = 40, joyBoxSize = 50; // 遥感显示区域参数
            // 如果存在上一次的点的位置记录 (非首次绘制)
            if (prev_joystickX_display != -1) {
                // 将上一次的遥感Y和X值映射到屏幕坐标 (注意这里X/Y与屏幕坐标的对应关系，可能需要根据实际情况调整)
                int prev_dotX = map(prev_joystickY_display, 0, 1023, joyBoxX + 2, joyBoxX + joyBoxSize - 3);
                int prev_dotY = map(prev_joystickX_display, 1023, 0, joyBoxY + 2, joyBoxY + joyBoxSize - 3); // Y轴通常是反向的
                // 如果遥感位置发生变化，或者尚未完成初始绘制，则擦除旧的点
                if (joystickX != prev_joystickX_display || joystickY != prev_joystickY_display || !initial_draw_complete) {
                    tft.fillCircle(prev_dotX, prev_dotY, 3, ST77XX_WHITE); // 用白色填充圆形以擦除旧点
                }
            }
            // 如果遥感位置发生变化，或者尚未完成初始绘制，则绘制新的点
            if (joystickX != prev_joystickX_display || joystickY != prev_joystickY_display || !initial_draw_complete) {
                // 将当前的遥感Y和X值映射到屏幕坐标
                int current_dotX = map(joystickY, 0, 1023, joyBoxX + 2, joyBoxX + joyBoxSize - 3);
                int current_dotY = map(joystickX, 1023, 0, joyBoxY + 2, joyBoxY + joyBoxSize - 3);
                tft.fillCircle(current_dotX, current_dotY, 3, ST77XX_RED); // 用红色绘制新的点
            }
            prev_joystickX_display = joystickX; // 保存当前X值作为下次比较的基准
            prev_joystickY_display = joystickY; // 保存当前Y值作为下次比较的基准
            currentDisplayState = UPDATE_BUTTON_0; // 切换到下一个显示状态：更新第一个按钮
            break;
        }
        case UPDATE_BUTTON_0: // 更新按钮0的显示状态
        case UPDATE_BUTTON_1: // 更新按钮1的显示状态
        case UPDATE_BUTTON_2: // 更新按钮2的显示状态
        case UPDATE_BUTTON_3: { // 更新按钮3的显示状态
            size_t button_idx = currentDisplayState - UPDATE_BUTTON_0; // 计算当前正在处理的按钮索引
            if (button_idx < sizeof(buttons)/sizeof(Button)) { // 确保按钮索引有效
                int buttonDisplayStartX = 150, buttonDisplayStartY = 40; // 按钮显示区域的起始X, Y坐标
                int buttonRectHeight = 18, buttonRectWidth = 55, lineSpacing = 25; // 按钮矩形的高度、宽度和行间距
                int currentY = buttonDisplayStartY + (button_idx * lineSpacing); // 计算当前按钮的Y坐标

                // 如果按钮的稳定状态与上次显示的状态不同，或者尚未完成初始绘制，则更新按钮显示
                if (buttons[button_idx].stableState != prev_button_states_display[button_idx] || !initial_draw_complete) {
                    tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 设置文本颜色和背景色
                    tft.fillRect(buttonDisplayStartX, currentY, 40 + buttonRectWidth + 10, buttonRectHeight, ST77XX_WHITE); // 清除按钮区域

                    tft.setCursor(buttonDisplayStartX, currentY + (buttonRectHeight/2) - 4); // 设置光标到按钮名称文本位置 (垂直居中)
                    tft.print(String(buttons[button_idx].name) + ":"); // 打印按钮名称
                    int rectX = buttonDisplayStartX + 40; // 状态指示矩形的X坐标
                    int16_t x1, y1; uint16_t w, h; // 用于文本边界计算

                    // 特殊处理引脚4, 11, 12的按钮，它们的按下状态是HIGH (因为INPUT_PULLUP, 按下是LOW, 但这里可能代表激活状态)
                    // 正常逻辑是stableState == LOW 表示按下。
                    // 此处逻辑：如果按钮引脚是4, 11,或12 (UP, DOWN, MOS_CTRL)，且stableState为HIGH，则认为它是"视觉上按下"的状态 (显示为ON)
                    // 否则，如果stableState为LOW，则认为它是"视觉上按下"的状态。
                    // 这个反转逻辑是为了匹配按钮按下时引脚变为LOW，但在UI上显示为ON。
                    bool isInvertedButton = (buttons[button_idx].pin == 4 || 
                                           buttons[button_idx].pin == 11 || 
                                           buttons[button_idx].pin == 12);
                    bool displayAsPressed; // 决定是否将按钮显示为"按下"状态 (绿色ON)
                    if (isInvertedButton) {
                        // 对于UP, DOWN, MOS_CTRL按钮 (使用INPUT_PULLUP，按下为LOW)
                        // 如果 stableState 是 HIGH (未按下)，则 displayAsPressed 为 false (显示OFF)
                        // 如果 stableState 是 LOW (按下)，则 displayAsPressed 为 true (显示ON)
                        // 这里原来的逻辑是反的，现在修正：按下(LOW)时显示ON
                        displayAsPressed = (buttons[button_idx].stableState == LOW);
                    } else {
                        // 对于CENTER_JOY_PIN按钮 (同样INPUT_PULLUP，按下为LOW)
                        // 按下(LOW)时显示ON
                        displayAsPressed = (buttons[button_idx].stableState == LOW);
                    }

                    if (displayAsPressed) { // 如果按钮应显示为按下状态
                        tft.fillRect(rectX, currentY, buttonRectWidth, buttonRectHeight, ST77XX_GREEN); // 绘制绿色矩形
                        tft.setTextColor(ST77XX_WHITE); // 设置文本颜色为白色 (在绿色背景上)
                        tft.getTextBounds("ON", rectX, currentY, &x1, &y1, &w, &h); // 计算"ON"文本边界
                        tft.setCursor(rectX + (buttonRectWidth - w) / 2, currentY + (buttonRectHeight - h) / 2 + h); // 文本居中显示
                        tft.print("ON"); // 打印"ON"
                    } else { // 如果按钮应显示为松开状态
                        tft.fillRect(rectX, currentY, buttonRectWidth, buttonRectHeight, ST77XX_DARKGREY); // 绘制深灰色矩形
                        tft.setTextColor(ST77XX_WHITE); // 设置文本颜色为白色
                        tft.getTextBounds("OFF", rectX, currentY, &x1, &y1, &w, &h); // 计算"OFF"文本边界
                        tft.setCursor(rectX + (buttonRectWidth - w) / 2, currentY + (buttonRectHeight - h) / 2 + h); // 文本居中显示
                        tft.print("OFF"); // 打印"OFF"
                    }
                    prev_button_states_display[button_idx] = buttons[button_idx].stableState; // 保存当前按钮状态作为下次比较的基准
                }
            }
            
            // 绘制电磁铁状态框 (仅在更新第一个按钮时或初始绘制时执行，减少重复绘制)
            if (button_idx == 0 || !initial_draw_complete) {
                int statusBoxX = 10, statusBoxY = 150; // 状态框位置
                int statusBoxWidth = 100, statusBoxHeight = 60; // 状态框尺寸
                
                tft.drawRect(statusBoxX, statusBoxY, statusBoxWidth, statusBoxHeight, ST77XX_BLACK); // 绘制状态框边框
                tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 设置文本颜色和背景
                tft.setCursor(statusBoxX + 5, statusBoxY + 5); // 设置光标位置
                tft.print("Magnet:"); // 打印标签
                
                bool magnetState = digitalRead(MOS_PIN); // 读取MOS管引脚状态 (即电磁铁状态)
                // 根据电磁铁状态填充颜色块 (绿色表示ON, 红色表示OFF)
                tft.fillRect(statusBoxX + 5, statusBoxY + 25, 
                            statusBoxWidth - 10, statusBoxHeight - 30, 
                            magnetState ? ST77XX_GREEN : ST77XX_RED);
            }

            // 如果还有下一个按钮需要更新，则切换到下一个按钮的更新状态
            if (button_idx + 1 < sizeof(buttons)/sizeof(Button)) {
                currentDisplayState = (DisplayUpdateState)(UPDATE_BUTTON_0 + button_idx + 1);
            } else { // 所有按钮都已更新完毕
                currentDisplayState = DISPLAY_UPDATE_COMPLETE; // 切换到显示更新完成状态
            }
            break;
        }
        case DISPLAY_UPDATE_COMPLETE: { // 显示更新完成
            initial_draw_complete = true; // 标记初始完整绘制已完成
            currentDisplayState = UPDATE_JOY_X_TEXT; // 重置状态，准备下一轮完整的更新周期
            break;
        }
    } // end switch
}

// Arduino 的初始化函数，在程序开始时仅执行一次
void setup() {
    Serial.begin(9600); // 初始化串口通信，波特率为9600
    Serial.println("遥感(速率)和按钮控制 - V5 (长按) + LCD"); // 打印程序版本信息到串口

    pinMode(JOYSTICK_X_PIN, INPUT); // 设置遥感X轴引脚为输入模式
    pinMode(JOYSTICK_Y_PIN, INPUT); // 设置遥感Y轴引脚为输入模式
    
    // 设置按钮引脚为输入上拉模式 (INPUT_PULLUP 表示引脚默认为高电平，按下时变为低电平)
    pinMode(UP_PIN, INPUT_PULLUP);       // 向上按钮
    pinMode(DOWN_PIN, INPUT_PULLUP);     // 向下按钮
    pinMode(MOS_CONTROL_BUTTON_PIN, INPUT_PULLUP); // MOS控制按钮 (原RESET_PIN)
    pinMode(CENTER_JOY_PIN, INPUT_PULLUP); // 回中按钮
    pinMode(MOS_PIN, OUTPUT);            // 设置MOS管控制引脚为输出模式
    digitalWrite(MOS_PIN, LOW);          // 初始化MOS管为关闭状态 (电磁铁不工作)

    // 将舵机对象附加到对应的控制引脚
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    
    // 将舵机移动到初始定义的当前位置 (currentServoXPos 变量的初始值)
    moveServos(currentServo1Pos, currentServo2Pos, currentServo3Pos);
    setupDisplay(); // 调用函数初始化LCD显示屏
    Serial.println("初始位置已设置."); // 打印舵机初始位置设置完成信息到串口
}

// Arduino 的主循环函数，在setup()执行完毕后会不断重复执行
void loop() {
    handleButtons();             // 调用函数处理按钮输入和逻辑
    mapJoystickToServos();       // 调用函数处理遥感输入并控制舵机
    updateDisplay_cooperative(); // 调用函数更新TFT显示屏内容 (分时合作式)
    //delay(SERVO_UPDATE_RATE); // 重新启用延迟，配合新的较小SERVO_UPDATE_RATE值，用于控制主循环频率，影响舵机和显示的平滑度
                                 // 注意：如果显示更新或按钮/遥感处理已经足够耗时，此处的delay可能需要调整或移除以避免响应迟缓
}

// Servo and Button logic functions (existing - ensure they are complete)
// 根据输入的角度 (0-360度)，在预定义的8个基准方向舵机角度之间进行线性插值，计算出目标舵机角度组合。
// 这个函数用于将连续的遥感方向映射到具体的舵机角度。
ServoAngles interpolateDirection(float angle) {
    // 将输入角度 (0-360) 转换为扇区索引 (0-7)，每个扇区45度。
    int baseSector = (int)(angle / 45.0f);
    if (baseSector < 0) baseSector = 0; // 确保扇区索引不小于0
    if (baseSector > 7) baseSector = 7; // 确保扇区索引不大于7 (对应 BASE_DIRECTIONS 数组大小 - 1)
    
    // 计算下一个扇区的索引，用于插值。如果当前是最后一个扇区，则下一个是第一个扇区 (形成闭环)。
    int nextSector = (baseSector + 1) % 8; // 使用模运算实现循环
    
    // 计算在当前扇区内的插值比例 (0.0 - 1.0)。
    // blend = 0 时，完全使用 baseSector 的角度；blend = 1 时，完全使用 nextSector 的角度。
    float blend = (angle - (baseSector * 45.0f)) / 45.0f;
    if (blend < 0.0f) blend = 0.0f; // 确保比例不小于0
    if (blend > 1.0f) blend = 1.0f; // 确保比例不大于1

    ServoAngles result; // 用于存储插值计算结果的舵机角度结构体
    // 对每个舵机，根据插值比例在两个基准方向之间计算目标角度。
    // result.servoX = base_angleX * (1 - blend) + next_angleX * blend
    result.servo1 = round(BASE_DIRECTIONS[baseSector].servo1 * (1.0f - blend) + BASE_DIRECTIONS[nextSector].servo1 * blend);
    result.servo2 = round(BASE_DIRECTIONS[baseSector].servo2 * (1.0f - blend) + BASE_DIRECTIONS[nextSector].servo2 * blend);
    result.servo3 = round(BASE_DIRECTIONS[baseSector].servo3 * (1.0f - blend) + BASE_DIRECTIONS[nextSector].servo3 * blend);
    return result; // 返回计算得到的舵机角度组合
}

// 控制三个舵机移动到指定的目标角度 (s1, s2, s3)。
// 此函数会进行角度约束，并实际驱动舵机旋转。
void moveServos(float s1, float s2, float s3) {
    // 将目标角度约束在各自舵机的最小和最大角度范围内
    currentServo1Pos = constrain(s1, (float)MIN_ANGLE_1, (float)MAX_ANGLE_1);
    currentServo2Pos = constrain(s2, (float)MIN_ANGLE_2, (float)MAX_ANGLE_2);
    currentServo3Pos = constrain(s3, (float)MIN_ANGLE_3, (float)MAX_ANGLE_3);
    
    // 将约束后的浮点数角度四舍五入为整数，并发送给舵机库进行驱动
    servo1.write(round(currentServo1Pos));
    servo2.write(round(currentServo2Pos));
    servo3.write(round(currentServo3Pos));
    
    // 舵机调试信息输出，每隔200毫秒打印一次当前舵机的目标浮点角度和实际写入的整数角度
    static unsigned long lastServoDebugTime = 0;
    if (millis() - lastServoDebugTime > 200) { // 控制打印频率
        Serial.print("舵机目标(float): ");
        Serial.print(currentServo1Pos, 2); Serial.print(", "); // 打印浮点角度，保留2位小数
        Serial.print(currentServo2Pos, 2); Serial.print(", ");
        Serial.print(currentServo3Pos, 2); Serial.print(" -> Int: ");
        Serial.print(round(currentServo1Pos)); Serial.print(", "); // 打印四舍五入后的整数角度
        Serial.print(round(currentServo2Pos)); Serial.print(", ");
        Serial.println(round(currentServo3Pos));
        lastServoDebugTime = millis(); // 更新上次打印时间
    }
}

// 将所有舵机移动到预设的中心位置 (SERVOX_CENTER 定义的值)。
void moveToCenterPosition() {
    // 调用 moveServos 函数，传入预设的中心角度值
    moveServos((float)SERVO1_CENTER, (float)SERVO2_CENTER, (float)SERVO3_CENTER);
}

// 增加所有舵机的当前角度，增加量为 ANGLE_STEP。
// 用于向上按钮的连续控制。
void servoAnglesIncrease() {
    // 在当前舵机角度的基础上增加 ANGLE_STEP，然后调用 moveServos 更新舵机
    moveServos(currentServo1Pos + ANGLE_STEP, 
               currentServo2Pos + ANGLE_STEP, 
               currentServo3Pos + ANGLE_STEP);
}

// 减少所有舵机的当前角度，减少量为 ANGLE_STEP。
// 用于向下按钮的连续控制。
void servoAnglesDecrease() {
    // 在当前舵机角度的基础上减少 ANGLE_STEP，然后调用 moveServos 更新舵机
    moveServos(currentServo1Pos - ANGLE_STEP, 
               currentServo2Pos - ANGLE_STEP, 
               currentServo3Pos - ANGLE_STEP);
}

// 将所有舵机移动到各自的最小角度 (MIN_ANGLE_X 定义的值)。
// 注意：此函数当前未被 MOS_CONTROL_BUTTON_PIN 调用，原设计可能是用于复位按钮。
void resetToMinPosition() { 
    // 调用 moveServos 函数，传入预设的最小角度值
    moveServos((float)MIN_ANGLE_1, (float)MIN_ANGLE_2, (float)MIN_ANGLE_3);
    Serial.println("按钮: 已重置到最小角度."); // 串口打印信息
}

// 读取遥感X、Y轴的模拟值，计算遥感方向和强度，并据此以速率控制方式更新舵机角度。
void mapJoystickToServos() {
    joystickX = analogRead(JOYSTICK_X_PIN); // 读取遥感X轴的模拟值 (0-1023)
    joystickY = analogRead(JOYSTICK_Y_PIN); // 读取遥感Y轴的模拟值 (0-1023)

    // 将X, Y轴的读数从 (0-1023) 映射到 (-100 到 100) 的范围
    float x_mapped = map(joystickX, 0, 1023, -100, 100);
    float y_mapped = map(joystickY, 0, 1023, -100, 100);

    // 使用atan2计算遥感的角度 (弧度制)，atan2能正确处理所有象限
    float angle_rad = atan2(y_mapped, x_mapped);
    // 将弧度转换为角度 (0-360度范围)
    float angle_deg = angle_rad * 180.0f / PI;
    if (angle_deg < 0) angle_deg += 360.0f; //确保角度在0-360之间

    // 计算遥感的强度 (摇杆偏离中心的距离，0-100*sqrt(2)范围，近似0-141)
    float strength = sqrt(x_mapped*x_mapped + y_mapped*y_mapped);

    // 如果强度小于死区阈值 (strength在0-141范围，DEADZONE是0-1比例，所以用 strength/100.0f 比较)
    // 则认为遥感在中心区域，不进行舵机更新，直接返回。
    if (strength / 100.0f < DEADZONE) {
        return; // 在死区内，不操作
    }
    
    // 对强度进行归一化处理，使其在死区之外从0.0到1.0变化。
    // normalized_strength = (当前强度 - 死区对应强度) / (最大强度 - 死区对应强度)
    float normalized_strength = (strength - (DEADZONE * 100.0f)) / (100.0f - (DEADZONE * 100.0f)); // 注意: 此处分母用100.0f可能不完全精确，因为最大strength是约141。更精确的应该是 (100.0f * sqrt(2) - DEADZONE * 100.0f)。但对于比例控制影响不大。
    normalized_strength = constrain(normalized_strength, 0.0f, 1.0f); // 确保归一化强度在0.0到1.0之间

    // 根据计算得到的遥感角度，插值计算出目标方向的舵机基准角度
    ServoAngles targetDirectionPos = interpolateDirection(angle_deg);

    // 计算每个舵机需要移动的微小增量 (速率控制)
    // 增量 = (目标方向舵机角度 - 当前舵机角度) * 归一化强度 * 灵敏度
    float delta1 = (targetDirectionPos.servo1 - currentServo1Pos) * normalized_strength * JOYSTICK_SENSITIVITY;
    float delta2 = (targetDirectionPos.servo2 - currentServo2Pos) * normalized_strength * JOYSTICK_SENSITIVITY;
    float delta3 = (targetDirectionPos.servo3 - currentServo3Pos) * normalized_strength * JOYSTICK_SENSITIVITY;

    // 更新舵机位置：当前位置 + 计算得到的增量
    moveServos(currentServo1Pos + delta1, 
               currentServo2Pos + delta2, 
               currentServo3Pos + delta3);

    // 遥感调试信息输出，控制打印频率，此处仅用于占位，实际未打印具体信息
    static unsigned long lastJoyDebug = 0;
    if (millis() - lastJoyDebug > 250) { // 每250ms允许一次
        lastJoyDebug = millis();
        // 可以添加 Serial.print 语句来调试遥感值、角度、强度等
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