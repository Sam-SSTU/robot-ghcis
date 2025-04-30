// 定义引脚
const int X_PIN = A0;  // X轴接A0
const int Y_PIN = A1;  // Y轴接A1
const int BTN_4 = 4;   // 按钮4
const int BTN_3 = 3;   // 按钮3
const int BTN_2 = 2;   // 按钮2

void setup() {
  // 初始化串口通信
  Serial.begin(9600);
  
  // 设置按钮引脚为输入模式
  pinMode(BTN_4, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
}

void loop() {
  // 读取遥感值
  int xValue = analogRead(X_PIN);
  int yValue = analogRead(Y_PIN);
  
  // 读取按钮状态
  int btn4State = digitalRead(BTN_4);
  int btn3State = digitalRead(BTN_3);
  int btn2State = digitalRead(BTN_2);
  
  // 判断遥感方向
  String direction = "";
  
  // 遥感中间值大约在512左右
  // 设定一个阈值，用于判断是否移动
  const int threshold = 100;
  const int center = 512;
  
  // 判断八个方向
  if (abs(xValue - center) < threshold && abs(yValue - center) < threshold) {
    direction = "中间";
  }
  else {
    // 判断上下
    if (yValue < (center - threshold)) {
      if (xValue < (center - threshold)) {
        direction = "左上";
      }
      else if (xValue > (center + threshold)) {
        direction = "右上";
      }
      else {
        direction = "上";
      }
    }
    else if (yValue > (center + threshold)) {
      if (xValue < (center - threshold)) {
        direction = "左下";
      }
      else if (xValue > (center + threshold)) {
        direction = "右下";
      }
      else {
        direction = "下";
      }
    }
    else {
      if (xValue < (center - threshold)) {
        direction = "左";
      }
      else if (xValue > (center + threshold)) {
        direction = "右";
      }
    }
  }
  
  // 打印结果
  Serial.print("方向: ");
  Serial.print(direction);
  Serial.print(" X值: ");
  Serial.print(xValue);
  Serial.print(" Y值: ");
  Serial.print(yValue);
  Serial.print(" 按钮4: ");
  Serial.print(btn4State);
  Serial.print(" 按钮3: ");
  Serial.print(btn3State);
  Serial.print(" 按钮2: ");
  Serial.println(btn2State);
  
  delay(100);  // 短暂延时，避免输出太快
}