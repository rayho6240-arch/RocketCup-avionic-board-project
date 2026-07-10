/*
 * 航電組 - STM32F103C8T6 3.3V 邏輯與二極體隔離驗證程式
 * 硬體配置：PB12(按鈕)、PC13(內建LED)、PA0(馬達1)、PA1(馬達2)、PA2(伺服馬達)
 */

#include <Servo.h>

// === STM32 專屬腳位定義 ===
const int TRIGGER_PIN = PB12; // 擊發按鈕 (接 PB12 與 GND)
const int LED_PIN     = PC13; // STM32 內建綠色/藍色 LED
const int MOTOR1_PIN  = PA0;  // 馬達 1 透過二極體接 L298N IN1
const int MOTOR2_PIN  = PA1;  // 馬達 2 透過二極體接 L298N IN3
const int SERVO_PIN   = PA2;  // 伺服馬達透過二極體接 PWM 訊號線

Servo releaseServo;

bool hasFired = false; 

void setup() {
  Serial.begin(115200);
  Serial.println("--- STM32 系統開機，硬體初始化 ---");

  // 設定輸入，開啟 STM32 內部上拉電阻
  pinMode(TRIGGER_PIN, INPUT_PULLUP); 
  
  // 設定輸出腳位
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // 初始狀態斷電
  digitalWrite(MOTOR1_PIN, LOW); 
  digitalWrite(MOTOR2_PIN, LOW); 
  
  // STM32 的 PC13 是「HIGH 為熄滅，LOW 為點亮」，所以開機先設為 HIGH
  digitalWrite(LED_PIN, HIGH); 

  Serial.println(">>> STM32 準備就緒，等待 PB12 觸發 <<<");
}

void loop() {
  if (hasFired) return; 

  if (digitalRead(TRIGGER_PIN) == LOW) {
    delay(50); // 防抖動
    
    if (digitalRead(TRIGGER_PIN) == LOW) {
      Serial.println("!!! 收到擊發指令，啟動序列 !!!");
      
      // 點亮 PC13 指示燈 (拉低電位)
      digitalWrite(LED_PIN, LOW); 
      
      // ==========================================
      // 階段 1：馬達 1 作動
      // ==========================================
      Serial.println("[階段 1] 馬達 1 啟動");
      digitalWrite(MOTOR1_PIN, HIGH);
      delay(3000); // 測試用，縮短為 3 秒
      digitalWrite(MOTOR1_PIN, LOW);
      delay(500);  

      // ==========================================
      // 階段 2：伺服馬達動態掛載與解鎖
      // ==========================================
      Serial.println("[階段 2] 伺服馬達轉向 90 度");
      releaseServo.write(90); 
      releaseServo.attach(SERVO_PIN); 
      delay(800);  
      releaseServo.detach(); 

      // ==========================================
      // 階段 3：馬達 2 作動
      // ==========================================
      Serial.println("[階段 3] 馬達 2 啟動");
      digitalWrite(MOTOR2_PIN, HIGH);
      delay(1500); 
      digitalWrite(MOTOR2_PIN, LOW);

      // ==========================================
      // 鎖死系統與關閉指示燈
      // ==========================================
      digitalWrite(LED_PIN, HIGH); // 熄滅 PC13 指示燈
      hasFired = true; 
      Serial.println("--- 測試完成，系統安全鎖死 ---");
    }
  }
}