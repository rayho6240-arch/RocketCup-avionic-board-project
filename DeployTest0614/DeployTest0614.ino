/*
 * 地面機構連鎖驗證程式 (Arduino Nano) + 極致左右分流整線版
 * 觸發側 (5V同側)：馬達1(A0)、馬達2(A1)、伺服馬達(A2)、實體擊發按鈕(A3)
 * 視覺側 (另一側)：LED指示燈(D12)
 * 作動順序：LED亮 -> 馬達(1) -> 伺服馬達解鎖 -> 馬達(2) -> LED滅 & 系統鎖死
 */

#include <Servo.h>

// === 另一側腳位定義 (純視覺指示燈) ===
const int LED_PIN     = 12;  // 外接狀態指示燈 (在 D12 腳位)

// === 5V 同側腳位定義 (動力控制與擊發訊號) ===
const int MOTOR1_PIN  = A0;  // 馬達 1 控制（接 L298N IN1）
const int MOTOR2_PIN  = A1;  // 馬達 2 控制（接 L298N IN3）
const int SERVO_PIN   = 9;  // 伺服馬達 PWM 訊號
const int TRIGGER_PIN = A3;  // 擊發訊號輸入（接按鈕或杜邦線，觸發接地）

Servo releaseServo;

// 系統狀態標記，確保每次開機只能擊發一次
bool hasFired = false; 

void setup() {
  Serial.begin(115200);
  Serial.println("--- 系統開機，三段式連鎖機構初始化 ---");

  // 設定輸入與上拉電阻 (A3 作為數位輸入)
  pinMode(TRIGGER_PIN, INPUT_PULLUP); 
  
  // 設定輸出腳位，並強制預設為低電位(斷電/熄滅)
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(MOTOR1_PIN, LOW); 
  digitalWrite(MOTOR2_PIN, LOW); 
  digitalWrite(LED_PIN, LOW); 

  Serial.println(">>> 系統準備就緒，等待擊發指令 (A3 -> GND) <<<");
}

void loop() {
  // 若已執行過，直接返回，鎖死系統
  if (hasFired) {
    return; 
  }

  // 偵測觸發訊號 (A3 被拉到 LOW)
  if (digitalRead(TRIGGER_PIN) == LOW) {
    delay(50); // 防抖動
    
    if (digitalRead(TRIGGER_PIN) == LOW) {
      Serial.println("!!! 收到擊發指令，啟動三段式序列 !!!");
      
      // === 點亮視覺指示燈 ===
      digitalWrite(LED_PIN, HIGH); 
      Serial.println("[警告] 系統作動中，外接指示燈已點亮！");
      
      // ==========================================
      // 第一階段：馬達 1 作動
      // ==========================================
      Serial.println("[階段 1] 馬達 1 啟動...");
      digitalWrite(MOTOR1_PIN, HIGH);
      delay(10000); 
      digitalWrite(MOTOR1_PIN, LOW);
      Serial.println("[階段 1] 馬達 1 停止並斷電。");
      delay(5000);  

      // ==========================================
      // 第二階段：伺服馬達解鎖
      // ==========================================

      Serial.println("[測試] 執行原始 PWM 脈衝測試...");
      pinMode(9, OUTPUT);
      analogWrite(9, 127); // 給 50% 的 PWM 訊號，馬達理論上應該會乖乖停在中間，不發出聲音
      delay(2000);
      // ==========================================
      // 第三階段：馬達 2 作動
      // ==========================================
      Serial.println("[階段 3] 馬達 2 啟動...");
      digitalWrite(MOTOR2_PIN, HIGH);
      delay(3000); 
      digitalWrite(MOTOR2_PIN, LOW);
      Serial.println("[階段 3] 馬達 2 停止並斷電。");

      // ==========================================
      // 終止階段：系統安全鎖死 & 關閉指示燈
      // ==========================================
      digitalWrite(LED_PIN, LOW); // 熄滅指示燈，代表危險期結束
      hasFired = true; 
      Serial.println("--- 測試序列完成，指示燈已熄滅，系統進入安全鎖死模式 ---");
    }
  }
}