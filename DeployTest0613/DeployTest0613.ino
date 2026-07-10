#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ================= 引腳定義 =================
#define PIN_3S_MOTOR      25  // 輸出1: 3S直流馬達
#define PIN_3S_IGNITER    26  // 輸出2: 3S點火頭
#define PIN_SERVO_MOTOR   14  // 輸出3: 5V PWM伺服馬達

// ================= Wi-Fi 熱點設定 =================
const char *ssid = "Rocket_State_Avionics";
const char *password = "12345678";
WebServer server(80);
Servo myServo;

// ================= 底層狀態旗標 (System States) =================
// 使用獨立布林值，允許 3S馬達、3S點火、PWM馬達三種狀態「同時存在」
bool lock_3sMotor  = false;  bool trig_3sMotor  = false;
bool lock_3sFire   = false;  bool trig_3sFire   = false;
bool lock_servo    = false;  bool trig_servo    = false;

// 記錄動作開始的時間（用於非阻塞式定時器）
unsigned long motorStartTime = 0; 
unsigned long fireStartTime  = 0;
const unsigned long MOTOR_TIMEOUT = 10000; // 3S馬達最長給電 10 秒
const unsigned long FIRE_TIMEOUT  = 2000;  // 3S點火頭給電 2 秒

// ================= HTML + CSS + JS 網頁端 =================
// 透過 AJAX 動態更新按鈕顏色（綠色->紅色）與後端同步
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>航電狀態機控制台</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; background-color: #1e1e24; color: #fff; padding: 10px; }
        h1 { color: #f4d03f; font-size: 24px; }
        .control-panel { display: flex; flex-direction: column; align-items: center; gap: 15px; margin-top: 20px; }
        .row { background: #2a2a35; padding: 15px; border-radius: 10px; width: 90%; max-width: 400px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
        h3 { margin-top: 0; color: #e1e1e1; border-bottom: 1px solid #444; padding-bottom: 5px; }
        .btn-group { display: flex; justify-content: space-around; gap: 10px; margin-top: 10px; }
        .btn { flex: 1; padding: 12px 5px; font-size: 14px; font-weight: bold; color: white; border: none; border-radius: 6px; cursor: pointer; transition: 0.2s; }
        /* 狀態顏色：預設綠色 (false)，觸發後紅色 (true) */
        .state-false { background-color: #2ec4b6; box-shadow: 0 3px #1da196; }
        .state-true { background-color: #e71d36; box-shadow: 0 3px #b21224; transform: translateY(2px); }
        .btn:active { transform: translateY(3px); box-shadow: none; }
    </style>
</head>
<body>
    <h1>🚀 航電雙重鎖控制台 (並列狀態機)</h1>
    <div class="control-panel">
        
        <div class="row">
            <h3>【 3S 直流馬達 】</h3>
            <div class="btn-group">
                <button id="lock_3sMotor" class="btn state-false" onclick="toggle('lock_3sMotor')">安全鎖</button>
                <button id="trig_3sMotor" class="btn state-false" onclick="toggle('trig_3sMotor')">啟動開關</button>
            </div>
        </div>

        <div class="row">
            <h3>【 3S 點火頭 】</h3>
            <div class="btn-group">
                <button id="lock_3sFire" class="btn state-false" onclick="toggle('lock_3sFire')">安全鎖</button>
                <button id="trig_3sFire" class="btn state-false" onclick="toggle('trig_3sFire')">啟動開關</button>
            </div>
        </div>

        <div class="row">
            <h3>【 5V PWM 馬達 (180°) 】</h3>
            <div class="btn-group">
                <button id="lock_servo" class="btn state-false" onclick="toggle('lock_servo')">安全鎖</button>
                <button id="trig_servo" class="btn state-false" onclick="toggle('trig_servo')">180° / 0°</button>
            </div>
        </div>

    </div>

    <script>
        // 發送指令並動態改變按鈕顏色
        function toggle(target) {
            fetch('/toggle?target=' + target)
            .then(response => response.text())
            .then(state => {
                let btn = document.getElementById(target);
                if(state === "1") {
                    btn.className = "btn state-true";
                } else {
                    btn.className = "btn state-false";
                }
            });
        }
        
        // 定時向後端同步狀態（防止網頁刷新後顏色跑掉）
        void function syncState() {
            fetch('/status').then(r => r.json()).then(data => {
                for (let key in data) {
                    let btn = document.getElementById(key);
                    if(btn) btn.className = data[key] ? "btn state-true" : "btn state-false";
                }
            });
            setTimeout(syncState, 1000);
        }();
    </script>
</body>
</html>
)rawliteral";

// ================= WebServer 路由處理 =================
void handleRoot() {
  server.send_P(200, "text/html", HTML_PAGE);
}

// 處理按鈕點擊，切換狀態
void handleToggle() {
  if (!server.hasArg("target")) { server.send(400, "text/plain", "Error"); return; }
  String target = server.arg("target");
  bool currentState = false;

  if (target == "lock_3sMotor") { lock_3sMotor = !lock_3sMotor; currentState = lock_3sMotor; }
  else if (target == "trig_3sMotor") { 
    trig_3sMotor = !trig_3sMotor; currentState = trig_3sMotor;
    if (lock_3sMotor && trig_3sMotor) motorStartTime = millis(); // 進入狀態，記錄時間
  }
  else if (target == "lock_3sFire")  { lock_3sFire = !lock_3sFire;   currentState = lock_3sFire; }
  else if (target == "trig_3sFire")  { 
    trig_3sFire = !trig_3sFire;   currentState = trig_3sFire;
    if (lock_3sFire && trig_3sFire) fireStartTime = millis();    // 進入狀態，記錄時間
  }
  else if (target == "lock_servo")   { lock_servo = !lock_servo;     currentState = lock_servo; }
  else if (target == "trig_servo")   { trig_servo = !trig_servo;     currentState = trig_servo; }

  server.send(200, "text/plain", currentState ? "1" : "0");
}

// 回傳目前所有鎖的 JSON 狀態給網頁更新顏色
void handleStatus() {
  String json = "{";
  json += "\"lock_3sMotor\":" + String(lock_3sMotor) + ",";
  json += "\"trig_3sMotor\":" + String(trig_3sMotor) + ",";
  json += "\"lock_3sFire\":" + String(lock_3sFire) + ",";
  json += "\"trig_3sFire\":" + String(trig_3sFire) + ",";
  json += "\"lock_servo\":" + String(lock_servo) + ",";
  json += "\"trig_servo\":" + String(trig_servo) + "}";
  server.send(200, "application/json", json);
}

// ================= 主程式 Setup =================
void setup() {
  Serial.begin(115200);

  // 1. 強制拉低所有數位輸出腳位（安全狀態）
  pinMode(PIN_3S_MOTOR, OUTPUT);
  pinMode(PIN_3S_IGNITER, OUTPUT);
  digitalWrite(PIN_3S_MOTOR, LOW);
  digitalWrite(PIN_3S_IGNITER, LOW);

  // 2. 初始化伺服馬達 PWM
  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(PIN_SERVO_MOTOR, 500, 2400);
  myServo.write(0); // 初始安全位置歸零

  // 3. 啟動 Wi-Fi 熱點
  WiFi.softAP(ssid, password);
  Serial.print("熱點已啟動。IP 位址: ");
  Serial.println(WiFi.softAPIP());

  // 4. 配置伺服器路由
  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);
  server.on("/status", handleStatus);
  server.begin();
}

// ================= 主程式 Loop (底層狀態機) =================
void loop() {
  server.handleClient(); // 監聽網頁通訊

  // -------------------------------------------------------------
  // 底層狀態機：分開判定各狀態，非 if-else 互斥，可同時成立
  // -------------------------------------------------------------

  // 狀態一：3S 直流馬達控制狀態
  if (lock_3sMotor && trig_3sMotor) {
    // 雙重解鎖成功 -> 輸出 HIGH
    digitalWrite(PIN_3S_MOTOR, HIGH);
    
    // 安全機制：超時（馬達跑到底自斷或滿 10 秒）自動重設狀態
    if (millis() - motorStartTime >= MOTOR_TIMEOUT) {
      digitalWrite(PIN_3S_MOTOR, LOW);
      trig_3sMotor = false; // 自動關閉啟動開關（網頁端會變回綠色）
      Serial.println("狀態機提示：3S 馬達超時，強制拉低");
    }
  } else {
    // 任意一個條件沒滿足 -> 安全狀態（LOW）
    digitalWrite(PIN_3S_MOTOR, LOW);
  }

  // 狀態二：3S 點火頭控制狀態
  if (lock_3sFire && trig_3sFire) {
    // 雙重解鎖成功 -> 輸出 HIGH
    digitalWrite(PIN_3S_IGNITER, HIGH);
    
    // 安全機制：給電滿 2 秒隨後關閉（點火頭通常已燒斷）
    if (millis() - fireStartTime >= FIRE_TIMEOUT) {
      digitalWrite(PIN_3S_IGNITER, LOW);
      trig_3sFire = false; // 自動關閉啟動開關（網頁端會變回綠色）
      Serial.println("狀態機提示：3S 點火完成，強制拉低");
    }
  } else {
    // 任意一個條件沒滿足 -> 安全狀態（LOW）
    digitalWrite(PIN_3S_IGNITER, LOW);
  }

  // 狀態三：5V PWM 馬達控制狀態
  if (lock_servo) {
    // 只有在安全鎖解除時，啟動開關（trig_servo）才能決定角度
    if (trig_servo) {
      myServo.write(180); // 啟動開關按下 (true) -> 正轉 180 度
    } else {
      myServo.write(0);   // 啟動開關放開 (false) -> 反轉回到 0 度
    }
  } else {
    // 安全鎖未解除 -> 強制維持在 0 度安全位置，不理會啟動開關
    myServo.write(0);
    trig_servo = false; // 強制將啟動狀態歸零
  }
}