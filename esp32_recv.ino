#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

//  Cấu hình chân
const int steeringPin = 15;  // Chân điều khiển servo
const int RPWM = 12;         // Điều hướng motor phải
const int LPWM = 13;         // Điều hướng motor trái
const int EN_PIN = 2;        // Kích hoạt motor (R_EN & L_EN)
const int turnSignalPin = 5;  // Chân đèn báo rẽ
bool turnSignalState = false;
unsigned long previousMillis = 0;
const int blinkInterval = 500; // Nhấp nháy 2 lần mỗi giây (1000ms / 2Hz = 500ms)


//  Cấu hình PWM
const int MotorPWMFreq = 1000;  // Tần số PWM cho motor
const int ServoPWMFreq = 50;    // Tần số PWM cho servo
const int PWMResolution = 8;    // 8-bit PWM (0-255)
//  Timeout nếu mất tín hiệu
#define SIGNAL_TIMEOUT 1000  
unsigned long lastRecvTime = 0;

//  Cấu trúc dữ liệu nhận (KHÔNG SỬA)
typedef struct {
 uint8_t header;  // Byte đầu (0xAA)
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;
  uint8_t pot;
  bool b1, b2, b3, b4, b5, b6, js1, js2;
  uint8_t footer;  // Byte cuối (0x55)
} DataPacket;

DataPacket receiverData;

// Xử lý dữ liệu nhận từ ESP-NOW
void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
  if (len == sizeof(DataPacket)) {  // Kiểm tra kích thước gói tin
    memcpy(&receiverData, incomingData, sizeof(receiverData));

    // Kiểm tra header và footer để đảm bảo dữ liệu hợp lệ
    if (receiverData.header == 0x4E && receiverData.footer == 0x41) {
      Serial.print("Received - LY: "); Serial.print(receiverData.ly);
      Serial.print(" RX: "); Serial.print(receiverData.rx);
      Serial.print(" Pot: "); Serial.println(receiverData.pot);

      int steering = map(receiverData.rx, 0, 254, 0, 180);
      setServoAngle(steering);
      controlMotor();
      lastRecvTime = millis();

    } else {
      Serial.println("⚠️ Dữ liệu không hợp lệ! (Sai header/footer)");
    }
  } else {
    Serial.println("⚠️ Gói tin không đúng kích thước!");
  }
}


void turnSignalLeft(bool state) {
 
}

void turnSignalRight(bool state) {
 
}


// Điều khiển motor BTS7960
void controlMotor() {
  int motorSpeed = map(receiverData.ly, 0, 254, -255, 255);
  int pwmSpeed = map(receiverData.pot, 0, 254, 0, 255);
  int steering = map(constrain(receiverData.rx + (127 - 112), 0, 254), 0, 254, -10, 180);

  Serial.print("Pot: "); Serial.print(receiverData.pot);
  Serial.print(" | Servo: "); Serial.print(steering);
  Serial.print(" | Motor Speed: "); Serial.print(motorSpeed);
  Serial.print(" | PWM: "); Serial.print(pwmSpeed);
  Serial.print(" | State: ");

  if (motorSpeed > 50) {
    Serial.println("Forward");
    goAhead(pwmSpeed);
  } else if (motorSpeed < -50) {
    Serial.println("Backward");
    goBack(pwmSpeed);
  } else {
    Serial.println("Stopped");
    stopMotor();
  }
}

//  Chạy tới
void goAhead(int MotorSpeed) {
  digitalWrite(EN_PIN, HIGH);    // Kích hoạt motor
  ledcWrite(RPWM, MotorSpeed);   // PWM cho tiến
  ledcWrite(LPWM, 0);            // Đặt lùi về 0
}

//  Chạy lùi
void goBack(int MotorSpeed) {
  digitalWrite(EN_PIN, HIGH);    // Kích hoạt motor
  ledcWrite(RPWM, 0);            // Đặt tiến về 0
  ledcWrite(LPWM, MotorSpeed);   // PWM cho lùi
}

//  Dừng motor
void stopMotor() {
  digitalWrite(EN_PIN, LOW);     // Tắt motor
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}
void brakeMoter(){
  digitalWrite(EN_PIN, LOW);
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}

// Điều khiển góc servo (0 - 180 độ)
void setServoAngle(int angle) {
  int duty = map(angle, 0, 180, 1638, 8192); // Map servo PWM (16-bit)
  ledcWrite(steeringPin, duty); // Ghi trực tiếp vào chân steeringPin
}
// 🛠 Thiết lập chân & PWM
void setUpPinModes() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  // Sử dụng ledcAttach thay cho ledcSetup và ledcAttachPin
  ledcAttach(RPWM, MotorPWMFreq, PWMResolution);  // PWM cho RPWM
  ledcAttach(LPWM, MotorPWMFreq, PWMResolution);  // PWM cho LPWM
  ledcAttach(steeringPin, ServoPWMFreq, 16);      // PWM cho Servo (16-bit)

  // Đưa servo về giữa khi khởi động
  setServoAngle(90);
}

void setup() {
  Serial.begin(115200);
  setUpPinModes();  
  
  WiFi.mode(WIFI_STA);
  //esp_wifi_set_max_tx_power(78);  // Giá trị 78 tương đương ~20.5 dBm
  WiFi.setSleep(false);  // Tắt tiết kiệm năng lượng WiFi
  esp_wifi_set_max_tx_power(74);  // Công suất phát WiFi tối đa
  setCpuFrequencyMhz(200);  // Chạy CPU ở tốc độ cao nhất
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
  esp_now_set_wake_window(65535);  // Giữ ESP-NOW luôn hoạt động
  //esp_now_set_peer_rate(WIFI_PHY_RATE_54M);  // Tốc độ truyền cao nhất
  esp_now_register_recv_cb(OnDataRecv);
  }

void loop() {
  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
    setServoAngle(90); // Trả về giữa
    stopMotor();
    Serial.println("No signal. Motor stopped, steering centered.");
  }
}
