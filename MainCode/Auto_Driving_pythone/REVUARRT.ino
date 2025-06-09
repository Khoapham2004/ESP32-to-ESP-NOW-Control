#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// Cấu hình chân
const int steeringPin = 15;
const int RPWM = 12;
const int LPWM = 13;
const int EN_PIN = 2;
const int turnSignalPin = 5;
bool turnSignalState = false;
unsigned long previousMillis = 0;
const int blinkInterval = 500;
#define SIGNAL_TIMEOUT 1000
unsigned long lastRecvTime = 0;

// Cấu hình PWM
const int MotorPWMFreq = 1000;
const int ServoPWMFreq = 50;
const int PWMResolution = 8;

// Cấu trúc dữ liệu nhận
typedef struct {
  uint8_t header;  // 0x4E
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;
  uint8_t pot;
  bool b1, b2, b3, b4, b5, b6, js1, js2;
  uint8_t footer;  // 0x41
} DataPacket;

DataPacket receiverData;

// ==========================
// Hàm xử lý dữ liệu nhận được
// ==========================
void processReceivedData(const DataPacket &data) {
  int steering = map(data.rx, 0, 254, 0, 180);
  setServoAngle(steering);
  controlMotor();
  lastRecvTime = millis();

  Serial.print("Received - LY: "); Serial.print(data.ly);
  Serial.print(" RX: "); Serial.print(data.rx);
  Serial.print(" Pot: "); Serial.println(data.pot);
}

// ==========================
// Callback nhận dữ liệu từ ESP-NOW
// ==========================
void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
  if (len == sizeof(DataPacket)) {
    memcpy(&receiverData, incomingData, sizeof(receiverData));
    if (receiverData.header == 0x4E && receiverData.footer == 0x41) {
      processReceivedData(receiverData);
    } else {
      Serial.println("⚠️ Dữ liệu ESP-NOW không hợp lệ!");
    }
  } else {
    Serial.println("⚠️ Gói tin ESP-NOW không đúng kích thước!");
  }
}

  // ==========================
  // Đọc dữ liệu UART
  // ==========================
  void checkUART() {
    static byte buffer[sizeof(DataPacket)];
    static byte index = 0;

    while (Serial.available()) {
      byte incoming = Serial.read();

      if (index == 0 && incoming != 0x4E) continue; // Bỏ qua cho đến khi gặp header

      buffer[index++] = incoming;

      if (index == sizeof(DataPacket)) {
        memcpy(&receiverData, buffer, sizeof(DataPacket));

        if (receiverData.header == 0x4E && receiverData.footer == 0x41) {
          processReceivedData(receiverData);
        } else {
          Serial.println("⚠️ Dữ liệu UART không hợp lệ!");
        }

        index = 0; // Reset lại buffer
      }
    }
  }

// ==========================
// Motor điều khiển
// ==========================
void controlMotor() {
  int motorSpeed = map(receiverData.ly, 0, 254, -255, 255);
  int pwmSpeed = map(receiverData.pot, 254, 0, 0, 255);
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

void goAhead(int MotorSpeed) {
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM, MotorSpeed);
  ledcWrite(LPWM, 0);
}

void goBack(int MotorSpeed) {
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, MotorSpeed);
}

void stopMotor() {
  digitalWrite(EN_PIN, LOW);
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}

void setServoAngle(int angle) {
  int duty = map(angle, 0, 180, 1638, 8192);
  ledcWrite(steeringPin, duty);
}

void setUpPinModes() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  ledcAttach(RPWM, MotorPWMFreq, PWMResolution);
  ledcAttach(LPWM, MotorPWMFreq, PWMResolution);
  ledcAttach(steeringPin, ServoPWMFreq, 16);
  setServoAngle(90);
}

void setup() {
  Serial.begin(115200);
  setUpPinModes();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(70);
  setCpuFrequencyMhz(160);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_wake_window(65535);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  checkUART(); // Đọc dữ liệu UART

  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
    setServoAngle(90);
    stopMotor();
    Serial.println("No signal. Motor stopped, steering centered.");
  }
}
