#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define RPWM_CHANNEL 0
#define LPWM_CHANNEL 1
#define SERVO_CHANNEL 2

// ======== Pins ========
const int steeringPin = 12;
const int RPWM = 4;
const int LPWM = 5;
const int EN_PIN = 18;
const int LED_F = 27;
const int LED_TURN_R = 16;
const int LED_TURN_L = 17;

const int MotorPWMFreq = 1000;
const int ServoPWMFreq = 50;
const int PWMResolution = 8;

#define SIGNAL_TIMEOUT 1000
unsigned long lastRecvTime = 0;

// ======== Data Packet (đồng bộ với TX) ========
typedef struct {
  uint8_t header;
  uint8_t ly, rx, ry, pot;
  bool b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, js1, js2;
  uint8_t footer;
} DataPacket;

DataPacket receiverData;

// ======== Motor & Servo ========
void goAhead(int speed) {
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, speed);
}

void goBack(int speed) {
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM_CHANNEL, speed);
  ledcWrite(LPWM_CHANNEL, 0);
}

void stopMotor() {
  digitalWrite(EN_PIN, LOW);
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
}

void brakeMotor() {
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
}

void setServoAngle(int angle) {
  int duty = map(angle, 0, 180, 1638, 8192);
  ledcWrite(SERVO_CHANNEL, duty);

  int steering = map(receiverData.rx, 0, 254, 0, 206);
  if (steering > 100) {
    digitalWrite(LED_TURN_R, HIGH);
    digitalWrite(LED_TURN_L, LOW);
  } else if (steering < 60) {
    digitalWrite(LED_TURN_R, LOW);
    digitalWrite(LED_TURN_L, HIGH);
  } else {
    digitalWrite(LED_TURN_R, LOW);
    digitalWrite(LED_TURN_L, LOW);
  }
}

void controlMotor() {
  int motorSpeed = map(receiverData.ly, 0, 254, -255, 255);
  int pwmSpeed   = map(receiverData.pot, 0, 254, 0, 255);

  if (motorSpeed > 50)       goAhead(pwmSpeed);
  else if (motorSpeed < -50) goBack(pwmSpeed > 130 ? pwmSpeed - 30 : pwmSpeed);
  else                       stopMotor();
}

void setUpPinModes() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_F, OUTPUT);
  pinMode(LED_TURN_L, OUTPUT);
  pinMode(LED_TURN_R, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  digitalWrite(LED_F, LOW);

  ledcSetup(RPWM_CHANNEL, MotorPWMFreq, PWMResolution);
  ledcAttachPin(RPWM, RPWM_CHANNEL);

  ledcSetup(LPWM_CHANNEL, MotorPWMFreq, PWMResolution);
  ledcAttachPin(LPWM, LPWM_CHANNEL);

  ledcSetup(SERVO_CHANNEL, ServoPWMFreq, 16);
  ledcAttachPin(steeringPin, SERVO_CHANNEL);

  setServoAngle(90);
}

// ======== ESP-NOW Callback ========
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.print("Received! len=");
  Serial.print(len);
  Serial.print(" expected=");
  Serial.println(sizeof(DataPacket));

  if (len != sizeof(DataPacket)) return;
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  if (receiverData.header != 0x4E || receiverData.footer != 0x42) return;

  lastRecvTime = millis();
  Serial.println("Valid packet!");

  // Lái + ga
  int steering  = map(receiverData.rx, 0, 254, 0, 206);
  int ssteering = constrain(steering, 60, 130);
  setServoAngle(ssteering);
  controlMotor();

  // js1: tiến full + bật đèn pha
  if (receiverData.js1) {
    digitalWrite(LED_F, HIGH);
    goAhead(255);
  }

  // js2: phanh + trả lái giữa
  if (receiverData.js2) {
    brakeMotor();
    setServoAngle(90);
  }
}

// ======== Setup & Loop ========
void setup() {
  Serial.begin(115200);
  setUpPinModes();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(84);
  setCpuFrequencyMhz(240);

  // In MAC để kiểm tra
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_set_wake_window(65535);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Receiver ready, waiting for data...");
}

void loop() {
  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
    setServoAngle(90);
    stopMotor();
  }
}