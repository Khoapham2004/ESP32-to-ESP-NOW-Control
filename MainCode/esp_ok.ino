#include <Wire.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <math.h>

// ======== MPU6050 ========
MPU6050 mpu;
float angleZ = 0;
float gyroZ_offset = 0;
unsigned long lastMPUTime;

#define FILTER_SIZE 10
float rateZ_buffer[FILTER_SIZE];
int filter_index = 0;

// ======== PID ========
double input, output, setpoint = 0;
double Kp = 5.0, Ki = 0.5, Kd = 0.5;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ======== Yaw PID ========
double yawInput, yawOutput, yawSetpoint = 0;
double yawKp = 3.0, yawKi = 0.2, yawKd = 0.8;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);

// ======== Pins ========
const int steeringPin = 15;
const int RPWM = 12;
const int LPWM = 13;
const int EN_PIN = 2;
const int LED_F = 23;
const int LED_TURN_R = 16;
const int LED_TURN_L = 17;
int yawDirection = -1; // Đổi -1 hoặc 1 nếu bị ngược

// Chỉ dùng 2 cảm biến: Trái (LEFT) và Phải (RIGHT)
const int trigPins[2] = {4, 27};    // LEFT, RIGHT
const int echoPins[2] = {26, 25};  // LEFT, RIGHT
long duration[2];
float distance[2];

const int MotorPWMFreq = 1000;
const int ServoPWMFreq = 50;
const int PWMResolution = 8;

#define SIGNAL_TIMEOUT 1000
unsigned long lastRecvTime = 0;
bool isSensorMode = false;

// ======== Restore Tracking ========
bool isPathClear = false;
unsigned long sensorClearStart = 0;

// ======== Data Packet ========
typedef struct {
  uint8_t header;
  uint8_t ly, rx, ry, pot;
  bool b1, b2, b3, b4, b5, b6, js1, js2;
  uint8_t footer;
} DataPacket;

DataPacket receiverData;

void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
  if (len == sizeof(DataPacket)) {
    memcpy(&receiverData, incomingData, sizeof(receiverData));
    if (receiverData.header == 0x4A && receiverData.footer == 0x42) {
      if (!isSensorMode) {
        int steering = map(receiverData.rx, 0, 254, 0, 206);
        int ssteering = constrain(steering, 60, 130);
        setServoAngle(ssteering);
        controlMotor();
      }
      lastRecvTime = millis();

      if (receiverData.js1) {
        digitalWrite(LED_F, HIGH);
        goAhead(255);
        delay(100);
      }
      if (receiverData.js2) {
        brakeMoter();
        setServoAngle(90);
        delay(200);
      }

      if (receiverData.b6 && !isSensorMode) {
        isSensorMode = true;
        angleZ = 0;
        Serial.println("Chế độ cảm biến BẬT - Cập nhật angleZ = 0");
        delay(100);
      }

      if (receiverData.b5 && isSensorMode) {
        isSensorMode = false;
        Serial.println("Chế độ cảm biến TẮT");
        delay(100);
      }

      if (receiverData.b3) {
        angleZ = 0;
      }
    }
  }
}

void setupMPU6050() {
  Wire.begin(33, 32);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 không kết nối");
    while (1);
  }
  delay(1000);
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroZ_offset = sum / 1000.0;
  lastMPUTime = millis();
  for (int i = 0; i < FILTER_SIZE; i++) rateZ_buffer[i] = 0;
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-40, 40);
}

void updateYaw() {
  if (!isSensorMode) return;
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  unsigned long now = millis();
  float dt = (now - lastMPUTime) / 1000.0;
  lastMPUTime = now;
  if (dt <= 0 || dt > 1.0) return;

  float rawRateZ = -(gz - gyroZ_offset) / 131.0;
  rateZ_buffer[filter_index] = rawRateZ;
  filter_index = (filter_index + 1) % FILTER_SIZE;

  float filteredRateZ = 0;
  for (int i = 0; i < FILTER_SIZE; i++) filteredRateZ += rateZ_buffer[i];
  filteredRateZ /= FILTER_SIZE;

  angleZ += filteredRateZ * dt;
  angleZ = fmod(angleZ, 360.0);
  if (angleZ < 0) angleZ += 360;
}

void adjustSteeringByYaw() {
  yawInput = angleZ;
  if (yawInput > 180) yawInput -= 360;
  if (yawInput < -180) yawInput += 360;
  yawPID.Compute();
}

void readUltrasonicSensors() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    long duration = pulseIn(echoPins[i], HIGH, 30000);
    if (duration == 0) {
      distance[i] = -1; // Không đo được
    } else {
      distance[i] = duration * 0.0343 / 2; // cm
    }
  }
}

void goAhead(int speed) {
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM, speed);
  ledcWrite(LPWM, 0);
}

void goBack(int speed) {
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, speed);
}

void stopMotor() {
  digitalWrite(LED_F, LOW);
  digitalWrite(EN_PIN, LOW);
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}

void brakeMoter() {
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}

void setServoAngle(int angle) {
  int duty = map(angle, 0, 180, 1638, 8192);
  ledcWrite(steeringPin, duty);

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
  int pwmSpeed = map(receiverData.pot, 0, 254, 0, 255);
  int steering = map(receiverData.rx , 0, 254, 0, 206);
  int ssteering = constrain(steering, 50, 140);
  if (motorSpeed > 50) goAhead(pwmSpeed);
  else if (motorSpeed < -50) goBack(pwmSpeed > 130 ? pwmSpeed - 30 : pwmSpeed);
  else stopMotor();
}


void setUpPinModes() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_F, OUTPUT);
  pinMode(LED_TURN_L, OUTPUT);
  pinMode(LED_TURN_R, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  digitalWrite(LED_F, LOW);
  ledcAttach(RPWM, MotorPWMFreq, PWMResolution);
  ledcAttach(LPWM, MotorPWMFreq, PWMResolution);
  ledcAttach(steeringPin, ServoPWMFreq, 16);
  setServoAngle(90);
  for (int i = 0; i < 2; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void setup() {
  Serial.begin(115200);
  setUpPinModes();
  setupMPU6050();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-40, 40);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(74);
  setCpuFrequencyMhz(200);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_set_wake_window(65535);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
    setServoAngle(90);
    stopMotor();
    return;
  }

  if (!isSensorMode) return;

  updateYaw();
  readUltrasonicSensors();
  int pwmSpeed = map(receiverData.pot, 0, 254, 0, 255);

  // ===== Kiểm tra vật cản =====
  bool objectDetected = false;
  for (int i = 0; i < 2; i++) {
    if (distance[i] > 0 && distance[i] <= 37) {
      objectDetected = true;
      break;
    }
  }

  if (!objectDetected) {
    // ======= Không có vật cản - chỉ PID MPU =======
    if (!isPathClear) {
      sensorClearStart = millis();
      isPathClear = true;
      setServoAngle(90);
    } else if (millis() - sensorClearStart >= 700) {
      adjustSteeringByYaw();
      int correctedAngle = constrain(90 + yawDirection * yawOutput, 50, 140);
      setServoAngle(correctedAngle);
      goAhead(pwmSpeed);
    }
  } else {
    // ======= Có vật cản - PID Siêu âm =======
    isPathClear = false;

    float leftDist = distance[0];
    float rightDist = distance[1];

    if (leftDist < 0 || rightDist < 0) return;

    input = rightDist - leftDist; // So sánh phải - trái
    myPID.Compute();

    int ultrasonicAngle = constrain(90 + output, 50, 140);
    setServoAngle(ultrasonicAngle);
    goAhead(pwmSpeed);
    delay(50);
  }
}
