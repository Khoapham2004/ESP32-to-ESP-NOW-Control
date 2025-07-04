#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

// PID
double input, output, setpoint = 0;
double Kp = 2.0, Ki = 0.5, Kd = 0.2;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Pin setup
const int steeringPin = 15;
const int RPWM = 12;
const int LPWM = 13;
const int EN_PIN = 2;
const int LED_F = 23;
const int LED_TURN_R = 16;
const int LED_TURN_L = 17;

// Trig/Echo
const int trigPins[5] = {19, 4, 27, 5, 32};
const int echoPins[5] = {21, 26, 25, 18, 33};
long duration[5];
float distance[5];

// PWM config
const int MotorPWMFreq = 1000;
const int ServoPWMFreq = 50;
const int PWMResolution = 8;

#define SIGNAL_TIMEOUT 1000
unsigned long lastRecvTime = 0;

bool isSensorMode = false;

// Hồi vị
int previousPIDAngle = 90;
bool needRestoreAngle = false;
bool isRestoring = false;
bool isWaitingForRestore = false;
unsigned long waitStartTime = 0;
unsigned long restoreStartTime = 0;
int restoreAngle = 90;

// Data packet
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
    if (receiverData.header == 0x4E && receiverData.footer == 0x41) {
      if (!isSensorMode) {
        int steering = map(receiverData.rx, 0, 254, 0, 206);
        int ssteering = constrain(steering, 50, 140);
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
    } else Serial.println("Sai header/footer");
  } else Serial.println("Sai kích thước gói tin!");
}

void readUltrasonicSensors() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);
    duration[i] = pulseIn(echoPins[i], HIGH, 30000);
    distance[i] = (duration[i] == 0) ? -1 : duration[i] * 0.0343 / 2;
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
}

void setup() {
  Serial.begin(115200);
  setUpPinModes();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-40, 40);
  for (int i = 0; i < 5; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
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
    Serial.println("Mất tín hiệu - Dừng");
    return;
  }

  if (receiverData.b4) digitalWrite(LED_F, HIGH);
  if (receiverData.b6 && !isSensorMode) {
    Serial.println(">> BẬT CHẾ ĐỘ CẢM BIẾN");
    isSensorMode = true;
    delay(100);
  }
  if (receiverData.b5 && isSensorMode) {
    Serial.println(">> TẮT CHẾ ĐỘ CẢM BIẾN");
    isSensorMode = false;
    delay(100);
  }

  // Nếu đang hồi vị → giữ góc trong 3s rồi về giữa
  if (isRestoring) {
    if (millis() - restoreStartTime < 3000) {
      setServoAngle(restoreAngle);
      return;
    } else {
      setServoAngle(90);
      isRestoring = false;
      previousPIDAngle = 90;
      Serial.println("✅ Hồi vị xong - Servo về giữa (90°)");
      return;
    }
  }

  // Nếu đang chờ 1s xác nhận đường thoáng
  if (isWaitingForRestore) {
    if (millis() - waitStartTime >= 1000) {
      restoreAngle = constrain(180 - previousPIDAngle, 50, 140);
      restoreStartTime = millis();
      isRestoring = true;
      isWaitingForRestore = false;
      Serial.print("↩ Bắt đầu hồi vị sau 1s: "); Serial.println(restoreAngle);
    }
    return;
  }

  // Chế độ cảm biến
  if (isSensorMode) {
    readUltrasonicSensors();

    float leftSum = 0; int leftCount = 0;
    if (distance[2] > 0) { leftSum += distance[2]; leftCount++; }
    if (distance[3] > 0) { leftSum += distance[3]; leftCount++; }
    float leftAvg = (leftCount > 0) ? leftSum / leftCount : -1;

    float rightSum = 0; int rightCount = 0;
    if (distance[0] > 0) { rightSum += distance[0]; rightCount++; }
    if (distance[1] > 0) { rightSum += distance[1]; rightCount++; }
    float rightAvg = (rightCount > 0) ? rightSum / rightCount : -1;

    if (leftAvg < 0 || rightAvg < 0) {
      Serial.println("⚠ Thiếu dữ liệu cảm biến");
      return;
    }

    // Nếu tất cả cảm biến đều xa hơn 30cm → chờ 1s rồi hồi vị
    if (distance[0] > 40 && distance[1] > 40 && distance[2] > 40 && distance[3] > 40) {
      if (needRestoreAngle && previousPIDAngle != 90 && !isWaitingForRestore) {
        waitStartTime = millis();
        isWaitingForRestore = true;
        Serial.println("⏳ Đường thoáng - chờ 1s trước khi hồi vị...");
      } else if (!needRestoreAngle) {
        setServoAngle(90);
        Serial.println("✅ Servo giữ giữa (90°)");
      }
      return;
    }

    // PID điều khiển
    input = rightAvg - leftAvg;
    myPID.Compute();
    int angle = constrain(90 + output, 50, 140);
    setServoAngle(angle);
    previousPIDAngle = angle;
    needRestoreAngle = true;

    Serial.print("PID: "); Serial.print(input);
    Serial.print(" | Out: "); Serial.print(output);
    Serial.print(" | Servo: "); Serial.println(angle);

    delay(50);
    return;
  }
}
