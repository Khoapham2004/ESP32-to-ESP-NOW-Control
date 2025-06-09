#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

//  Cấu hình chân
const int steeringPin = 15;  // Chân điều khiển servo
const int RPWM = 12;         // Điều hướng motor phải
const int LPWM = 13;         // Điều hướng motor trái
const int EN_PIN = 2;        // Kích hoạt motor (R_EN & L_EN)
const int LED_F = 23;
const int LED_TURN_R = 16;
const int LED_TURN_L = 17;

bool isSensorMode = false;

// Danh sách chân TRIG và ECHO cho 5 cảm biến
const int trigPins[5] = {19, 4, 27, 5, 32};
const int echoPins[5] = {21, 26, 25, 18, 33};
long duration[5];
float distance[5];

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
        if (!isSensorMode) {
      Serial.print("Received - LY: "); Serial.print(receiverData.ly);
      Serial.print(" RX: "); Serial.print(receiverData.rx);
      Serial.print(" Pot: "); Serial.println(receiverData.pot);
      

      int steering = map(receiverData.rx, 0, 254, 0, 180);
      setServoAngle(steering);
      controlMotor();
      }
      lastRecvTime = millis();
       // Xử lý nút js1 và js2
    if (receiverData.js1) {
      digitalWrite(LED_F, HIGH);
      goAhead(255);
      Serial.println(" js1 pressed - Force Forward at Full Speed");
      delay(100);  // Chống rung nút
    }

    if (receiverData.js2) {
      brakeMoter();       // Phanh khẩn cấp
      setServoAngle(90);  // Trả servo về giữa
      Serial.println(" js2 pressed - Emergency Brake");
      delay(200);  // Chống rung nút
    }

    } else {
      Serial.println(" Dữ liệu không hợp lệ! (Sai header/footer)");
    }
  } else {
    Serial.println(" Gói tin không đúng kích thước!");
  }
}

void readUltrasonicSensors() {
  for (int i = 0; i < 5; i++) {
    // Gửi xung kích TRIG
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    // Đo thời gian phản hồi từ ECHO
    duration[i] = pulseIn(echoPins[i], HIGH, 30000); // timeout 30ms (5m)

    if (duration[i] == 0) {
      distance[i] = -1; // Không nhận tín hiệu
    } else {
      distance[i] = duration[i] * 0.0343 / 2;  // cm
    }
  }

  // In kết quả từng cảm biến
  for (int i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (distance[i] < 0) {
      Serial.println("Out of range");
    } else {
      Serial.print(distance[i]);
      Serial.println(" cm");
    }
  }

  Serial.println("------------------------");
}

// Điều khiển motor BTS7960
void controlMotor() {
  int motorSpeed = map(receiverData.ly, 0, 254, -255, 255);
  int pwmSpeed = map(receiverData.pot, 0, 254, 0, 255);
  int steering = map(receiverData.rx , 0, 254, 0, 206);
  steering = constrain(steering, 0, 180);

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
    goBack(180);
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
  digitalWrite(LED_F, LOW);
  digitalWrite(EN_PIN, LOW);     // Tắt motor
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}
void brakeMoter(){
  digitalWrite(EN_PIN, HIGH);
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}

// Điều khiển góc servo (0 - 180 độ)
void setServoAngle(int angle) {
  int steering = map(receiverData.rx , 0, 254, 0, 206);
  steering = constrain(steering, 0, 180);
  steering = angle;
  int duty = map(angle, 0, 180, 1638, 8192); // Map servo PWM (16-bit)
  ledcWrite(steeringPin, duty); // Ghi trực tiếp vào chân steeringPin

  // Điều khiển LED rẽ
  if (steering > 100) {
    digitalWrite(LED_TURN_R, HIGH);
    digitalWrite(LED_TURN_L, LOW);
  } else if (steering < 30) {
    digitalWrite(LED_TURN_R, LOW);
    digitalWrite(LED_TURN_L, HIGH);
  } else {
    digitalWrite(LED_TURN_R, LOW);
    digitalWrite(LED_TURN_L, LOW);
  }
}


// 🛠 Thiết lập chân & PWM
void setUpPinModes() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_F, OUTPUT);
  pinMode(LED_TURN_L, OUTPUT);
  pinMode(LED_TURN_R, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  digitalWrite(LED_F, LOW);

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
   
   // Cấu hình chân siêu âm
  for (int i = 0; i < 5; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

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
  
  // Kiểm tra timeout tín hiệu
  if (millis() - lastRecvTime > SIGNAL_TIMEOUT) {
    setServoAngle(90); // Trả về giữa
    stopMotor();
    Serial.println("No signal. Motor stopped, steering centered.");
    return;
  }
  if (receiverData.b4) {
  digitalWrite(LED_F, HIGH);
  }

  // Chuyển sang chế độ đọc cảm biến khi nhấn b6
  if (receiverData.b6 && !isSensorMode) {
    Serial.println(">>> CHUYỂN SANG CHẾ ĐỘ ĐỌC CẢM BIẾN <<<");
    isSensorMode = true;
    delay(100); // Chống rung nút
  }

  // Thoát chế độ cảm biến khi nhấn b5
  if (receiverData.b5 && isSensorMode) {
    Serial.println(">>> THOÁT CHẾ ĐỘ ĐỌC CẢM BIẾN <<<");
    isSensorMode = false;
    delay(100); // Chống rung nút
  }

  // Nếu đang ở chế độ đọc cảm biến thì chỉ đọc cảm biến
  if (isSensorMode) {
    readUltrasonicSensors();  // In giá trị cảm biến
    delay(50);               // Tránh in liên tục quá nhanh
    return;                   // Không xử lý các lệnh khác
  }

  // Nếu không ở chế độ đọc cảm biến thì xử lý bình thường
  // (Không cần else vì return ở trên đã ngắt luồng rồi)
}


