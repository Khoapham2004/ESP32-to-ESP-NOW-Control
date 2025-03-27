#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Địa chỉ MAC của ESP32 nhận dữ liệu (Cập nhật đúng địa chỉ)
uint8_t receiverMAC[] = {0x78, 0x42, 0x1C, 0x6C, 0x01, 0xB4};

// Cấu trúc dữ liệu gửi đi (Thêm header và footer)
typedef struct {
  uint8_t header;  // Byte đầu (0xAA)
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;
  uint8_t pot;
  bool b1, b2, b3, b4, b5, b6, js1, js2;
  uint8_t footer;  // Byte cuối (0x55)
} DataPacket;

DataPacket data;

// Gọi lại khi gửi thành công hoặc thất bại
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);

  // Cấu hình WiFi ở chế độ Station
  WiFi.mode(WIFI_STA);

  // **Tăng công suất WiFi để giảm nhiễu**
  //esp_wifi_set_max_tx_power(78);  // Giá trị 78 tương đương ~20.5 dBm (mạnh nhất)
  WiFi.setSleep(false);  // Tắt tiết kiệm năng lượng WiFi
  esp_wifi_set_max_tx_power(84);  // Công suất phát tối đa
  setCpuFrequencyMhz(240);  // Chạy CPU ở tốc độ cao nhất
  // Khởi tạo ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  // Cấu hình peer (thiết bị nhận)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    return;
  }

  esp_now_set_wake_window(65535);  // Giữ ESP-NOW luôn hoạt động
  //esp_now_set_peer_rate(WIFI_PHY_RATE_54M);
  esp_now_register_send_cb(OnDataSent);

  // Cấu hình nút nhấn
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
}

void loop() {
  // Gán header và footer
  data.header = 0x4E;
  data.footer = 0x41;

  // Đọc giá trị joystick và biến trở
  data.ly = map(analogRead(35), 0, 4095, 0, 255);
  data.rx = map(analogRead(34), 0, 4095, 0, 255);
  data.ry = map(analogRead(33), 0, 4095, 0, 255);
  data.pot = map(analogRead(32), 0, 4095, 0, 255);

  // Đọc trạng thái các nút nhấn (0: Nhấn, 1: Không nhấn)
  data.b1 = !digitalRead(2);
  data.b2 = !digitalRead(4);
  data.b3 = !digitalRead(5);
  data.b4 = !digitalRead(12);
  data.b5 = !digitalRead(13);
  data.b6 = !digitalRead(14);
  data.js1 = !digitalRead(15);
  data.js2 = !digitalRead(18);

  // Gửi dữ liệu qua ESP-NOW
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));

  if (result != ESP_OK) {
    Serial.println("Gửi ESP-NOW thất bại!");
  }

  // In giá trị lên Serial
  Serial.print(" | Header: "); Serial.print(data.header, HEX);
  Serial.print(" | LY: "); Serial.print(data.ly);
  Serial.print(" | RX: "); Serial.print(data.rx);
  Serial.print(" | RY: "); Serial.print(data.ry);
  Serial.print(" | Pot: "); Serial.print(data.pot);
  Serial.print(" | B1: "); Serial.print(data.b1);
  Serial.print(" | B2: "); Serial.print(data.b2);
  Serial.print(" | B3: "); Serial.print(data.b3);
  Serial.print(" | B4: "); Serial.print(data.b4);
  Serial.print(" | B5: "); Serial.print(data.b5);
  Serial.print(" | B6: "); Serial.print(data.b6);
  Serial.print(" | JS1: "); Serial.print(data.js1);
  Serial.print(" | JS2: "); Serial.print(data.js2);
  Serial.print(" | Footer: "); Serial.println(data.footer, HEX);

  delay(100);  // Gửi dữ liệu mỗi 100ms
}
