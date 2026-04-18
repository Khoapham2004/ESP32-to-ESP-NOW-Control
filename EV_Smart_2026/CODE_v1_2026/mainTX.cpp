#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// OLED 1.3 inch SH1106 (128x64), I2C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === Danh sách địa chỉ MAC ===
const uint8_t macList[][6] = {
  {0x94, 0x51, 0xDC, 0x3A, 0xC8, 0xE8},
  {0x94, 0x51, 0xDC, 0x3C, 0x47, 0x8C},
};
const int macCount = sizeof(macList) / sizeof(macList[0]);

// MAC đang dùng
uint8_t receiverMAC[6];
int selectedMAC = 0;

// Trạng thái menu MAC
bool inMacMenu = false;
int menuCursor = 0;

// Trạng thái login
bool inLoginMode = false;
int loginDigits[4] = {-1, -1, -1, -1};
int loginIndex = 0;
const int passwordCode[4] = {2, 0, 0, 4};

// Trạng thái kết nối
bool espNowConnected = false;
unsigned long lastSendTime = 0;

// Debounce
unsigned long lastB1Press = 0;
unsigned long lastB2Press = 0;
unsigned long lastB3Press = 0;
unsigned long lastB4Press = 0;
unsigned long lastB5Press = 0;
unsigned long lastB6Press = 0;
unsigned long lastB7Press = 0;
unsigned long lastB8Press = 0;
unsigned long lastB9Press = 0;
unsigned long lastB10Press = 0;
unsigned long lastB11Press = 0;
unsigned long lastB12Press = 0;
unsigned long lastJs1Press = 0;
unsigned long lastJs2Press = 0;
const unsigned long debounceDelay = 200;

// Cấu trúc dữ liệu gửi đi
typedef struct {
  uint8_t header;
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;
  uint8_t pot;
  bool b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, js1, js2;
  uint8_t footer;
} DataPacket;

DataPacket data;

// === Hàm thiết lập peer ESP-NOW ===
void setupPeer() {
  esp_now_del_peer(receiverMAC);

  memcpy(receiverMAC, macList[selectedMAC], 6);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 6;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
  } else {
    Serial.print("Peer set to: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", receiverMAC[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  espNowConnected = (status == ESP_NOW_SEND_SUCCESS);
}

// === Login helpers ===
void resetLoginInput() {
  for (int i = 0; i < 4; i++) {
    loginDigits[i] = -1;
  }
  loginIndex = 0;
}

bool isPasswordCorrect() {
  for (int i = 0; i < 4; i++) {
    if (loginDigits[i] != passwordCode[i]) return false;
  }
  return true;
}

void inputLoginDigit(int digit) {
  if (loginIndex < 4) {
    loginDigits[loginIndex] = digit;
    loginIndex++;
  }
}

void deleteLastLoginDigit() {
  if (loginIndex > 0) {
    loginIndex--;
    loginDigits[loginIndex] = -1;
  }
}

// === Hiển thị login ===
void displayLoginScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(24, 0);
  display.println("LOGIN NUMBER");
  display.drawLine(0, 10, 127, 10, SH110X_WHITE);

  display.setCursor(0, 18);
  display.println("Nhap ma 4 so:");

  display.setTextSize(2);
  display.setCursor(14, 34);
  for (int i = 0; i < 4; i++) {
    if (loginDigits[i] == -1) {
      display.print("_");
    } else {
      display.print(loginDigits[i]);
    }
    display.print(" ");
  }

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print("JS1:Xoa JS2:OK");

  display.display();
}

// === Hiển thị menu chọn MAC ===
void displayMacMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(10, 0);
  display.println("== SELECT MAC ==");
  display.drawLine(0, 10, 127, 10, SH110X_WHITE);

  int startIdx = 0;
  int maxVisible = 4;
  if (menuCursor >= maxVisible) {
    startIdx = menuCursor - maxVisible + 1;
  }

  for (int i = startIdx; i < min(macCount, startIdx + maxVisible); i++) {
    int y = 14 + (i - startIdx) * 12;

    if (i == menuCursor) {
      display.setCursor(0, y);
      display.print(">");
    }

    if (i == selectedMAC) {
      display.setCursor(120, y);
      display.print("*");
    }

    display.setCursor(8, y);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             macList[i][0], macList[i][1], macList[i][2],
             macList[i][3], macList[i][4], macList[i][5]);
    display.print(macStr);
  }

  display.setCursor(0, 56);
  display.print("B11:Up B9:Dn B10:OK");

  display.display();
}

// === Hiển thị OLED chính ===
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           receiverMAC[0], receiverMAC[1], receiverMAC[2],
           receiverMAC[3], receiverMAC[4], receiverMAC[5]);
  display.print(macStr);
  display.print(" ");
  display.println(espNowConnected ? "Yes" : "No");

  // === Thanh trượt LY (trái) ===
  int sx = 6, st = 14, sb = 50;
  display.drawLine(sx, st, sx, sb, SH110X_WHITE);
  display.fillCircle(sx, map(data.ly, 0, 255, sb, st), 3, SH110X_WHITE);
  display.setCursor(0, sb + 3);
  display.print(data.ly);

  // === Thanh POT (giữa) ===
  int barX = 30, barY = 14, barW = 12, barH = 36;
  display.drawRect(barX, barY, barW, barH, SH110X_WHITE);
  int fillH = map(data.pot, 0, 255, 0, barH - 2);
  if (fillH > 0) {
    display.fillRect(barX + 1, barY + barH - 1 - fillH, barW - 2, fillH, SH110X_WHITE);
  }
  display.setCursor(barX, barY + barH + 3);
  display.print(data.pot);

  // === Joystick RX/RY (phải) ===
  int cx = 90, cy = 32, R = 14, r = 2;
  display.drawCircle(cx, cy, R, SH110X_WHITE);
  display.drawLine(cx - 2, cy, cx + 2, cy, SH110X_WHITE);
  display.drawLine(cx, cy - 2, cx, cy + 2, SH110X_WHITE);

  int dotX = map(data.rx, 0, 255, cx + R - r, cx - R + r);
  int dotY = map(data.ry, 0, 255, cy + R - r, cy - R + r);

  float dx = dotX - cx, dy = dotY - cy;
  float dist = sqrt(dx * dx + dy * dy);
  float maxD = R - r;
  if (dist > maxD) {
    dotX = cx + (int)(dx * maxD / dist);
    dotY = cy + (int)(dy * maxD / dist);
  }

  display.fillCircle(dotX, dotY, r, SH110X_WHITE);
  display.setCursor(cx - R, cy + R + 3);
  display.print(data.rx);
  display.print(",");
  display.print(data.ry);

  display.display();
}

void setup() {
  Serial.begin(115200);

  // Khởi tạo OLED
  Wire.begin(21, 22);
  if (!display.begin(0x3C, true)) {
    Serial.println("OLED init failed!");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 20);
  display.println("  Initializing...");
  display.display();

  // Cấu hình WiFi
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(84);
  setCpuFrequencyMhz(240);

  // Khởi tạo ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("ESP-NOW INIT FAIL!");
    display.display();
    return;
  }

  esp_now_set_wake_window(65535);
  esp_now_register_send_cb(OnDataSent);

  memcpy(receiverMAC, macList[selectedMAC], 6);
  setupPeer();

  // Cấu hình 14 nút nhấn
  pinMode(4, INPUT_PULLUP);   // B1
  pinMode(5, INPUT_PULLUP);   // B2
  pinMode(12, INPUT_PULLUP);  // B3
  pinMode(13, INPUT_PULLUP);  // B4
  pinMode(14, INPUT_PULLUP);  // B5
  pinMode(15, INPUT_PULLUP);  // B6
  pinMode(16, INPUT_PULLUP);  // B7
  pinMode(17, INPUT_PULLUP);  // B8
  pinMode(18, INPUT_PULLUP);  // B9
  pinMode(19, INPUT_PULLUP);  // B10
  pinMode(23, INPUT_PULLUP);  // B11
  pinMode(25, INPUT_PULLUP);  // B12
  pinMode(26, INPUT_PULLUP);  // JS1
  pinMode(27, INPUT_PULLUP);  // JS2

  resetLoginInput();
}

void loop() {
  unsigned long now = millis();

  // Đọc nút (active low)
  bool b1_pressed  = !digitalRead(4);
  bool b2_pressed  = !digitalRead(5);
  bool b3_pressed  = !digitalRead(12);
  bool b4_pressed  = !digitalRead(13);
  bool b5_pressed  = !digitalRead(14);
  bool b6_pressed  = !digitalRead(15);
  bool b7_pressed  = !digitalRead(16);
  bool b8_pressed  = !digitalRead(17);
  bool b9_pressed  = !digitalRead(18);
  bool b10_pressed = !digitalRead(19);
  bool b11_pressed = !digitalRead(23);
  bool b12_pressed = !digitalRead(25);
  bool js1_pressed = !digitalRead(26);
  bool js2_pressed = !digitalRead(27);

  // === Chế độ LOGIN ===
  if (inLoginMode) {
    if (b1_pressed && (now - lastB1Press > debounceDelay)) {
      lastB1Press = now;
      inputLoginDigit(1);
    }
    if (b2_pressed && (now - lastB2Press > debounceDelay)) {
      lastB2Press = now;
      inputLoginDigit(2);
    }
    if (b3_pressed && (now - lastB3Press > debounceDelay)) {
      lastB3Press = now;
      inputLoginDigit(3);
    }
    if (b4_pressed && (now - lastB4Press > debounceDelay)) {
      lastB4Press = now;
      inputLoginDigit(4);
    }
    if (b5_pressed && (now - lastB5Press > debounceDelay)) {
      lastB5Press = now;
      inputLoginDigit(5);
    }
    if (b6_pressed && (now - lastB6Press > debounceDelay)) {
      lastB6Press = now;
      inputLoginDigit(6);
    }
    if (b7_pressed && (now - lastB7Press > debounceDelay)) {
      lastB7Press = now;
      inputLoginDigit(7);
    }
    if (b8_pressed && (now - lastB8Press > debounceDelay)) {
      lastB8Press = now;
      inputLoginDigit(8);
    }
    if (b9_pressed && (now - lastB9Press > debounceDelay)) {
      lastB9Press = now;
      inputLoginDigit(9);
    }

    // B10 = số 0
    if (b10_pressed && (now - lastB10Press > debounceDelay)) {
      lastB10Press = now;
      inputLoginDigit(0);
    }

    // JS1 = xóa
    if (js1_pressed && (now - lastJs1Press > debounceDelay)) {
      lastJs1Press = now;
      deleteLastLoginDigit();
    }

    // JS2 = xác nhận
    if (js2_pressed && (now - lastJs2Press > debounceDelay)) {
      lastJs2Press = now;

      if (loginIndex >= 4) {
        if (isPasswordCorrect()) {
          inLoginMode = false;
          inMacMenu = true;
          menuCursor = selectedMAC;
          Serial.println(">> Login OK, open MAC menu");
        } else {
          inLoginMode = false;
          resetLoginInput();
          Serial.println(">> Login FAIL");
        }
      }
    }

    displayLoginScreen();
    delay(50);
    return;
  }

  // === Chế độ MENU MAC ===
  if (inMacMenu) {
    if (b11_pressed && (now - lastB11Press > debounceDelay)) {
      lastB11Press = now;
      menuCursor--;
      if (menuCursor < 0) menuCursor = macCount - 1;
    }

    if (b9_pressed && (now - lastB9Press > debounceDelay)) {
      lastB9Press = now;
      menuCursor++;
      if (menuCursor >= macCount) menuCursor = 0;
    }

    if (b10_pressed && (now - lastB10Press > debounceDelay)) {
      lastB10Press = now;
      selectedMAC = menuCursor;
      setupPeer();
      inMacMenu = false;
      Serial.println(">> MAC selected, menu closed");
    }

    displayMacMenu();
    delay(50);
    return;
  }

  // === B11 vào login ===
  if (b11_pressed && (now - lastB11Press > debounceDelay)) {
    lastB11Press = now;
    inLoginMode = true;
    resetLoginInput();
    Serial.println(">> Enter login mode");
    displayLoginScreen();
    delay(50);
    return;
  }

  // === Chế độ hoạt động bình thường ===
  data.header = 0x4E;
  data.footer = 0x42;

  // Đọc joystick và biến trở
  data.ly  = map(analogRead(33), 0, 4095, 0, 255);
  data.rx  = map(analogRead(34), 0, 4095, 0, 255);
  data.ry  = map(analogRead(35), 0, 4095, 0, 255);
  data.pot = map(analogRead(32), 0, 4095, 0, 255);

  // Đọc 14 nút nhấn
  data.b1  = b1_pressed;
  data.b2  = b2_pressed;
  data.b3  = b3_pressed;
  data.b4  = b4_pressed;
  data.b5  = b5_pressed;
  data.b6  = b6_pressed;
  data.b7  = b7_pressed;
  data.b8  = b8_pressed;
  data.b9  = b9_pressed;
  data.b10 = b10_pressed;
  data.b11 = b11_pressed;
  data.b12 = b12_pressed;
  data.js1 = js1_pressed;
  data.js2 = js2_pressed;

  // Gửi dữ liệu
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
  if (result != ESP_OK) {
    Serial.println("Gửi ESP-NOW thất bại!");
  }

  // Cập nhật OLED mỗi 50ms
  if (now - lastSendTime > 50) {
    updateOLED();
    lastSendTime = now;
  }

  // In Serial
  Serial.print("Header: "); Serial.print(data.header, HEX);
  Serial.print(" | LY: ");  Serial.print(data.ly);
  Serial.print(" | RX: ");  Serial.print(data.rx);
  Serial.print(" | RY: ");  Serial.print(data.ry);
  Serial.print(" | Pot: "); Serial.print(data.pot);
  Serial.print(" | B1: ");  Serial.print(data.b1);
  Serial.print(" | B2: ");  Serial.print(data.b2);
  Serial.print(" | B3: ");  Serial.print(data.b3);
  Serial.print(" | B4: ");  Serial.print(data.b4);
  Serial.print(" | B5: ");  Serial.print(data.b5);
  Serial.print(" | B6: ");  Serial.print(data.b6);
  Serial.print(" | B7: ");  Serial.print(data.b7);
  Serial.print(" | B8: ");  Serial.print(data.b8);
  Serial.print(" | B9: ");  Serial.print(data.b9);
  Serial.print(" | B10: "); Serial.print(data.b10);
  Serial.print(" | B11: "); Serial.print(data.b11);
  Serial.print(" | B12: "); Serial.print(data.b12);
  Serial.print(" | JS1: "); Serial.print(data.js1);
  Serial.print(" | JS2: "); Serial.print(data.js2);
  Serial.print(" | Footer: "); Serial.println(data.footer, HEX);

  delay(100);
}