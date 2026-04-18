#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <math.h>

// OLED 1.3 inch SH1106 (128x64), I2C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== Config =====
uint8_t receiverMAC[6] = {0x94, 0x51, 0xDC, 0x3A, 0xC8, 0xE8};

const char* WIFI_SSID      = "TP-LINK_10BA";
const char* WIFI_PASSWORD  = "04785439";
const int   RASP_PORT      = 5000;

// IP targets - them IP vao day thoai mai
#define NUM_TARGETS 4
const char* targetNames[NUM_TARGETS] = { "RASP", "CHON", "KHOA", "Quan" };
const char* targetIPs[NUM_TARGETS]   = { "192.168.0.109", "10.160.59.63", "10.160.59.8", "192.168.0.106" };

// Active target IP
char activeIP[16] = "192.168.0.109";

WiFiUDP udp;

// ===== State =====
bool espNowConnected = false;
unsigned long lastSendTime = 0;
unsigned long lastOledTime = 0;

Preferences preferences;

enum Mode { MODE_CAR, MODE_DOG };
Mode currentMode = MODE_CAR;

// Menu states
enum MenuState { MENU_MODE, MENU_IP, MENU_NONE };
MenuState menuState = MENU_MODE;
int menuSelection = 0;
int ipSelection = 0;

// Debounce
bool lastB9  = false;
bool lastB10 = false;
bool lastB11 = false;
unsigned long lastB9Time  = 0;
unsigned long lastB10Time = 0;
unsigned long lastB11Time = 0;
const unsigned long DEBOUNCE_MS = 200;

// T_cycle (DOG mode speed)
float T_cycle = 0.60f;
const float T_MIN = 0.10f;
const float T_MAX = 3.00f;
const float T_STEP = 0.10f;

// Data packet
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

// Forward declarations
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void drawModeMenu();
void drawIPMenu();
void updateOLED();
void readInputs();

void clampAndRoundT() {
  if (T_cycle < T_MIN) T_cycle = T_MIN;
  if (T_cycle > T_MAX) T_cycle = T_MAX;
  T_cycle = roundf(T_cycle * 10.0f) / 10.0f;
}

// ===== ESP-NOW setup (CAR mode) =====
void setupESPNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(84);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_set_wake_window(65535);
  esp_now_register_send_cb(OnDataSent);

  esp_now_del_peer(receiverMAC);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 6;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  espNowConnected = (status == ESP_NOW_SEND_SUCCESS);
}

// ===== WiFi setup (DOG mode) =====
void setupWiFiStation() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 8);
  display.print("DOG MODE");
  display.drawLine(0, 18, 127, 18, SH110X_WHITE);
  display.setCursor(10, 24);
  display.print("WiFi: ");
  display.print(WIFI_SSID);
  display.setCursor(10, 36);
  display.print("Target: ");
  display.print(activeIP);
  display.setCursor(10, 50);
  display.print("Connecting");
  display.display();

  unsigned long start = millis();
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(400);
    display.fillRect(70, 50, 58, 8, SH110X_BLACK);
    display.setCursor(70, 50);
    for (int i = 0; i < dots; i++) display.print(".");
    display.display();
    dots++;
    if (dots > 8) dots = 0;
  }

  if (WiFi.status() == WL_CONNECTED) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 15);
    display.print("WiFi Connected!");
    display.setCursor(10, 30);
    display.print("My IP: ");
    display.print(WiFi.localIP().toString());
    display.setCursor(10, 45);
    display.print("-> ");
    display.print(activeIP);
    display.display();
    delay(1200);
  } else {
    display.clearDisplay();
    display.setCursor(10, 25);
    display.print("WiFi FAILED!");
    display.display();
    delay(1500);
  }
}

// ===== Deinit =====
void deinitCurrentMode() {
  if (currentMode == MODE_CAR) {
    esp_now_deinit();
  } else {
    WiFi.disconnect(true);
  }
  delay(100);
}

// ===== Init mode =====
void initMode(Mode mode) {
  currentMode = mode;
  if (mode == MODE_CAR) {
    setupESPNow();
  } else {
    setupWiFiStation();
  }
}

// ===== Send UDP =====
void sendUDP() {
  if (WiFi.status() != WL_CONNECTED) return;
  udp.beginPacket(activeIP, RASP_PORT);
  udp.write((uint8_t *)&data, sizeof(data));
  udp.endPacket();
}

// ================================================================
//  MENU 1: Mode select
// ================================================================
void drawModeMenu() {
  display.clearDisplay();

  display.fillRect(0, 0, 128, 13, SH110X_WHITE);
  display.setTextSize(1);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(28, 3);
  display.print("SELECT MODE");

  int carY = 21;
  display.setTextColor(SH110X_WHITE);
  if (menuSelection == 0) {
    display.fillRoundRect(4, carY, 120, 18, 4, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
  } else {
    display.drawRoundRect(4, carY, 120, 18, 4, SH110X_WHITE);
  }
  display.setCursor(10, carY + 5);
  display.print(menuSelection == 0 ? "> " : "  ");
  display.setCursor(22, carY + 5);
  display.print("CAR   ESP-NOW");

  int dogY = 43;
  display.setTextColor(SH110X_WHITE);
  if (menuSelection == 1) {
    display.fillRoundRect(4, dogY, 120, 18, 4, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
  } else {
    display.drawRoundRect(4, dogY, 120, 18, 4, SH110X_WHITE);
  }
  display.setCursor(10, dogY + 5);
  display.print(menuSelection == 1 ? "> " : "  ");
  display.setCursor(22, dogY + 5);
  display.print("DOG   WiFi/UDP");

  display.display();
}

// ================================================================
//  MENU 2: IP target (scrollable, 3 visible at a time)
// ================================================================
void drawIPMenu() {
  display.clearDisplay();

  display.fillRect(0, 0, 128, 13, SH110X_WHITE);
  display.setTextSize(1);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(22, 3);
  display.print("SELECT TARGET");

  // Scrollable window: max 3 items visible
  const int maxVisible = 3;
  int startIdx = 0;

  if (ipSelection >= maxVisible) {
    startIdx = ipSelection - maxVisible + 1;
  }
  if (startIdx > NUM_TARGETS - maxVisible) {
    startIdx = NUM_TARGETS - maxVisible;
  }
  if (startIdx < 0) startIdx = 0;

  int endIdx = startIdx + maxVisible;
  if (endIdx > NUM_TARGETS) endIdx = NUM_TARGETS;

  for (int i = startIdx; i < endIdx; i++) {
    int slot = i - startIdx;
    int y = 17 + slot * 16;

    display.setTextColor(SH110X_WHITE);
    if (ipSelection == i) {
      display.fillRoundRect(4, y, 120, 14, 3, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.drawRoundRect(4, y, 120, 14, 3, SH110X_WHITE);
    }

    display.setCursor(8, y + 3);
    display.print(ipSelection == i ? "> " : "  ");
    display.print(targetNames[i]);

    int ipLen = strlen(targetIPs[i]);
    display.setCursor(128 - 6 - ipLen * 6, y + 3);
    display.print(targetIPs[i]);
  }

  // Scroll arrows
  if (startIdx > 0) {
    display.fillTriangle(124, 15, 120, 19, 127, 19, SH110X_WHITE);
  }
  if (endIdx < NUM_TARGETS) {
    display.fillTriangle(124, 63, 120, 59, 127, 59, SH110X_WHITE);
  }

  display.display();
}

// ================================================================
//  MAIN OLED UI
// ================================================================
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);

  // ===== TOP BAR =====
  display.fillRect(0, 0, 128, 12, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);

  if (currentMode == MODE_CAR) {
    display.setCursor(2, 2);
    display.print("CAR");
    display.setCursor(26, 2);
    display.print(espNowConnected ? "OK" : "--");
    char macStr[12];
    snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X",
             receiverMAC[2], receiverMAC[3], receiverMAC[4], receiverMAC[5]);
    display.setCursor(80, 2);
    display.print(macStr);
  } else {
    display.setCursor(2, 2);
    display.print("DOG");
    if (WiFi.status() == WL_CONNECTED) {
      display.setCursor(26, 2);
      display.print("OK");
    } else {
      display.setCursor(26, 2);
      display.print("NO WIFI");
    }
  }

  display.setTextColor(SH110X_WHITE);

  // =========================================================
  //  CAR MODE UI
  // =========================================================
  if (currentMode == MODE_CAR) {
    // LEFT: LY slider
    int sliderX = 5, sliderTop = 16, sliderBot = 52;
    int sliderH = sliderBot - sliderTop;

    display.drawLine(sliderX, sliderTop, sliderX, sliderBot, SH110X_WHITE);
    display.drawLine(sliderX - 2, sliderTop, sliderX + 2, sliderTop, SH110X_WHITE);
    display.drawLine(sliderX - 2, sliderBot, sliderX + 2, sliderBot, SH110X_WHITE);
    int sliderMid = sliderTop + sliderH / 2;
    display.drawLine(sliderX - 2, sliderMid, sliderX + 2, sliderMid, SH110X_WHITE);

    int thumbY = map(data.ly, 0, 255, sliderBot, sliderTop);
    display.fillRoundRect(sliderX - 4, thumbY - 3, 9, 7, 2, SH110X_WHITE);
    display.setCursor(0, 55);
    display.print(data.ly);

    // CENTER: POT bar
    int barX = 30, barY = 16, barW = 16, barH = 40;
    display.drawRect(barX, barY, barW, barH, SH110X_WHITE);
    display.drawRect(barX + 1, barY + 1, barW - 2, barH - 2, SH110X_WHITE);
    int fillH = map(data.pot, 0, 255, 0, barH - 4);
    if (fillH > 0) {
      display.fillRect(barX + 2, barY + barH - 2 - fillH, barW - 4, fillH, SH110X_WHITE);
    }
    int pct = map(data.pot, 0, 255, 0, 100);
    display.setCursor(barX + 1, barY + barH + 3);
    display.print(pct);
    display.print("%");

    // RIGHT: RX/RY joystick
    int cx = 88, cy = 38, R = 20, r = 3;
    display.drawCircle(cx, cy, R, SH110X_WHITE);
    display.drawLine(cx - 3, cy, cx + 3, cy, SH110X_WHITE);
    display.drawLine(cx, cy - 3, cx, cy + 3, SH110X_WHITE);

    int dotX = map(data.rx, 0, 255, cx + R - r, cx - R + r);
    int dotY = map(data.ry, 0, 255, cy + R - r, cy - R + r);
    float dx = dotX - cx, dy = dotY - cy;
    float dist = sqrtf(dx * dx + dy * dy);
    float maxD = R - r;
    if (dist > maxD && dist > 0.0f) {
      dotX = cx + (int)(dx * maxD / dist);
      dotY = cy + (int)(dy * maxD / dist);
    }
    display.fillCircle(dotX, dotY, r, SH110X_WHITE);
    display.setCursor(cx - R + 2, cy + R + 2);
    display.print(data.rx);
    display.print(",");
    display.print(data.ry);
  }

  // =========================================================
  //  DOG MODE UI: 2 joysticks + T_cycle
  // =========================================================
  else {
    uint8_t lx = data.pot;
    uint8_t ly = data.ly;
    uint8_t rx = data.rx;
    uint8_t ry = data.ry;

    // LEFT joystick
    int cxL = 30, cyL = 31, RL = 16, rL = 2;
    display.setCursor(cxL - 3, 14);
    display.print("L");
    display.drawCircle(cxL, cyL, RL, SH110X_WHITE);
    display.drawLine(cxL - 3, cyL, cxL + 3, cyL, SH110X_WHITE);
    display.drawLine(cxL, cyL - 3, cxL, cyL + 3, SH110X_WHITE);

    int dotXL = map(lx, 0, 255, cxL + RL - rL, cxL - RL + rL);
    int dotYL = map(ly, 0, 255, cyL + RL - rL, cyL - RL + rL);
    float dxL = dotXL - cxL, dyL = dotYL - cyL;
    float distL = sqrtf(dxL * dxL + dyL * dyL);
    float maxDL = RL - rL;
    if (distL > maxDL && distL > 0.0f) {
      dotXL = cxL + (int)(dxL * maxDL / distL);
      dotYL = cyL + (int)(dyL * maxDL / distL);
    }
    display.fillCircle(dotXL, dotYL, rL, SH110X_WHITE);
    display.setCursor(cxL - 12, 52);
    display.print(lx);
    display.print(",");
    display.print(ly);

    // RIGHT joystick
    int cxR = 98, cyR = 31, RR = 16, rR = 2;
    display.setCursor(cxR - 3, 14);
    display.print("R");
    display.drawCircle(cxR, cyR, RR, SH110X_WHITE);
    display.drawLine(cxR - 3, cyR, cxR + 3, cyR, SH110X_WHITE);
    display.drawLine(cxR, cyR - 3, cxR, cyR + 3, SH110X_WHITE);

    int dotXR = map(rx, 0, 255, cxR + RR - rR, cxR - RR + rR);
    int dotYR = map(ry, 0, 255, cyR + RR - rR, cyR - RR + rR);
    float dxR = dotXR - cxR, dyR = dotYR - cyR;
    float distR = sqrtf(dxR * dxR + dyR * dyR);
    float maxDR = RR - rR;
    if (distR > maxDR && distR > 0.0f) {
      dotXR = cxR + (int)(dxR * maxDR / distR);
      dotYR = cyR + (int)(dyR * maxDR / distR);
    }
    display.fillCircle(dotXR, dotYR, rR, SH110X_WHITE);
    display.setCursor(cxR - 12, 52);
    display.print(rx);
    display.print(",");
    display.print(ry);

    // T_cycle in center
    char tBuf[16];
    snprintf(tBuf, sizeof(tBuf), "T=%.1f", T_cycle);
    int textW = strlen(tBuf) * 6;
    int textX = (128 - textW) / 2;
    display.setCursor(textX, 40);
    display.print(tBuf);
  }

  display.display();
}

// ===== Read all inputs =====
void readInputs() {
  data.header = 0x4E;
  data.footer = 0x42;

  data.ly  = map(analogRead(33), 0, 4095, 0, 255);
  data.rx  = map(analogRead(34), 0, 4095, 0, 255);
  data.ry  = map(analogRead(35), 0, 4095, 0, 255);
  data.pot = map(analogRead(32), 0, 4095, 0, 255);

  data.b1  = !digitalRead(4);
  data.b2  = !digitalRead(5);
  data.b3  = !digitalRead(12);
  data.b4  = !digitalRead(13);
  data.b5  = !digitalRead(14);
  data.b6  = !digitalRead(15);
  data.b7  = !digitalRead(16);
  data.b8  = !digitalRead(17);
  data.b9  = !digitalRead(18);
  data.b10 = !digitalRead(19);
  data.b11 = !digitalRead(23);
  data.b12 = !digitalRead(25);
  data.js1 = !digitalRead(26);
  data.js2 = !digitalRead(27);
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);

  Wire.begin(21, 22);
  if (!display.begin(0x3C, true)) {
    Serial.println("OLED init failed!");
  }

  // Boot splash
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(30, 10);
  display.println("RC TX");
  display.setTextSize(1);
  display.setCursor(25, 35);
  display.println("CAR | DOG Mode");
  display.setCursor(30, 50);
  display.println("Starting...");
  display.display();
  delay(1200);

  // Button pins
  int pins[] = {4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 23, 25, 26, 27};
  for (int i = 0; i < 14; i++) {
    pinMode(pins[i], INPUT_PULLUP);
  }

  // Load saved settings
  preferences.begin("rc", false);
  int savedMode = preferences.getInt("mode", -1);
  int savedIP   = preferences.getInt("ipIdx", 0);
  T_cycle = 0.60f;
  preferences.end();

  clampAndRoundT();

  if (savedMode >= 0) {
    currentMode = (Mode)savedMode;
    menuSelection = savedMode;
    ipSelection = savedIP;
    if (ipSelection < 0 || ipSelection >= NUM_TARGETS) ipSelection = 0;
    strncpy(activeIP, targetIPs[ipSelection], sizeof(activeIP) - 1);
    activeIP[sizeof(activeIP) - 1] = '\0';
    menuState = MENU_NONE;
    initMode(currentMode);
  } else {
    menuState = MENU_MODE;
    menuSelection = 0;
    ipSelection = 0;
    drawModeMenu();
  }
}

// ===== Loop =====
void loop() {
  unsigned long now = millis();

  bool b9_now  = !digitalRead(18);
  bool b10_now = !digitalRead(19);
  bool b11_now = !digitalRead(23);

  bool b9_rising  = (b9_now  && !lastB9  && (now - lastB9Time  > DEBOUNCE_MS));
  bool b10_rising = (b10_now && !lastB10 && (now - lastB10Time > DEBOUNCE_MS));
  bool b11_rising = (b11_now && !lastB11 && (now - lastB11Time > DEBOUNCE_MS));

  if (b9_rising)  lastB9Time  = now;
  if (b10_rising) lastB10Time = now;
  if (b11_rising) lastB11Time = now;

  lastB9  = b9_now;
  lastB10 = b10_now;
  lastB11 = b11_now;

  // ===== MENU MODE SELECT =====
  if (menuState == MENU_MODE) {
    if (b11_rising) { menuSelection = 0; drawModeMenu(); }
    if (b9_rising)  { menuSelection = 1; drawModeMenu(); }
    if (b10_rising) {
      if (menuSelection == 0) {
        menuState = MENU_NONE;
        preferences.begin("rc", false);
        preferences.putInt("mode", 0);
        preferences.end();
        deinitCurrentMode();
        initMode(MODE_CAR);
      } else {
        menuState = MENU_IP;
        drawIPMenu();
      }
    }
    delay(50);
    return;
  }

  // ===== MENU IP SELECT =====
  if (menuState == MENU_IP) {
    if (b11_rising) {
      ipSelection--;
      if (ipSelection < 0) ipSelection = NUM_TARGETS - 1;
      drawIPMenu();
    }
    if (b9_rising) {
      ipSelection++;
      if (ipSelection >= NUM_TARGETS) ipSelection = 0;
      drawIPMenu();
    }
    if (b10_rising) {
      strncpy(activeIP, targetIPs[ipSelection], sizeof(activeIP) - 1);
      activeIP[sizeof(activeIP) - 1] = '\0';
      menuState = MENU_NONE;
      preferences.begin("rc", false);
      preferences.putInt("mode", 1);
      preferences.putInt("ipIdx", ipSelection);
      preferences.end();
      deinitCurrentMode();
      initMode(MODE_DOG);
    }
    delay(50);
    return;
  }

  // ===== RUNNING =====
  readInputs();

  // ===== DOG MODE: B9/B11 adjust T_cycle =====
  if (currentMode == MODE_DOG) {
    if (b9_rising) {
      T_cycle += T_STEP;
      clampAndRoundT();
      Serial.print("T_cycle = ");
      Serial.println(T_cycle, 1);
    }
    if (b11_rising) {
      T_cycle -= T_STEP;
      clampAndRoundT();
      Serial.print("T_cycle = ");
      Serial.println(T_cycle, 1);
    }
  }

  // ===== B10 -> back to mode menu =====
  if (b10_rising) {
    menuState = MENU_MODE;
    drawModeMenu();
    delay(50);
    return;
  }

  // Send data
  if (now - lastSendTime > 50) {
    if (currentMode == MODE_CAR) {
      esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
    } else {
      sendUDP();
    }
    lastSendTime = now;
  }

  // Update OLED
  if (now - lastOledTime > 80) {
    updateOLED();
    lastOledTime = now;
  }

  delay(20);
}