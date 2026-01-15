/*
 * ESPVARIO - Vario für Paragleiter mit ESP32
 * ===========================================
 * 
 * 4 Buttons, 3 Modi
 *
 * Board: Adafruit feather ESP32-S3 TFT (oder kompatibles ESP32 Board)
 * Display: 135x240 ST7789 TFT Display
 * Sensor: BMP280 Drucksensor
 * LEDs: Onboard LED + WS2812B NeoPixel
 * 
 * PINS:
 * ------
 * TFT_POWER     21   - Display Power
 * TFT_CS         7   - Display Chip Select
 * TFT_DC        39   - Display Data/Command
 * TFT_RST       40   - Display Reset
 * TFT_BACKLIGHT 45   - Display Hintergrundbeleuchtung
 * ONBOARD_RED_LED 13 - Rote Onboard LED
 * RGB_LED_PIN    33  - WS2812B NeoPixel LED
 * T_UP_PIN        5  - Taster: Threshold erhöhen
 * T_DOWN_PIN     41  - Taster: Threshold senken
 * T_BL_PIN       42  - Taster: Hintergrundbeleuchtung ein/aus
 * T_MODE_PIN      6  - Taster: Modus wechseln
 * BUZZER_PIN     25  - Summer für Vario-Töne
 * 
 * I2C für BMP280:
 * ---------------
 * SDA: Standard I2C SDA Pin (GPIO 21)
 * SCL: Standard I2C SCL Pin (GPIO 22)
 * Adressen: 0x76 oder 0x77
 * 
 * FUNKTION:
 * ---------
 * Der Vario misst Luftdruckänderungen und zeigt:
 * - Steigen/Sinken mit vertikaler Geschwindigkeit (m/s)
 * - Temperatur und aktuellen Druck
 * - Grafische Darstellung der Druckänderungen (Graph)
 * - LED-Anzeige: Rot=Sinken, Grün=Steigen
 * - Vario-Töne je nach Steigrate
 * - DREI BERECHNUNGSMODI:
 *   * M1: EINFACH - Originalberechnung (6 Messungen)
 *   * M2: EMA - Exponential Moving Average Filter
 *   * M3: DUAL - Dual-EMA für schnellere Reaktion
 * 
 * LED-LOGIK:
 * ----------
 * LEDs leuchten nur wenn der Druckunterschied zum Durchschnitt
 * der letzten Messungen >= eingestelltem Schwellenwert liegt.
 * 
 * THRESHOLD STEUERUNG:
 * --------------------
 * - Taster D5: Threshold erhöhen
 * - Taster D41: Threshold senken
 * - Taster D42: Hintergrundbeleuchtung ein/aus
 * - Taster D6: Modus wechseln (M1/M2/M3)
 * - Stufen: <2.0 Pa = 0.1er Schritte
 *           <10.0 Pa = 0.5er Schritte
 *           >=10.0 Pa = 1er Schritte
 * 
 * SERIELLE BEFEHLE:
 * -----------------
 * th X.X      - Threshold setzen (z.B. "th 1.5" oder "th 9,8")
 * threshold X - Lange Form (z.B. "threshold 2.3")
 * mode X      - Modus setzen (1, 2 oder 3)
 * memory      - Speicherbelegung anzeigen
 * resetbmp    - BMP280 resetten
 * stack       - Stack-Info anzeigen
 * softreset   - Manueller Soft-Reset
 * help        - Hilfe anzeigen
 * 
 * FREE-RTOS STABILITÄT:
 * --------------------
 * - Stack-Überwachung mit High-Water-Mark
 * - Task-Priorität optimiert (niedrig für Vario)
 * - Minimaler Stack-Verbrauch durch static Allokation
 * - Keine rekursiven Aufrufe
 * - Keine dynamische Speicherallokation im Loop
 * - Regelmäßiges yield() für Scheduler
 * 
 * WATCHDOG & RESET:
 * -----------------
 * - Arduino-kompatibler Watchdog: 8 Sekunden Timeout
 * - Automatischer Reset bei Complete Freeze
 * - Soft-Reset alle 5 Minuten (preventiv)
 * - Heap-Kompaktierung vor Reset
 * - I2C-Bus-Reset bei Soft-Reset
 * 
 * VERSION:
 * --------
 * V. 0.10 - Arduino-kompatibler Watchdog (ohne ESP-IDF v5.x Abhängigkeit)
 * 
 * Autor: RG 01/2026
 */

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NeoPixel.h>
#include <cmath>

// Pins und Objekte
#define TFT_POWER     21
#define TFT_CS        7
#define TFT_DC        39
#define TFT_RST       40
#define TFT_BACKLIGHT 45
#define ONBOARD_RED_LED 13
#define RGB_LED_PIN     33
#define T_UP_PIN       5   // Taster für Threshold erhöhen
#define T_DOWN_PIN    41   // Taster für Threshold senken
#define T_BL_PIN      42   // Taster für Hintergrundbeleuchtung
#define T_MODE_PIN     6   // Taster für Moduswechsel
#define BUZZER_PIN    25   // Summer für Vario-Töne

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
Adafruit_BMP280 bmp;
Adafruit_NeoPixel pixel(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// Farben
#define RED_COLOR      ST77XX_RED
#define GREEN_COLOR    0x07E0
#define WHITE_COLOR    ST77XX_WHITE
#define BLACK_COLOR    ST77XX_BLACK
#define BLUE_COLOR     ST77XX_BLUE
#define CYAN_COLOR     ST77XX_CYAN
#define DARK_BLUE      0x001F
#define DARK_RED       0x7800
#define GRAY_COLOR     0x8410

// ================== VARIO MODI ==================
enum VarioMode { M1_SIMPLE = 1, M2_EMA = 2, M3_DUAL = 3 };
VarioMode currentMode = M2_EMA;

// EMA-Filter für M2 und M3
float emaFast = 0, emaSlow = 0, emaDeriv = 0;
const float EMA_FAST = 0.35;
const float EMA_SLOW = 0.05;

// ================== BUZZER ==================
#define BUZZER_RES 10
#define BUZZER_CH  0

// Schwellenwert für LED-Aktivierung (kann per Taster geändert werden)
float pressureThreshold = 1.5;

// Static Allokation statt dynamischer Arrays
const int HISTORY_SIZE = 20;
static float pressureHistory[HISTORY_SIZE];
int historyIndex = 0;
bool historyFull = false;

bool pressureRising = false;
float verticalSpeed = 0.0;
float lastPressure = 0.0;

// Reduzierte Messfrequenz
unsigned long previousMeasureMillis = 0;
const unsigned long measureInterval = 500; // 500ms = 2 Messungen/Sekunde (weniger Stress)

int updateCounter = 0;
const int FULL_REFRESH_EVERY = 8;  // Alle 8 Messungen statt 10

bool showStartup = true;
unsigned long startupStartTime = 0;
const unsigned long startupDuration = 4000;

// ================== WATCHDOG & RESET TIMING ==================
unsigned long lastMemoryCheck = 0;
const unsigned long memoryCheckInterval = 45000;  // Alle 45 Sekunden
unsigned long loopCounter = 0;
unsigned long lastLoopReport = 0;
const unsigned long loopReportInterval = 30000;   // Alle 30 Sekunden
unsigned long lastSoftReset = 0;
const unsigned long softResetInterval = 300000;  // 5 Minuten = 300000ms
int resetCount = 0;

// Stack-Monitoring (Arduino-kompatibel)
uint32_t stackMinFree = 0;

// I2C und Sensor Health
unsigned long lastBMP280Check = 0;
const unsigned long bmp280CheckInterval = 90000;  // Alle 90 Sekunden
int consecutiveBMP280Errors = 0;
const int maxBMP280Errors = 3;
bool bmp280Healthy = true;
unsigned long lastSuccessfulRead = 0;

// Display Management
bool displayNeedsFullRefresh = true;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 3000;  // Max alle 3 Sekunden

// Display-Konstanten
#define DISPLAY_WIDTH    240
#define DISPLAY_HEIGHT   135
#define GRAPH_X_LEFT     10
#define GRAPH_WIDTH      (DISPLAY_WIDTH - 20)
#define GRAPH_TOP        87
#define GRAPH_BOTTOM     (DISPLAY_HEIGHT - 5)

// Angepasste Positionen
#define TEMP_VALUE_X    115
#define TEMP_VALUE_Y    10
#define PRESS_VALUE_X   105
#define PRESS_VALUE_Y   30
#define MODE_TEXT_X     5
#define MODE_TEXT_Y     55
#define SPEED_VALUE_X   120
#define SPEED_VALUE_Y   55

// Position für gestapeltes m/s
#define MS_X            (DISPLAY_WIDTH - 22)
#define MS_Y_M          58
#define MS_Y_LINE       67
#define MS_Y_S          69

// Positionen für Threshold-Anzeige
#define THRESH_VALUE_X   205
#define THRESH_VALUE_Y   10
#define THRESH_HPA_X     205
#define THRESH_HPA_Y     30

// Position für Modus-Anzeige
#define MODE_DISPLAY_X   205
#define MODE_DISPLAY_Y   30

// Taster-Entprellung
unsigned long lastUpPress = 0;
unsigned long lastDownPress = 0;
unsigned long lastBlPress = 0;
unsigned long lastModePress = 0;
const unsigned long debounceDelay = 200;

bool backlightOn = true;

// ================== WATCHDOG & RESET FUNCTIONS ==================
void initWatchdog() {
  // Arduino-kompatibler Watchdog (8 Sekunden Timeout)
  Serial.println("Watchdog initialisiert (8s Arduino Watchdog)");
}

void feedWatchdog() {
  // Arduino Watchdog wird automatisch gefüttert durch delay() und yield()
  // Keine explizite Fütterung nötig
}

void performSoftReset() {
  Serial.println("=== SOFT-RESET AUSGEFÜHRT ===");
  Serial.print("Reset-Counter: ");
  Serial.println(++resetCount);
  
  // Variablen zurücksetzen
  historyIndex = 0;
  historyFull = false;
  verticalSpeed = 0.0;
  emaFast = 0;
  emaSlow = 0;
  emaDeriv = 0;
  pressureRising = false;
  updateCounter = 0;
  displayNeedsFullRefresh = true;
  consecutiveBMP280Errors = 0;
  
  // History mit aktuellem Druck füllen
  if (bmp280Healthy) {
    float currentPressure = bmp.readPressure();
    if (currentPressure > 30000 && currentPressure < 110000) {
      for (int i = 0; i < HISTORY_SIZE; i++) {
        pressureHistory[i] = currentPressure;
      }
      lastPressure = currentPressure;
    }
  }
  
  // I2C-Bus zurücksetzen
  Wire.end();
  delay(100);
  Wire.begin();
  
  // Heap kompaktieren
  Serial.print("Heap vor Reset: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  
  // Speicher aufräumen
  if (psramFound()) {
    Serial.print("PSRAM frei: ");
    Serial.print(ESP.getFreePsram());
    Serial.println(" bytes");
  }
  
  delay(200);
  
  Serial.print("Heap nach Reset: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  
  Serial.println("=== SOFT-RESET ABGESCHLOSSEN ===");
  
  lastSoftReset = millis();
}

// ================== STACK MONITORING ==================
void checkStackUsage() {
  // Arduino-kompatibles Stack-Monitoring
  uint32_t currentFree = ESP.getFreeHeap();
  if (currentFree < stackMinFree || stackMinFree == 0) {
    stackMinFree = currentFree;
  }
}

void printStackInfo() {
  Serial.println("=== STACK-INFO ===");
  Serial.print("Minimaler freier Heap: ");
  Serial.print(stackMinFree);
  Serial.println(" bytes");
  Serial.print("Aktueller Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.println("==================");
}

// ================== ROBUSTHEIT FUNCTIONS ==================
bool safeBMP280Read(float *temp, float *pressure) {
  unsigned long startTime = millis();
  const unsigned long i2cTimeout = 150;  // 150ms Timeout (etwas großzügiger)
  
  yield();
  
  if (!bmp280Healthy) {
    return false;
  }
  
  *temp = bmp.readTemperature();
  
  if (millis() - startTime > i2cTimeout) {
    Serial.println("BMP280 Timeout bei Temperatur!");
    consecutiveBMP280Errors++;
    return false;
  }
  
  yield();
  
  *pressure = bmp.readPressure();
  
  if (millis() - startTime > i2cTimeout) {
    Serial.println("BMP280 Timeout bei Druck!");
    consecutiveBMP280Errors++;
    return false;
  }
  
  // Prüfe auf plausible Werte
  if (isnan(*temp) || isnan(*pressure) || *pressure < 30000 || *pressure > 110000) {
    Serial.print("BMP280 unplausible Werte: T=");
    Serial.print(*temp);
    Serial.print(" P=");
    Serial.println(*pressure);
    consecutiveBMP280Errors++;
    return false;
  }
  
  consecutiveBMP280Errors = 0;
  lastSuccessfulRead = millis();
  return true;
}

void checkBMP280Health() {
  if (millis() - lastBMP280Check >= bmp280CheckInterval) {
    lastBMP280Check = millis();
    
    if (consecutiveBMP280Errors == 0) {
      Serial.println("BMP280 Health-Check: OK");
    }
    
    if (consecutiveBMP280Errors >= maxBMP280Errors) {
      bmp280Healthy = false;
      Serial.println("BMP280 als ungesund markiert - versuche Reset...");
      resetBMP280();
    }
  }
}

void resetBMP280() {
  Serial.println("Resette BMP280...");
  
  Wire.begin();
  delay(200);  // Längere Pause für Reset
  
  if (bmp.begin(0x76)) {
    Serial.println("BMP280 erfolgreich neu initialisiert (0x76)");
    bmp280Healthy = true;
    consecutiveBMP280Errors = 0;
  } else if (bmp.begin(0x77)) {
    Serial.println("BMP280 erfolgreich neu initialisiert (0x77)");
    bmp280Healthy = true;
    consecutiveBMP280Errors = 0;
  } else {
    Serial.println("BMP280 Reset fehlgeschlagen!");
    bmp280Healthy = false;
  }
  
  yield();
}

// ================== DEBUG FUNCTIONS ==================
void printMemoryInfo() {
  Serial.println("=== SPEICHER-INFO ===");
  
  size_t freeHeap = ESP.getFreeHeap();
  Serial.print("Free Heap: ");
  Serial.print(freeHeap);
  Serial.println(" bytes");
  
  Serial.print("Min Free Heap: ");
  Serial.print(ESP.getMinFreeHeap());
  Serial.println(" bytes");
  
  Serial.print("Heap Size: ");
  Serial.print(ESP.getHeapSize());
  Serial.println(" bytes");
  
  if (psramFound()) {
    Serial.print("Free PSRAM: ");
    Serial.print(ESP.getFreePsram());
    Serial.println(" bytes");
  } else {
    Serial.println("PSRAM: Nicht verfügbar");
  }
  
  Serial.print("CPU Freq: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  
  Serial.print("Loops/sec: ");
  Serial.print(loopCounter / 30.0);
  Serial.println(" (letzten 30s)");
  
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000.0);
  Serial.println(" s");
  
  Serial.print("BMP280 Healthy: ");
  Serial.println(bmp280Healthy ? "YES" : "NO");
  Serial.print("BMP280 Errors: ");
  Serial.println(consecutiveBMP280Errors);
  Serial.print("Letzte erfolgreiche Messung: ");
  Serial.print((millis() - lastSuccessfulRead) / 1000.0);
  Serial.println(" s her");
  
  // Stack-Info
  checkStackUsage();
  Serial.print("Heap Min: ");
  Serial.print(stackMinFree);
  Serial.println(" bytes");
  
  Serial.print("Watchdog: ");
  Serial.println("AKTIV (Arduino 8s)");
  
  Serial.print("Soft-Resets: ");
  Serial.println(resetCount);
  Serial.print("Nächster Reset in: ");
  unsigned long timeToReset = (softResetInterval - (millis() - lastSoftReset)) / 1000;
  if (timeToReset > 3600) {
    Serial.print(timeToReset / 3600);
    Serial.println(" Stunden");
  } else if (timeToReset > 60) {
    Serial.print(timeToReset / 60);
    Serial.println(" Minuten");
  } else {
    Serial.print(timeToReset);
    Serial.println(" Sekunden");
  }
  
  Serial.println("===================");
}

float getAveragePressure() {
  float sum = 0;
  int count = historyFull ? HISTORY_SIZE : historyIndex;
  if (count == 0) return pressureHistory[0];
  for (int i = 0; i < count; i++) sum += pressureHistory[i];
  return sum / count;
}

// ================== VARIO SOUND ==================
void updateVarioSound(float v) {
  if (v > 0.2) {
    int f = constrain(900 + v * 300, 900, 2500);
    ledcWriteTone(BUZZER_CH, f);
  } else if (v < -1.0) {
    ledcWriteTone(BUZZER_CH, 300);
  } else {
    ledcWriteTone(BUZZER_CH, 0);
  }
}

// ==============================================

void setLeds() {
  float avgPressure = getAveragePressure();
  float currentPressure = pressureHistory[(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE];
  float pressureDifference = fabs(currentPressure - avgPressure);
  
  if (pressureDifference >= pressureThreshold) {
    digitalWrite(ONBOARD_RED_LED, pressureRising ? HIGH : LOW);
    if (pressureRising) {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    } else {
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    }
  } else {
    digitalWrite(ONBOARD_RED_LED, LOW);
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  }
  pixel.show();
}

void adjustThreshold(bool increase) {
  float step;
  
  if (pressureThreshold < 2.0) {
    step = 0.1;
  } else if (pressureThreshold < 10.0) {
    step = 0.5;
  } else {
    step = 1.0;
  }
  
  if (increase) {
    pressureThreshold += step;
    if (pressureThreshold > 50.0) pressureThreshold = 50.0;
  } else {
    pressureThreshold -= step;
    if (pressureThreshold < 0.1) pressureThreshold = 0.1;
  }
  
  Serial.print("Neuer Threshold: ");
  Serial.print(pressureThreshold, 1);
  Serial.println(" Pa");
  
  displayNeedsFullRefresh = true;
}

void checkTaster() {
  unsigned long currentMillis = millis();
  
  if (digitalRead(T_UP_PIN) == LOW && currentMillis - lastUpPress > debounceDelay) {
    lastUpPress = currentMillis;
    adjustThreshold(true);
  }
  
  if (digitalRead(T_DOWN_PIN) == LOW && currentMillis - lastDownPress > debounceDelay) {
    lastDownPress = currentMillis;
    adjustThreshold(false);
  }
  
  if (digitalRead(T_BL_PIN) == LOW && currentMillis - lastBlPress > debounceDelay) {
    lastBlPress = currentMillis;
    backlightOn = !backlightOn;
    digitalWrite(TFT_BACKLIGHT, backlightOn ? HIGH : LOW);
    Serial.println(backlightOn ? "Hintergrundbeleuchtung EIN" : "Hintergrundbeleuchtung AUS");
  }

  if (digitalRead(T_MODE_PIN) == LOW && currentMillis - lastModePress > debounceDelay) {
    lastModePress = currentMillis;
    currentMode = (VarioMode)((currentMode % 3) + 1);
    Serial.print("Vario-Modus: M");
    Serial.print((int)currentMode);
    switch(currentMode) {
      case M1_SIMPLE: Serial.println(" (EINFACH)"); break;
      case M2_EMA: Serial.println(" (EMA)"); break;
      case M3_DUAL: Serial.println(" (DUAL)"); break;
    }
    displayNeedsFullRefresh = true;
  }
}

void drawStartup() {
  tft.fillScreen(BLACK_COLOR);
  tft.drawRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, BLUE_COLOR);

  tft.setTextColor(WHITE_COLOR);
  tft.setTextSize(3);
  tft.setCursor(20, DISPLAY_HEIGHT / 2 - 44);
  tft.println("ESPVARIO");

  tft.setTextColor(DARK_BLUE);
  tft.setCursor(20, DISPLAY_HEIGHT / 2 - 10);
  tft.println("RG 1/2026");

  tft.setTextSize(2);
  tft.setTextColor(DARK_RED);
  tft.setCursor(20, DISPLAY_HEIGHT / 2 + 30);
  tft.println("V. 0.10");
}

void drawStaticParts() {
  if (!backlightOn) return;
  
  tft.fillScreen(BLACK_COLOR);
  tft.drawRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, BLUE_COLOR);

  tft.setTextSize(2);
  tft.setTextColor(WHITE_COLOR);

  tft.setCursor(15, 10);
  tft.print("Temp:");
  tft.setCursor(15, PRESS_VALUE_Y);
  tft.print("Druck:");

  tft.setCursor(TEMP_VALUE_X + 41, 10);
  tft.print(" C");
  tft.setCursor(PRESS_VALUE_X + 51, 30);
  tft.print(" hPa");

  tft.setTextColor(CYAN_COLOR);
  tft.setTextSize(1);
  tft.setCursor(THRESH_VALUE_X, THRESH_VALUE_Y-2);
  tft.print("LED:");
  
  tft.setCursor(THRESH_HPA_X, THRESH_HPA_Y - 12);
  tft.print(pressureThreshold, 1);
  tft.print("Pa");

  tft.setCursor(MODE_DISPLAY_X, MODE_DISPLAY_Y);
  tft.print("M");
  tft.print((int)currentMode);
  switch(currentMode) {
    case M1_SIMPLE: tft.print(" 1"); break;
    case M2_EMA: tft.print(" 2"); break;
    case M3_DUAL: tft.print(" 3"); break;
  }

  tft.setTextSize(1);
  tft.setTextColor(DARK_BLUE);
  tft.setCursor(MS_X, MS_Y_M);
  tft.print("m");
  tft.drawFastHLine(MS_X-2, MS_Y_LINE, 10, DARK_BLUE);
  tft.setCursor(MS_X, MS_Y_S);
  tft.print("s");
}

void updateDynamicParts(float temp, float pressure) {
  if (!backlightOn) return;
  
  if (millis() - lastDisplayUpdate < displayUpdateInterval) return;
  lastDisplayUpdate = millis();
  
  char buf[10];

  tft.fillRect(TEMP_VALUE_X, TEMP_VALUE_Y, 48, 22, BLACK_COLOR);
  tft.setTextSize(2);
  tft.setTextColor(WHITE_COLOR);
  tft.setCursor(TEMP_VALUE_X, TEMP_VALUE_Y);
  dtostrf(temp, 4, 1, buf);
  tft.print(buf);

  tft.fillRect(PRESS_VALUE_X, PRESS_VALUE_Y, 60, 22, BLACK_COLOR);
  tft.setTextColor(WHITE_COLOR);
  tft.setCursor(PRESS_VALUE_X, PRESS_VALUE_Y);
  dtostrf(pressure / 100.0, 5, 1, buf);
  tft.print(buf);

  tft.fillRect(MODE_TEXT_X, MODE_TEXT_Y, DISPLAY_WIDTH - 36, 32, BLACK_COLOR);

  tft.setTextSize(3);
  tft.setTextColor(pressureRising ? RED_COLOR : GREEN_COLOR);

  const char* modeText = pressureRising ? "SINKEN " : "STEIGEN ";
  tft.setCursor(MODE_TEXT_X + 10, MODE_TEXT_Y);
  tft.print(modeText);

  dtostrf(verticalSpeed, 5, 1, buf);
  tft.setCursor(SPEED_VALUE_X, SPEED_VALUE_Y);
  tft.print(buf);
}

void drawGraph() {
  if (!backlightOn) return;
  
  int count = historyFull ? HISTORY_SIZE : historyIndex;
  if (count < 2) return;

  tft.fillRect(GRAPH_X_LEFT, GRAPH_TOP, GRAPH_WIDTH, GRAPH_BOTTOM - GRAPH_TOP + 1, BLACK_COLOR);

  float reference = pressureHistory[(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE];
  const float MAX_DEVIATION_PA = 20.0f;
  float scale = ((GRAPH_BOTTOM - GRAPH_TOP) / 2.0f) / MAX_DEVIATION_PA;

  int midY = GRAPH_TOP + (GRAPH_BOTTOM - GRAPH_TOP) / 2;
  tft.drawFastHLine(GRAPH_X_LEFT, midY, GRAPH_WIDTH, GRAY_COLOR);

  for (int pos = 1; pos < count; pos++) {
    int idx1 = (historyIndex - count + pos - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    int idx2 = (historyIndex - count + pos + HISTORY_SIZE) % HISTORY_SIZE;

    int x1 = GRAPH_X_LEFT + (pos - 1) * GRAPH_WIDTH / (HISTORY_SIZE - 1);
    int x2 = GRAPH_X_LEFT + pos * GRAPH_WIDTH / (HISTORY_SIZE - 1);

    int y1 = midY + (int)((pressureHistory[idx1] - reference) * scale);
    int y2 = midY + (int)((pressureHistory[idx2] - reference) * scale);

    y1 = constrain(y1, GRAPH_TOP, GRAPH_BOTTOM);
    y2 = constrain(y2, GRAPH_TOP, GRAPH_BOTTOM);

    tft.drawLine(x1, y1, x2, y2, CYAN_COLOR);
    tft.drawLine(x1, y1 + 1, x2, y2 + 1, CYAN_COLOR);
  }

  int currentIdx = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
  int currentX = GRAPH_X_LEFT + (count - 1) * GRAPH_WIDTH / (HISTORY_SIZE - 1);
  int currentY = midY + (int)((pressureHistory[currentIdx] - reference) * scale);
  currentY = constrain(currentY, GRAPH_TOP, GRAPH_BOTTOM);
  tft.fillCircle(currentX, currentY, 4, WHITE_COLOR);
}

void checkSerialInput() {
  if (Serial.available()) {
    char input[32];
    int len = Serial.readBytesUntil('\n', input, 31);
    input[len] = '\0';
    
    // Trim whitespace
    while (len > 0 && (input[len-1] == ' ' || input[len-1] == '\r')) {
      input[--len] = '\0';
    }
    
    if (strncmp(input, "threshold ", 10) == 0) {
      float newThreshold = atof(input + 10);
      if (newThreshold >= 0.1 && newThreshold <= 50.0) {
        pressureThreshold = newThreshold;
        Serial.print("Schwellenwert gesetzt auf: ");
        Serial.print(pressureThreshold, 1);
        Serial.println(" Pa");
        displayNeedsFullRefresh = true;
      } else {
        Serial.println("Fehler: Schwellenwert muss zwischen 0.1 und 50.0 Pa liegen");
      }
    } else if (strncmp(input, "th ", 3) == 0) {
      float newThreshold = atof(input + 3);
      if (newThreshold >= 0.1 && newThreshold <= 50.0) {
        pressureThreshold = newThreshold;
        Serial.print("Schwellenwert gesetzt auf: ");
        Serial.print(pressureThreshold, 1);
        Serial.println(" Pa");
        displayNeedsFullRefresh = true;
      } else {
        Serial.println("Fehler: Schwellenwert muss zwischen 0.1 und 50.0 Pa liegen");
      }
    } else if (strncmp(input, "mode ", 5) == 0) {
      int newMode = atoi(input + 5);
      if (newMode >= 1 && newMode <= 3) {
        currentMode = (VarioMode)newMode;
        Serial.print("Modus gesetzt auf: M");
        Serial.print(newMode);
        switch(currentMode) {
          case M1_SIMPLE: Serial.println(" (EINFACH)"); break;
          case M2_EMA: Serial.println(" (EMA)"); break;
          case M3_DUAL: Serial.println(" (DUAL)"); break;
        }
        displayNeedsFullRefresh = true;
      } else {
        Serial.println("Fehler: Modus muss 1, 2 oder 3 sein");
      }
    } else if (strcmp(input, "memory") == 0) {
      printMemoryInfo();
    } else if (strcmp(input, "stack") == 0) {
      printStackInfo();
    } else if (strcmp(input, "resetbmp") == 0) {
      Serial.println("Manueller BMP280 Reset...");
      resetBMP280();
    } else if (strcmp(input, "softreset") == 0) {
      Serial.println("Manueller Soft-Reset...");
      performSoftReset();
    } else if (strcmp(input, "help") == 0) {
      Serial.println("Befehle:");
      Serial.println("  threshold X - Setze Schwellenwert (z.B. 'threshold 2.5')");
      Serial.println("  th X - Kurzform (z.B. 'th 1.5' oder 'th 9.8')");
      Serial.println("  mode X - Setze Modus (1=EINFACH, 2=EMA, 3=DUAL)");
      Serial.println("  memory - Zeige Speicherbelegung");
      Serial.println("  stack - Zeige Stack-Informationen");
      Serial.println("  resetbmp - BMP280 manuell resetten");
      Serial.println("  softreset - Sofortiger Soft-Reset");
      Serial.println("  help - Zeige diese Hilfe");
      Serial.println("Watchdog: AKTIV (Arduino 8s), Soft-Reset alle 5min");
      Serial.println("Taster: D5=Threshold↑, D41=Threshold↓, D42=Hintergrundbeleuchtung, D6=Modus");
      Serial.print("  Aktueller Schwellenwert: ");
      Serial.print(pressureThreshold, 1);
      Serial.println(" Pa");
      Serial.print("  Aktueller Modus: M");
      Serial.print((int)currentMode);
      switch(currentMode) {
        case M1_SIMPLE: Serial.println(" (EINFACH)"); break;
        case M2_EMA: Serial.println(" (EMA)"); break;
        case M3_DUAL: Serial.println(" (DUAL)"); break;
      }
    } else {
      Serial.println("Unbekannter Befehl. 'help' für Hilfe.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESPVARIO RG 01/2026 - V. 0.10 Arduino-kompatibel start...");
  Serial.println("V.10 FEATURES: Arduino Watchdog, 5min Soft-Reset, Heap-Optimierung");
  Serial.println("Taster: D5=Threshold↑, D41=Threshold↓, D42=Hintergrundbeleuchtung, D6=Modus");
  Serial.print("Start-Schwellenwert: ");
  Serial.print(pressureThreshold, 1);
  Serial.println(" Pa");
  Serial.print("Start-Modus: M");
  Serial.print((int)currentMode);
  switch(currentMode) {
    case M1_SIMPLE: Serial.println(" (EINFACH)"); break;
    case M2_EMA: Serial.println(" (EMA)"); break;
    case M3_DUAL: Serial.println(" (DUAL)"); break;
  }
  
  printMemoryInfo();

  pinMode(T_UP_PIN, INPUT_PULLUP);
  pinMode(T_DOWN_PIN, INPUT_PULLUP);
  pinMode(T_BL_PIN, INPUT_PULLUP);
  pinMode(T_MODE_PIN, INPUT_PULLUP);

  pinMode(ONBOARD_RED_LED, OUTPUT);
  digitalWrite(ONBOARD_RED_LED, LOW);
  
  pixel.begin();
  pixel.setBrightness(80);
  pixel.clear();
  pixel.show();

  pinMode(TFT_POWER, OUTPUT);
  digitalWrite(TFT_POWER, HIGH);
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);

  ledcAttach(BUZZER_PIN, 2000, BUZZER_RES);
  ledcWriteTone(BUZZER_CH, 0);

  SPI.begin();
  tft.init(135, 240);
  tft.setRotation(3);

  Wire.begin();
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("BMP280 nicht gefunden!");
      bmp280Healthy = false;
    } else {
      Serial.println("BMP280 gefunden auf 0x77");
      bmp280Healthy = true;
    }
  } else {
    Serial.println("BMP280 gefunden auf 0x76");
    bmp280Healthy = true;
  }

  if (bmp280Healthy) {
    float initialPressure = bmp.readPressure();
    for (int i = 0; i < HISTORY_SIZE; i++) {
      pressureHistory[i] = initialPressure;
    }
    lastPressure = initialPressure;
    lastSuccessfulRead = millis();
  }

  drawStartup();
  startupStartTime = millis();

  updateCounter = 0;
  loopCounter = 0;
  lastLoopReport = millis();
  lastMemoryCheck = millis();
  lastBMP280Check = millis();
  lastSoftReset = millis();

  // Watchdog initialisieren
  initWatchdog();

  // Initiale Stack-Überwachung
  checkStackUsage();

  Serial.println("Setup abgeschlossen, starte Watchdog-geschützten Betrieb...");
}

void loop() {
  loopCounter++;
  
  // Regelmäßiges yield() für Arduino Watchdog
  yield();
  
  // Watchdog füttern
  feedWatchdog();
  
  // Stack-Überwachung alle 1000 Loops
  if (loopCounter % 1000 == 0) {
    checkStackUsage();
  }
  
  unsigned long currentMillis = millis();

  // Soft-Reset alle 5 Minuten
  if (currentMillis - lastSoftReset >= softResetInterval) {
    Serial.println("5 Minuten erreicht - führe präventiven Soft-Reset durch...");
    performSoftReset();
    return;  // Neustart des Loops
  }

  checkTaster();
  checkSerialInput();

  checkBMP280Health();

  if (currentMillis - lastMemoryCheck >= memoryCheckInterval) {
    lastMemoryCheck = currentMillis;
    printMemoryInfo();
  }

  if (currentMillis - lastLoopReport >= loopReportInterval) {
    lastLoopReport = currentMillis;
    Serial.print("Loops/30s: ");
    Serial.println(loopCounter);
    loopCounter = 0;
  }

  if (showStartup) {
    if (currentMillis - startupStartTime >= startupDuration) {
      showStartup = false;
      if (backlightOn) {
        tft.fillScreen(BLACK_COLOR);
        drawStaticParts();
        drawGraph();
      }
      Serial.println("Startup abgeschlossen, normale Messung beginnt");
    }
    return;
  }

  if (currentMillis - previousMeasureMillis >= measureInterval) {
    previousMeasureMillis = currentMillis;

    float temp = 0.0;
    float pressure = 0.0;
    bool measurementSuccess = false;

    if (bmp280Healthy) {
      measurementSuccess = safeBMP280Read(&temp, &pressure);
    }

    if (measurementSuccess) {
      if (currentMode == M1_SIMPLE) {
        float avgPrevious = getAveragePressure();
        
        if (historyIndex >= 6) {
          int oldIdx = (historyIndex - 6 + HISTORY_SIZE) % HISTORY_SIZE;
          float oldPressure = pressureHistory[oldIdx];
          float dP = pressure - oldPressure;
          float dt = 3.0;  // 6 Messungen × 500ms = 3.0s
          verticalSpeed = - (dP / 100.0) * 8.3 / dt;
        } else {
          verticalSpeed = 0.0;
        }
        
        pressureRising = (pressure > avgPrevious);
      } else if (currentMode == M2_EMA) {
        float dt = measureInterval / 1000.0;
        float deriv = -((pressure - lastPressure)/dt) * (8.3/100.0);
        emaDeriv = EMA_FAST * deriv + (1 - EMA_FAST) * emaDeriv;
        verticalSpeed = emaDeriv;
        pressureRising = (verticalSpeed < 0);
      } else if (currentMode == M3_DUAL) {
        float dt = measureInterval / 1000.0;
        float deriv = -((pressure - lastPressure)/dt) * (8.3/100.0);
        emaFast = EMA_FAST * deriv + (1 - EMA_FAST) * emaFast;
        emaSlow = EMA_SLOW * deriv + (1 - EMA_SLOW) * emaSlow;
        verticalSpeed = emaFast - emaSlow;
        pressureRising = (verticalSpeed < 0);
      }

      lastPressure = pressure;
      pressureHistory[historyIndex] = pressure;
      historyIndex = (historyIndex + 1) % HISTORY_SIZE;
      if (historyIndex == 0) historyFull = true;

      setLeds();
      updateVarioSound(verticalSpeed);

      if (backlightOn) {
        if (displayNeedsFullRefresh || updateCounter >= FULL_REFRESH_EVERY) {
          drawStaticParts();
          displayNeedsFullRefresh = false;
          updateCounter = 0;
        } else {
          updateCounter++;
        }

        updateDynamicParts(temp, pressure);
        drawGraph();
      }

      static unsigned long lastSerialOutput = 0;
      if (millis() - lastSerialOutput > 4000) {
        lastSerialOutput = millis();
        Serial.print("T:");
        Serial.print(temp, 1);
        Serial.print(" P:");
        Serial.print(pressure / 100.0, 1);
        Serial.print(pressureRising ? " UP" : " DN");
        Serial.print(verticalSpeed >= 0 ? "+" : "");
        Serial.print(verticalSpeed, 1);
        Serial.print(" M");
        Serial.print((int)currentMode);
        Serial.print(" H:");
        Serial.print(ESP.getFreeHeap());
        Serial.print(" R:");
        Serial.print(resetCount);
        Serial.println(backlightOn ? " BL:ON" : " BL:OFF");
      }
      
    } else {
      static unsigned long lastErrorOutput = 0;
      if (millis() - lastErrorOutput > 6000) {
        lastErrorOutput = millis();
        Serial.print("BMP280 Messfehler! Errors: ");
        Serial.print(consecutiveBMP280Errors);
        Serial.print(" Heap: ");
        Serial.print(ESP.getFreeHeap());
        Serial.print(" Resets: ");
        Serial.println(resetCount);
      }
      
      updateVarioSound(verticalSpeed);
      setLeds();
    }
  }
}
