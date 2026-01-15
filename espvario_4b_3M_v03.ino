/*
 * ESPVARIO - Vario für Paragleiter mit ESP32
 * ===========================================
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
 * resetsettings - Einstellungen auf Default zurücksetzen
 * stats       - Reset-Statistik anzeigen
 * help        - Hilfe anzeigen
 * 
 * V.13.1 FEATURES:
 * ---------------
 * ✅ Einstellungen im nicht-flüchtigen Speicher (Preferences/NVS)
 * ✅ Reset-Statistik mit Uptime-Tracking
 * ✅ Gesamt-Reboots, längste Uptime, Durchschnitt
 * ✅ Muster-Erkennung (STABLE/UNSTABLE/CRITICAL)
 * ✅ Keine RTC needed - nutzt millis()
 * ✅ Einstellungen überleben Stromtrennung
 * ✅ ESP32-IDF Watchdog mit 8s Panic-Mode
 * ✅ Bei BMP-Fehler → sofortiger Reboot
 * ✅ Bewährte V11 Architektur
 * 
 * VERSION:
 * --------
 * V. 0.13.1 - Persistent Settings + Reset-Statistik
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
#include "esp_task_wdt.h"
#include <Preferences.h>

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
Preferences settings;

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
const unsigned long measureInterval = 250; // 4Hz = 250ms

int updateCounter = 0;
const int FULL_REFRESH_EVERY = 40;  // Alle 8 Messungen

bool showStartup = true;
bool showStatsOnStartup = false;
unsigned long startupStartTime = 0;
const unsigned long startupDuration = 4000;
const unsigned long statsDisplayDuration = 8000;  // 8 Sekunden Statistik anzeigen

// ================== WATCHDOG & RESET TIMING ==================
unsigned long lastMemoryCheck = 0;
const unsigned long memoryCheckInterval = 45000;  // Alle 45 Sekunden
unsigned long loopCounter = 0;
unsigned long lastLoopReport = 0;
const unsigned long loopReportInterval = 30000;   // Alle 30 Sekunden
int resetCount = 0;

// Uptime Saving für Crash-Recovery
unsigned long lastUptimeSave = 0;
const unsigned long uptimeSaveInterval = 30000;  // Alle 30 Sekunden Uptime speichern

// Stack-Monitoring
uint32_t stackMinFree = 0;

// I2C und Sensor Health
unsigned long lastBMP280Check = 0;
const unsigned long bmp280CheckInterval = 90000;  // Alle 90 Sekunden
int consecutiveBMP280Errors = 0;
const int maxBMP280Errors = 3;  // Bei 3 Fehlern → SOFORT REBOOT!
bool bmp280Healthy = true;
unsigned long lastSuccessfulRead = 0;

// Display Management
bool displayNeedsFullRefresh = true;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 500;  // Max alle 500ms

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
#define THRESH_VALUE_X   206
#define THRESH_VALUE_Y   10
#define THRESH_HPA_X     206
#define THRESH_HPA_Y     32

// Position für Modus-Anzeige
#define MODE_DISPLAY_X   206
#define MODE_DISPLAY_Y   38

// Taster-Entprellung
unsigned long lastUpPress = 0;
unsigned long lastDownPress = 0;
unsigned long lastBlPress = 0;
unsigned long lastModePress = 0;
const unsigned long debounceDelay = 200;

bool backlightOn = true;

// ================== RESET STATISTIK ==================
struct ResetStats {
  uint32_t totalReboots;        // Gesamtanzahl Reboots
  uint32_t lastUptimeSeconds;   // Uptime vor dem letzten Reset
  uint32_t maxUptimeSeconds;    // Längste Uptime ever
  uint32_t totalUptimeSeconds;  // Gesamte Laufzeit (akkumuliert)
  uint32_t bootTimestamp;       // Relativer Zeitstempel (millis())
  uint32_t currentUptimeSeconds;// Aktuelle Uptime (regelmäßig gespeichert)
} resetStats;

enum StabilityPattern {
  STABLE,     // Uptime > 1 Stunde
  UNSTABLE,   // Uptime < 5 Minuten
  CRITICAL    // Uptime < 30 Sekunden
};

// ================== SETTINGS MANAGEMENT ==================
void loadSettings() {
  Serial.println("Lade Einstellungen aus NVS...");
  
  // Settings Namespace öffnen (RW-Mode)
  settings.begin("vario", false);
  
  // Einstellungen laden mit Defaults
  pressureThreshold = settings.getFloat("threshold", 1.5);
  backlightOn = settings.getBool("backlight", true);
  currentMode = (VarioMode)settings.getUChar("mode", 2);
  
  // Reset-Statistik laden
  resetStats.totalReboots = settings.getUInt("reboots", 0);
  resetStats.lastUptimeSeconds = settings.getUInt("lastUptime", 0);
  resetStats.maxUptimeSeconds = settings.getUInt("maxUptime", 0);
  resetStats.totalUptimeSeconds = settings.getUInt("totalUptime", 0);
  resetStats.bootTimestamp = settings.getUInt("bootTime", 0);
  resetStats.currentUptimeSeconds = settings.getUInt("currentUptime", 0);
  
  Serial.print("Threshold geladen: ");
  Serial.print(pressureThreshold, 1);
  Serial.println(" Pa");
  Serial.print("Hintergrundbeleuchtung geladen: ");
  Serial.println(backlightOn ? "EIN" : "AUS");
  Serial.print("Modus geladen: M");
  Serial.print((int)currentMode);
  switch(currentMode) {
    case M1_SIMPLE: Serial.println(" (EINFACH)"); break;
    case M2_EMA: Serial.println(" (EMA)"); break;
    case M3_DUAL: Serial.println(" (DUAL)"); break;
  }
  
  Serial.print("Reset-Statistik: ");
  Serial.print(resetStats.totalReboots);
  Serial.println(" Reboots bisher");
  
  // Crash-Recovery: Gespeicherte Uptime vom letzten Boot auswerten
  if (resetStats.currentUptimeSeconds > 0) {
    Serial.print("Uptime vor dem letzten Boot: ");
    Serial.print(resetStats.currentUptimeSeconds);
    Serial.println("s (Crash-Recovery)");
    
    // Letzte Uptime aktualisieren (war vor dem Crash)
    resetStats.lastUptimeSeconds = resetStats.currentUptimeSeconds;
    settings.putUInt("lastUptime", resetStats.lastUptimeSeconds);
    
    // Maximale Uptime aktualisieren
    if (resetStats.currentUptimeSeconds > resetStats.maxUptimeSeconds) {
      resetStats.maxUptimeSeconds = resetStats.currentUptimeSeconds;
      settings.putUInt("maxUptime", resetStats.maxUptimeSeconds);
    }
    
    // Gesamte Uptime akkumulieren
    resetStats.totalUptimeSeconds += resetStats.currentUptimeSeconds;
    settings.putUInt("totalUptime", resetStats.totalUptimeSeconds);
    
    // Aktuelle Uptime zurücksetzen für neuen Boot
    resetStats.currentUptimeSeconds = 0;
    settings.putUInt("currentUptime", 0);
    
    Serial.println("Crash-Uptime wurde in Statistik übernommen");
  }
}

void saveThreshold() {
  settings.putFloat("threshold", pressureThreshold);
  Serial.print("Threshold gespeichert: ");
  Serial.print(pressureThreshold, 1);
  Serial.println(" Pa");
}

void saveBacklight() {
  settings.putBool("backlight", backlightOn);
  Serial.print("Hintergrundbeleuchtung gespeichert: ");
  Serial.println(backlightOn ? "EIN" : "AUS");
}

void saveMode() {
  settings.putUChar("mode", (uint8_t)currentMode);
  Serial.print("Modus gespeichert: M");
  Serial.print((int)currentMode);
  switch(currentMode) {
    case M1_SIMPLE: Serial.println(" (EINFACH)"); break;
    case M2_EMA: Serial.println(" (EMA)"); break;
    case M3_DUAL: Serial.println(" (DUAL)"); break;
  }
}

void updateResetStats() {
  // Reboot-Zähler aktualisieren (mit Modulo 10000)
  resetStats.totalReboots++;
  if (resetStats.totalReboots > 9999) {
    resetStats.totalReboots = 0;
  }
  
  // Statistik speichern
  settings.putUInt("reboots", resetStats.totalReboots);
  
  Serial.print("Reset-Statistik aktualisiert: ");
  Serial.print(resetStats.totalReboots);
  Serial.println(" Reboots");
}

void saveCurrentUptime() {
  // Aktuelle Uptime regelmäßig speichern für Crash-Recovery
  resetStats.currentUptimeSeconds = millis() / 1000;
  settings.putUInt("currentUptime", resetStats.currentUptimeSeconds);
}

void saveUptimeOnShutdown() {
  unsigned long currentUptime = millis() / 1000;  // in Sekunden
  
  // Letzte Uptime speichern
  resetStats.lastUptimeSeconds = currentUptime;
  settings.putUInt("lastUptime", resetStats.lastUptimeSeconds);
  
  // Maximale Uptime aktualisieren
  if (currentUptime > resetStats.maxUptimeSeconds) {
    resetStats.maxUptimeSeconds = currentUptime;
    settings.putUInt("maxUptime", resetStats.maxUptimeSeconds);
  }
  
  // Gesamte Uptime akkumulieren
  resetStats.totalUptimeSeconds += currentUptime;
  settings.putUInt("totalUptime", resetStats.totalUptimeSeconds);
  
  Serial.print("Uptime gespeichert: ");
  Serial.print(currentUptime);
  Serial.println("s");
}

void resetSettings() {
  Serial.println("Setze alle Einstellungen auf Default zurück...");
  settings.clear();
  pressureThreshold = 1.5;
  backlightOn = true;
  currentMode = M2_EMA;
  resetStats.totalReboots = 0;
  resetStats.lastUptimeSeconds = 0;
  resetStats.maxUptimeSeconds = 0;
  resetStats.totalUptimeSeconds = 0;
  resetStats.bootTimestamp = millis();
  resetStats.currentUptimeSeconds = 0;
  // Sofort speichern
  saveThreshold();
  saveBacklight();
  saveMode();
  updateResetStats();
  settings.putUInt("currentUptime", 0);  // Auch zurücksetzen
  displayNeedsFullRefresh = true;
  Serial.println("Einstellungen zurückgesetzt und gespeichert");
}

StabilityPattern getStabilityPattern() {
  if (resetStats.totalReboots == 0) return STABLE;
  
  uint32_t avgUptime = resetStats.totalUptimeSeconds / resetStats.totalReboots;
  
  if (avgUptime < 30) return CRITICAL;
  if (avgUptime < 300) return UNSTABLE;  // 5 Minuten
  return STABLE;
}

void formatTime(uint32_t seconds, char* buffer, size_t bufferSize) {
  uint32_t days = seconds / 86400;
  uint32_t hours = (seconds % 86400) / 3600;
  uint32_t minutes = (seconds % 3600) / 60;
  uint32_t secs = seconds % 60;
  
  if (days > 0) {
    snprintf(buffer, bufferSize, "%dd %dh %dm", days, hours, minutes);
  } else if (hours > 0) {
    snprintf(buffer, bufferSize, "%dh %dm %ds", hours, minutes, secs);
  } else {
    snprintf(buffer, bufferSize, "%dm %ds", minutes, secs);
  }
}

void printResetStats() {
  Serial.println("=== RESET STATISTIK ===");
  Serial.print("Gesamt-Reboots:   ");
  Serial.println(resetStats.totalReboots);
  
  char timeBuffer[32];
  formatTime(resetStats.lastUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  Serial.print("Letzte Uptime:    ");
  Serial.println(timeBuffer);
  
  formatTime(resetStats.maxUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  Serial.print("Längste Uptime:    ");
  Serial.println(timeBuffer);
  
  formatTime(resetStats.totalUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  Serial.print("Gesamt-Laufzeit:  ");
  Serial.println(timeBuffer);
  
  if (resetStats.totalReboots > 0) {
    uint32_t avgUptime = resetStats.totalUptimeSeconds / resetStats.totalReboots;
    formatTime(avgUptime, timeBuffer, sizeof(timeBuffer));
    Serial.print("Durchschnitt:      ");
    Serial.println(timeBuffer);
  }
  
  Serial.print("Stabilitäts-Pattern: ");
  switch(getStabilityPattern()) {
    case STABLE: Serial.println("STABLE (gute Laufzeiten)"); break;
    case UNSTABLE: Serial.println("UNSTABLE (häufige Resets)"); break;
    case CRITICAL: Serial.println("CRITICAL (sehr kurze Laufzeiten)"); break;
  }
  
  Serial.print("NVS genutzt: ");
  Serial.print(settings.getBytesLength("/nvs/vario/"));
  Serial.println(" Bytes");
  Serial.println("========================");
}

// ================== WATCHDOG & RESET FUNCTIONS ==================
void initWatchdog() {
  // ESP32-IDF Watchdog mit 8 Sekunden Timeout (ESP-IDF v5.x API)
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 8000,        // 8 Sekunden Timeout
    .idle_core_mask = 0,       // Beide Cores überwachen
    .trigger_panic = true      // Panic-Mode aktiv
  };
  
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);       // Aktuellen Task zum Watchdog hinzufügen
  Serial.println("ESP32-IDF Watchdog initialisiert (8s, Panic-Mode)");
}

void feedWatchdog() {
  // ESP32-IDF Watchdog explizit füttern
  esp_task_wdt_reset();
}

void performHardReboot() {
  // Uptime vor Reset speichern!
  saveUptimeOnShutdown();
  
  // Reboot-Statistik aktualisieren
  updateResetStats();
  
  Serial.println("=== I2C HÄNGT → ESP.restart() ===");
  Serial.print("Fehlerteiler: ");
  Serial.println(consecutiveBMP280Errors);
  Serial.print("Reboot-Counter: ");
  Serial.println(++resetCount);
  Serial.println("In der Luft ist ein Neustart 100× besser als Zombie-Vario");
  
  delay(100);  // Kurze Verzögerung für Serial-Ausgabe
  ESP.restart();  // HARTER REBOOT - kein Zurück mehr!
}

// ================== STACK MONITORING ==================
void checkStackUsage() {
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
  const unsigned long i2cTimeout = 150;  // 150ms Timeout
  
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
  
  // Bei 3 aufeinanderfolgenden Fehlern → SOFORT REBOOT!
  if (consecutiveBMP280Errors >= maxBMP280Errors) {
    Serial.println("I2C hängt → ESP.restart()");
    delay(100);
    ESP.restart();
  }
  
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
  Serial.println("AKTIV (ESP32-IDF 8s Panic)");
  
  Serial.print("Resets: ");
  Serial.println(resetCount);
  
  // Settings Info
  Serial.print("Settings Namespace: ");
  Serial.println(settings.isKey("threshold") ? "GELADEN" : "DEFAULT");
  
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
  
  // SPEICHERN!
  saveThreshold();
  
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
    
    // SPEICHERN!
    saveBacklight();
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
    
    // SPEICHERN!
    saveMode();
    
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
  tft.println("V. 0.13.1");
}

void drawStatsOnDisplay() {
  if (!backlightOn) return;
  
  tft.fillScreen(BLACK_COLOR);
  tft.drawRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, BLUE_COLOR);

  tft.setTextColor(WHITE_COLOR);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("RESET-STATISTIK");

  tft.setTextSize(1);
  tft.setTextColor(CYAN_COLOR);
  
  char timeBuffer[20];
  
  // Gesamt-Reboots
  tft.setCursor(10, 35);
  tft.print("Reboots: ");
  tft.setTextColor(WHITE_COLOR);
  tft.print(resetStats.totalReboots);
  
  // Letzte Uptime
  formatTime(resetStats.lastUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  tft.setCursor(10, 50);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Letzte: ");
  tft.setTextColor(WHITE_COLOR);
  tft.println(timeBuffer);
  
  // Längste Uptime
  formatTime(resetStats.maxUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  tft.setCursor(10, 65);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Max: ");
  tft.setTextColor(WHITE_COLOR);
  tft.println(timeBuffer);
  
  // Gesamt-Laufzeit
  formatTime(resetStats.totalUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  tft.setCursor(10, 80);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Gesamt: ");
  tft.setTextColor(WHITE_COLOR);
  tft.println(timeBuffer);
  
  // Durchschnitt
  if (resetStats.totalReboots > 0) {
    uint32_t avgUptime = resetStats.totalUptimeSeconds / resetStats.totalReboots;
    formatTime(avgUptime, timeBuffer, sizeof(timeBuffer));
    tft.setCursor(10, 95);
    tft.setTextColor(CYAN_COLOR);
    tft.print("Avg: ");
    tft.setTextColor(WHITE_COLOR);
    tft.println(timeBuffer);
  }
  
  // Stabilitäts-Pattern
  tft.setCursor(10, 115);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Pattern: ");
  switch(getStabilityPattern()) {
    case STABLE: 
      tft.setTextColor(GREEN_COLOR);
      tft.println("STABLE");
      break;
    case UNSTABLE: 
      tft.setTextColor(0xFFE0); // Gelb
      tft.println("UNSTABLE");
      break;
    case CRITICAL: 
      tft.setTextColor(RED_COLOR);
      tft.println("CRITICAL");
      break;
  }
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
  
  tft.setCursor(THRESH_HPA_X, THRESH_HPA_Y - 10);
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
        saveThreshold();  // SPEICHERN!
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
        saveThreshold();  // SPEICHERN!
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
        saveMode();  // SPEICHERN!
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
    } else if (strcmp(input, "resetsettings") == 0) {
      Serial.println("Einstellungen zurücksetzen...");
      resetSettings();
    } else if (strcmp(input, "stats") == 0) {
      Serial.println();
      printResetStats();
      Serial.println();
    } else if (strcmp(input, "help") == 0) {
      Serial.println("Befehle:");
      Serial.println("  threshold X - Setze Schwellenwert (z.B. 'threshold 2.5')");
      Serial.println("  th X - Kurzform (z.B. 'th 1.5' oder 'th 9.8')");
      Serial.println("  mode X - Setze Modus (1=EINFACH, 2=EMA, 3=DUAL)");
      Serial.println("  memory - Zeige Speicherbelegung");
      Serial.println("  stack - Zeige Stack-Informationen");
      Serial.println("  resetbmp - BMP280 manuell resetten");
      Serial.println("  resetsettings - Einstellungen auf Default zurücksetzen");
      Serial.println("  stats - Zeige Reset-Statistik");
      Serial.println("  help - Zeige diese Hilfe");
      Serial.println("Watchdog: ESP32-IDF 8s Panic-Mode, Bei BMP-Fehler → REBOOT");
      Serial.println("Settings: Persistent (überleben Stromtrennung)");
      Serial.println("Statistik: Reset-Tracking mit Uptime-Analyse + Crash-Recovery");
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
      Serial.print("  Hintergrundbeleuchtung: ");
      Serial.println(backlightOn ? "EIN" : "AUS");
      Serial.print("  Gesamt-Reboots: ");
      Serial.println(resetStats.totalReboots);
    } else {
      Serial.println("Unbekannter Befehl. 'help' für Hilfe.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESPVARIO RG 01/2026 - V. 0.13.1 start...");
  Serial.println("V.13.1 FEATURES: Persistent Settings + Reset-Statistik");
  Serial.println("Einstellungen und Statistik überleben Stromtrennung!");
  Serial.println("Statistik wird nach Kalt-Boot angezeigt (8 Sekunden)");
  Serial.println("Crash-Recovery: Uptime wird alle 30s für Crash-Erkennung gespeichert");
  Serial.println("ESP32-IDF Watchdog mit 8s Panic-Mode (v5.x API)");
  Serial.println("Bei BMP-Fehler → sofortiger Reboot");
  Serial.println("Taster: D5=Threshold↑, D41=Threshold↓, D42=Hintergrundbeleuchtung, D6=Modus");
  
  // ERST: Settings laden!
  loadSettings();
  
  // Boot-Zeitstempel setzen
  resetStats.bootTimestamp = millis();
  settings.putUInt("bootTime", resetStats.bootTimestamp);
  
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
  digitalWrite(TFT_BACKLIGHT, backlightOn ? HIGH : LOW);  // Geladene Einstellung!

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
  
  // Statistik nach Kalt-Boot im Serial ausgeben
  Serial.println();
  Serial.println("=== KALT-BOOT STATISTIK ===");
  printResetStats();
  Serial.println("==========================");
  Serial.println();

  updateCounter = 0;
  loopCounter = 0;
  lastLoopReport = millis();
  lastMemoryCheck = millis();
  lastBMP280Check = millis();
  lastUptimeSave = millis();  // Uptime-Saving Timer starten

  // ESP32-IDF Watchdog initialisieren
  initWatchdog();

  // Initiale Stack-Überwachung
  checkStackUsage();

  Serial.println("Setup abgeschlossen, starte Watchdog-geschützten Betrieb...");
  Serial.println("Einstellungen und Statistik sind persistent und überleben Stromtrennung!");
}

void loop() {
  loopCounter++;
  
  // ESP32-IDF Watchdog explizit füttern
  esp_task_wdt_reset();
  
  // Regelmäßiges yield() für Arduino Watchdog
  yield();
  
  // Stack-Überwachung alle 1000 Loops
  if (loopCounter % 1000 == 0) {
    checkStackUsage();
  }
  
  unsigned long currentMillis = millis();

  checkTaster();
  checkSerialInput();

  checkBMP280Health();

  // Alle 30 Sekunden aktuelle Uptime speichern (Crash-Recovery)
  if (currentMillis - lastUptimeSave >= uptimeSaveInterval) {
    lastUptimeSave = currentMillis;
    saveCurrentUptime();
  }

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
      showStatsOnStartup = true;  // Statistik anzeigen starten
      drawStatsOnDisplay();
      Serial.println("Startup abgeschlossen, zeige Statistik...");
    }
    return;
  }
  
  // Statistik für einige Sekunden anzeigen
  if (showStatsOnStartup) {
    if (currentMillis - startupStartTime >= startupDuration + statsDisplayDuration) {
      showStatsOnStartup = false;
      if (backlightOn) {
        tft.fillScreen(BLACK_COLOR);
        drawStaticParts();
        drawGraph();
      }
      Serial.println("Statistik-Anzeige beendet, normale Messung beginnt");
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
        Serial.print(" S:");
        Serial.print(settings.isKey("threshold") ? "✓" : "D");
        Serial.print(" RB:");
        Serial.print(resetStats.totalReboots);
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
