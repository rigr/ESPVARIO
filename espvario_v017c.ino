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
 * T_UP_PIN       5   - Taster für Threshold erhöhen
 * T_DOWN_PIN    41   - Taster für Threshold senken
 * T_BL_PIN      42   - Taster für Hintergrundbeleuchtung
 * T_MODE_PIN     6   - Taster für Moduswechsel
 * BOOT_PIN       0   - BOOT-Taste für Menü
 * BUZZER_PIN    25   - Summer für Vario-Töne
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
 * BOOT-TASTEN MENÜ V.17c:
 * -----------------------
 * 1) BOOT kurz drücken → Menü startet mit 3-Sekunden-Timer
 * 1a) "Display off" → 3-2-1 Countdown (rot/weiß)
 *     - Bei BOOT-Click: Display aus/an, Timer zurücksetzen
 * 1b) "Threshold" → Single/Double-Click Erhöhen/Senken
 *     - Single-Click: Wert erhöhen, Timer zurücksetzen
 *     - Double-Click: Wert senken, Timer zurücksetzen
 * 1c) "Modus" → M1/M2/M3 Anzeige
 *     - Bei BOOT-Click: Nächster Modus (M1→M2→M3→M1), Timer zurücksetzen
 * 1d) "Power off" → 3-2-1 Countdown
 *     - Bei BOOT-Click: Graceful Shutdown
 * 
 * TIMER-VERHALTEN:
 * ----------------
 * - Jeder Menüpunkt dauert 3 Sekunden
 * - BOOT-Taste drücken → Funktion ausführen + Timer von vorne beginnen
 * - Nach 3 Sekunden → Automatisch zum nächsten Menüpunkt
 * 
 * GRACEFUL SHUTDOWN (alternativ):
 * ------------------------------
 * - Display AUS + THRESHOLD UP/DOWN gleichzeitig > 2s
 * - Display zeigt "SAFE TO UNPLUG" und wartet auf Stromtrennung
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
 * shutdown    - Manuelles Graceful Shutdown
 * help        - Hilfe anzeigen
 * 
 * VERSION:
 * --------
 * V. 0.17c - Simple While-Loop Menu with BOOT Button Detection
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
#define BOOT_PIN       0   // BOOT-Taste für Menü
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

// Stack-Monitoring
uint32_t stackMinFree = 0;

// I2C und Sensor Health
unsigned long lastBMP280Check = 0;
const unsigned long bmp280CheckInterval = 90000;  // Alle 90 Sekunden
int consecutiveBMP280Errors = 0;
const int maxBMP280Errors = 3;  // Bei 3 Fehlern → SOFORT REBOOT!
bool bmp280Healthy = true;
unsigned long lastSuccessfulRead = 0;

// Uptime Saving für Crash-Recovery
unsigned long lastUptimeSave = 0;
const unsigned long uptimeSaveInterval = 30000;  // Alle 30 Sekunden Uptime speichern

// Memory Guard
unsigned long lastMemoryGuard = 0;
const unsigned long memoryGuardInterval = 60000;  // Alle 60 Sekunden Heap prüfen
const uint32_t criticalHeapThreshold = 300000;   // 300KB kritisch

// Display Management
bool displayNeedsFullRefresh = true;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 500;  // Max alle 500ms

// ================== BOOT MENU SYSTEM V.17c ==================
enum BootMenuState {
  MENU_INACTIVE,
  MENU_DISPLAY_OFF,
  MENU_THRESHOLD,
  MENU_MODE,
  MENU_POWER_OFF
};

BootMenuState bootMenuState = MENU_INACTIVE;
bool bootMenuActive = false;

// ================== GRACEFUL SHUTDOWN ==================
unsigned long shutdownStartTime = 0;
const unsigned long shutdownTimeout = 2000;  // 2 Sekunden halten
bool shutdownMode = false;

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
#define THRESH_VALUE_X   204
#define THRESH_VALUE_Y   10
#define THRESH_HPA_X     204
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
  uint32_t totalBoots;         // Gesamtanzahl Boots (alle Starts)
  uint32_t totalCrashes;        // Anzahl Crashes (ungeplante Reboots)
  uint32_t gracefulShutdowns;   // Anzahl saubere Ausschaltungen
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
  backlightOn = true;
  currentMode = (VarioMode)settings.getUChar("mode", 2);
  
  // Reset-Statistik laden
  resetStats.totalBoots = settings.getUInt("boots", 0);
  resetStats.totalCrashes = settings.getUInt("crashes", 0);
  resetStats.gracefulShutdowns = settings.getUInt("graceful", 0);
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
  
  Serial.print("Boot-Statistik: ");
  Serial.print(resetStats.totalBoots);
  Serial.print(" Boots, ");
  Serial.print(resetStats.totalCrashes);
  Serial.print(" Crashes, ");
  Serial.print(resetStats.gracefulShutdowns);
  Serial.println(" Graceful");
  
  // IMMER Boot-Zähler erhöhen
  resetStats.totalBoots++;
  if (resetStats.totalBoots > 9999) {
    resetStats.totalBoots = 1;  // Bei 10000 wieder bei 1 starten
  }
  settings.putUInt("boots", resetStats.totalBoots);
  
  // Smart Crash Detection: Nur kurze Laufzeiten als Crash werten
  if (resetStats.currentUptimeSeconds > 0) {
    if (resetStats.currentUptimeSeconds < 120) {  // < 2 Minuten = echter Crash
      Serial.print("KURZE Uptime vor Crash: ");
      Serial.print(resetStats.currentUptimeSeconds);
      Serial.println("s - WIRKLICHER CRASH!");
      
      resetStats.totalCrashes++;
      if (resetStats.totalCrashes > 9999) {
        resetStats.totalCrashes = 1;
      }
      settings.putUInt("crashes", resetStats.totalCrashes);
    } else {
      Serial.print("LANGE Uptime: ");
      Serial.print(resetStats.currentUptimeSeconds);
      Serial.println("s - Normales Ausschalten (kein Crash)");
    }
    
    // Uptime in Statistik übernehmen
    resetStats.totalUptimeSeconds += resetStats.currentUptimeSeconds;
    settings.putUInt("totalUptime", resetStats.totalUptimeSeconds);
    
    // Maximale Uptime aktualisieren
    if (resetStats.currentUptimeSeconds > resetStats.maxUptimeSeconds) {
      resetStats.maxUptimeSeconds = resetStats.currentUptimeSeconds;
      settings.putUInt("maxUptime", resetStats.maxUptimeSeconds);
    }
    
    // Aktuelle Uptime zurücksetzen für neuen Boot
    resetStats.currentUptimeSeconds = 0;
    settings.putUInt("currentUptime", 0);
  } else {
    Serial.println("Normaler Kalt-Boot (keine vorherige Uptime)");
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
  // Diese Funktion wird bei geplanten Reboots aufgerufen
  Serial.println("Geplanter Reboot - keine Crash-Zählung");
}

void saveCurrentUptime() {
  // Nur bei stabilem System speichern
  if (consecutiveBMP280Errors == 0 && ESP.getFreeHeap() > criticalHeapThreshold) {
    resetStats.currentUptimeSeconds = millis() / 1000;
    settings.putUInt("currentUptime", resetStats.currentUptimeSeconds);
  }
}

void resetSettings() {
  Serial.println("Setze alle Einstellungen auf Default zurück...");
  settings.clear();
  pressureThreshold = 1.5;
  backlightOn = true;
  currentMode = M2_EMA;
  resetStats.totalBoots = 0;
  resetStats.totalCrashes = 0;
  resetStats.gracefulShutdowns = 0;
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
  settings.putUInt("currentUptime", 0);
  displayNeedsFullRefresh = true;
  Serial.println("Einstellungen zurückgesetzt und gespeichert");
}

StabilityPattern getStabilityPattern() {
  if (resetStats.totalBoots == 0) return STABLE;
  
  uint32_t avgUptime = resetStats.totalUptimeSeconds / resetStats.totalBoots;
  
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
  Serial.println("=== RESET STATISTIK V.17c ===");
  Serial.print("Gesamt-Boots:       ");
  Serial.println(resetStats.totalBoots);
  Serial.print("Davon Crashes:      ");
  Serial.println(resetStats.totalCrashes);
  Serial.print("Davon Graceful:     ");
  Serial.println(resetStats.gracefulShutdowns);
  Serial.print("Unbekannt/Andere:   ");
  Serial.println(resetStats.totalBoots - resetStats.totalCrashes - resetStats.gracefulShutdowns);
  
  if (resetStats.totalBoots > 0) {
    float crashRate = (float)resetStats.totalCrashes / resetStats.totalBoots * 100.0;
    Serial.print("Crash-Rate:        ");
    Serial.print(crashRate, 1);
    Serial.println("%");
  }
  
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
  
  if (resetStats.totalBoots > 0) {
    uint32_t avgUptime = resetStats.totalUptimeSeconds / resetStats.totalBoots;
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
  Serial.println("===============================");
}

// ================== BOOT MENU SYSTEM V.17c ==================
void drawBootMenu(BootMenuState state, int countdown) {
  tft.fillScreen(BLACK_COLOR);
  tft.drawRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, BLUE_COLOR);
  
  // Titel basierend auf Menüzustand
  tft.setTextColor(WHITE_COLOR);
  tft.setTextSize(3);
  
  const char* title = "";
  switch(state) {
    case MENU_DISPLAY_OFF:
      title = "Display off";
      break;
    case MENU_THRESHOLD:
      title = "Threshold";
      break;
    case MENU_MODE:
      title = "Modus";
      break;
    case MENU_POWER_OFF:
      title = "Power off";
      break;
    default:
      title = "Menu";
      break;
  }
  
  int titleWidth = strlen(title) * 18;  // Ungefähre Breite
  int titleX = (DISPLAY_WIDTH - titleWidth) / 2;
  tft.setCursor(titleX, 20);
  tft.println(title);
  
  // Zusatztext je nach Menüpunkt
  tft.setTextSize(1);
  tft.setTextColor(CYAN_COLOR);
  
  if (state == MENU_THRESHOLD) {
    tft.setCursor(10, 48);
    tft.print("Aktuell: ");
    tft.print(pressureThreshold, 1);
    tft.println(" Pa");
    tft.setCursor(10, 60);
    tft.println("BOOT: Wert aendern");
  } else if (state == MENU_MODE) {
    // Aktuellen Modus anzeigen
    tft.setCursor(10, 48);
    tft.print("Aktuell: M");
    tft.print((int)currentMode);
    switch(currentMode) {
      case M1_SIMPLE: tft.println(" (EINFACH)"); break;
      case M2_EMA: tft.println(" (EMA)"); break;
      case M3_DUAL: tft.println(" (DUAL)"); break;
    }
    
    tft.setCursor(10, 60);
    tft.println("BOOT: Naechster Modus");
  } else if (state == MENU_DISPLAY_OFF) {
    tft.setCursor(10, 60);
    tft.println("BOOT: Display umschalten");
  } else if (state == MENU_POWER_OFF) {
    tft.setCursor(10, 60);
    tft.println("BOOT: Herunterfahren");
  }
  
  // Countdown Anzeige
  tft.setTextSize(4);  // war 6
  int y = 76;          // war 95
  for (int i = 3; i >= 1; i--) {
    int x = 52 + (3 - i) * 60;       // x war 30, jetzt mittiger
    char numStr[2];
    sprintf(numStr, "%d", i);
    
    if (i == countdown) {
      tft.setTextColor(RED_COLOR);
      // Rahmen um aktuelle Zahl
      tft.drawRect(x-6, y-5, 32, 40, RED_COLOR);
    } else {
      tft.setTextColor(WHITE_COLOR);
    }
    
    tft.setCursor(x, y);
    tft.println(numStr);
  }
  
  // Progress Bar
  int progress = ((3 - countdown) * 100) / 3;
  if (progress > 100) progress = 100;
  
  tft.drawRect(10, DISPLAY_HEIGHT - 15, DISPLAY_WIDTH - 20, 8, GRAY_COLOR);
  if (progress > 0) {
    tft.fillRect(12, DISPLAY_HEIGHT - 13, (DISPLAY_WIDTH - 24) * progress / 100, 4, GREEN_COLOR);
  }
}

void runBootMenu() {
  Serial.println("=== BOOT MENU V.17c START ===");
  
  // Display für Menü aktivieren
  digitalWrite(TFT_BACKLIGHT, HIGH);
  backlightOn = true;
  
  BootMenuState currentState = MENU_DISPLAY_OFF;
  
  while (true) {
    // Menüpunkt anzeigen
    for (int countdown = 3; countdown >= 1; countdown--) {
      drawBootMenu(currentState, countdown);
      
      // 1 Sekunde warten und BOOT-Taste prüfen
      unsigned long startTime = millis();
      while (millis() - startTime < 1000) {
        // Watchdog füttern
        esp_task_wdt_reset();
        yield();
        
        // BOOT-Taste prüfen
        if (digitalRead(BOOT_PIN) == LOW) {
          // Entprellen
          delay(50);
          if (digitalRead(BOOT_PIN) == LOW) {
            Serial.print("BOOT gedrückt bei State: ");
            Serial.println(currentState);
            
            // Aktion ausführen
            bool menuExit = false;
            
            switch(currentState) {
              case MENU_DISPLAY_OFF:
                backlightOn = !backlightOn;
                digitalWrite(TFT_BACKLIGHT, backlightOn ? HIGH : LOW);
                Serial.println(backlightOn ? "Display EIN" : "Display AUS");
                saveBacklight();
                menuExit = true;
                break;
                
              case MENU_THRESHOLD:
                // Einfach: immer erhöhen
                adjustThreshold(true);
                Serial.print("Threshold erhöht: ");
                Serial.println(pressureThreshold, 1);
                // Im Menü bleiben
                break;
                
              case MENU_MODE:
                // Nächsten Modus (zyklisch)
                if (currentMode == M1_SIMPLE) {
                  currentMode = M2_EMA;
                } else if (currentMode == M2_EMA) {
                  currentMode = M3_DUAL;
                } else {
                  currentMode = M1_SIMPLE;
                }
                Serial.print("Modus gewechselt: M");
                Serial.print((int)currentMode);
                switch(currentMode) {
                  case M1_SIMPLE: Serial.println(" (EINFACH)"); break;
                  case M2_EMA: Serial.println(" (EMA)"); break;
                  case M3_DUAL: Serial.println(" (DUAL)"); break;
                }
                saveMode();
                // Im Menü bleiben
                break;
                
              case MENU_POWER_OFF:
                menuExit = true;
                // Nach Schleife Graceful Shutdown
                break;
            }
            
            // Auf Taste loslassen warten
            while (digitalRead(BOOT_PIN) == LOW) {
              delay(10);
              esp_task_wdt_reset();
            }
            
            if (menuExit) {
              if (currentState == MENU_POWER_OFF) {
                // Graceful Shutdown
                Serial.println("Graceful Shutdown aus Menü");
                performGracefulShutdown();
                return; // Wird nie erreicht
              } else {
                // Menü beenden
                Serial.println("=== BOOT MENU BEENDET ===");
                displayNeedsFullRefresh = true;
                return;
              }
            } else {
              // Im selben Menüpunkt bleiben, Countdown neu starten
              countdown = 4; // Wird auf 3 reduziert in for-Schleife
              break; // Countdown-Schleife neu starten
            }
          }
        }
      }
    }
    
    // Zum nächsten Menüpunkt wechseln
    Serial.print("Timeout → ");
    if (currentState == MENU_DISPLAY_OFF) {
      Serial.println("Threshold");
      currentState = MENU_THRESHOLD;
    } else if (currentState == MENU_THRESHOLD) {
      Serial.println("Modus");
      currentState = MENU_MODE;
    } else if (currentState == MENU_MODE) {
      Serial.println("Power off");
      currentState = MENU_POWER_OFF;
    } else if (currentState == MENU_POWER_OFF) {
      Serial.println("Exit");
      break; // While-Schleife beenden
    }
  }
  
  Serial.println("=== BOOT MENU TIMEOUT BEENDET ===");
  displayNeedsFullRefresh = true;
}

// ================== GRACEFUL SHUTDOWN ==================
void checkGracefulShutdown() {
  // Prüfen: Display AUS und (TH UP ODER TH DOWN) gedrückt
  bool backlightOff = !backlightOn;
  bool upPressed = (digitalRead(T_UP_PIN) == LOW);
  bool downPressed = (digitalRead(T_DOWN_PIN) == LOW);
  bool thresholdPressed = upPressed || downPressed;
  
  if (backlightOff && thresholdPressed && !shutdownMode) {
    if (shutdownStartTime == 0) {
      shutdownStartTime = millis();
      Serial.println("Graceful Shutdown Timer gestartet...");
      
      // Kurze LED-Bestätigung (ohne delay!)
      digitalWrite(ONBOARD_RED_LED, HIGH);
    } else if (millis() - shutdownStartTime >= shutdownTimeout) {
      // 2 Sekunden erreicht - Graceful Shutdown
      performGracefulShutdown();
    }
  } else {
    // Bedingung nicht mehr erfüllt
    if (shutdownStartTime > 0) {
      Serial.println("Graceful Shutdown abgebrochen");
      digitalWrite(ONBOARD_RED_LED, LOW);  // LED wieder aus
    }
    shutdownStartTime = 0;
  }
}

void performGracefulShutdown() {
  Serial.println("=== GRACEFUL SHUTDOWN ===");
  Serial.println("Sicheres Ausschalten wird vorbereitet...");
  
  // Graceful Shutdown zählen
  resetStats.gracefulShutdowns++;
  settings.putUInt("graceful", resetStats.gracefulShutdowns);
  
  // Uptime-Zähler zurücksetzen (nicht als Crash werten)
  settings.putUInt("currentUptime", 0);
  
  // Display auf hell setzen für Shutdown-Nachricht
  digitalWrite(TFT_BACKLIGHT, HIGH);
  tft.fillScreen(BLACK_COLOR);
  tft.drawRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, GREEN_COLOR);
  
  // Shutdown-Nachricht
  tft.setTextColor(GREEN_COLOR);
  tft.setTextSize(3);
  tft.setCursor(25, DISPLAY_HEIGHT/2 - 30);
  tft.println("SAFE TO");
  tft.setCursor(35, DISPLAY_HEIGHT/2 + 5);
  tft.println("UNPLUG");
  
  tft.setTextSize(1);
  tft.setTextColor(WHITE_COLOR);
  tft.setCursor(15, DISPLAY_HEIGHT/2 + 40);
  tft.println("Gerät kann sicher vom Strom getrennt werden");
  
  // LED-Feedback
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));  // Grün
  pixel.show();
  
  // Warnton
  ledcWriteTone(BUZZER_CH, 800);
  delay(200);
  ledcWriteTone(BUZZER_CH, 0);
  delay(100);
  ledcWriteTone(BUZZER_CH, 800);
  delay(200);
  ledcWriteTone(BUZZER_CH, 0);
  
  Serial.println("Warte auf Stromtrennung...");
  
  // Endlosschleife - wartet auf Stromtrennung
  shutdownMode = true;
  while (true) {
    // Langsames Blinken zur Bestätigung
    digitalWrite(ONBOARD_RED_LED, HIGH);
    delay(1000);
    digitalWrite(ONBOARD_RED_LED, LOW);
    delay(1000);
    
    // Watchdog füttern
    esp_task_wdt_reset();
  }
}

// ================== WATCHDOG & RESET FUNCTIONS ==================
void initWatchdog() {
  // ESP32-IDF Watchdog mit 30 Sekunden Timeout (STABIL!)
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 30000,      // 30 Sekunden Timeout statt 8!
    .idle_core_mask = 0,      // Beide Cores überwachen
    .trigger_panic = false    // KEIN Panic-Mode, nur Reset!
  };
  
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);       // Aktuellen Task zum Watchdog hinzufügen
  Serial.println("ESP32-IDF Watchdog initialisiert (30s, No-Panic)");
}

void feedWatchdog() {
  // ESP32-IDF Watchdog explizit füttern
  esp_task_wdt_reset();
}

void performHardReboot() {
  // Uptime vor Reset speichern!
  saveCurrentUptime();
  
  // Reboot-Statistik aktualisieren
  updateResetStats();
  
  Serial.println("=== SYSTEM REBOOT ===");
  Serial.print("Fehlerteiler: ");
  Serial.println(consecutiveBMP280Errors);
  Serial.print("Reboot-Counter: ");
  Serial.println(++resetCount);
  Serial.println("Stabiler Reboot für maximale Zuverlässigkeit");
  
  delay(100);  // Kurze Verzögerung für Serial-Ausgabe
  ESP.restart();  // HARTER REBOOT
}

// ================== MEMORY GUARD ==================
void checkMemoryGuard() {
  size_t freeHeap = ESP.getFreeHeap();
  
  if (freeHeap < criticalHeapThreshold) {
    Serial.print("WARNUNG: Heap kritisch niedrig: ");
    Serial.print(freeHeap);
    Serial.println(" Bytes - Präventiver Reboot!");
    
    saveCurrentUptime();
    delay(100);
    ESP.restart();
  }
  
  // Memory Leaks erkennen
  if (ESP.getMinFreeHeap() < criticalHeapThreshold / 2) {
    Serial.print("WARNUNG: Min-Heap kritisch: ");
    Serial.print(ESP.getMinFreeHeap());
    Serial.println(" Bytes");
  }
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
  const int maxAttempts = 3;
  
  for (int attempt = 0; attempt < maxAttempts; attempt++) {
    unsigned long startTime = millis();
    const unsigned long i2cTimeout = 300;  // Längerer Timeout!
    
    yield();
    
    if (!bmp280Healthy) {
      return false;
    }
    
    // Bei wiederholten Fehlern I2C komplett resetten
    if (attempt > 0) {
      Serial.print("BMP280 Versuch ");
      Serial.print(attempt + 1);
      Serial.println(" - I2C Reset...");
      Wire.end();
      delay(100);
      Wire.begin();
      delay(100);
    }
    
    *temp = bmp.readTemperature();
    
    if (millis() - startTime > i2cTimeout || isnan(*temp)) {
      Serial.print("BMP280 Timeout bei Temperatur (Versuch ");
      Serial.print(attempt + 1);
      Serial.println(")");
      consecutiveBMP280Errors++;
      continue;
    }
    
    yield();
    
    *pressure = bmp.readPressure();
    
    if (millis() - startTime > i2cTimeout || isnan(*pressure)) {
      Serial.print("BMP280 Timeout bei Druck (Versuch ");
      Serial.print(attempt + 1);
      Serial.println(")");
      consecutiveBMP280Errors++;
      continue;
    }
    
    // Prüfe auf plausible Werte
    if (*pressure < 30000 || *pressure > 110000 || *temp < -40 || *temp > 85) {
      Serial.print("BMP280 unplausible Werte (Versuch ");
      Serial.print(attempt + 1);
      Serial.print("): T=");
      Serial.print(*temp);
      Serial.print(" P=");
      Serial.println(*pressure);
      consecutiveBMP280Errors++;
      continue;
    }
    
    // Erfolg bei diesem Versuch
    consecutiveBMP280Errors = 0;
    lastSuccessfulRead = millis();
    
    if (attempt > 0) {
      Serial.print("BMP280 erfolgreich nach ");
      Serial.print(attempt + 1);
      Serial.println(" Versuchen");
    }
    
    return true;
  }
  
  // Alle Versuche fehlgeschlagen
  Serial.println("BMP280 nach 3 Versuchen fehlgeschlagen - REBOOT!");
  performHardReboot();
  return false;
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
  Serial.println("AKTIV (ESP32-IDF 30s No-Panic)");
  
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
    if (pressureThreshold > 50.0) pressureThreshold = 0.1;
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
  
  // Graceful Shutdown zuerst prüfen (hat Priorität)
  checkGracefulShutdown();
  
  // BOOT-Taste prüfen (hat höchste Priorität)
  if (digitalRead(BOOT_PIN) == LOW && !bootMenuActive && !shutdownMode) {
    delay(50); // Entprellen
    if (digitalRead(BOOT_PIN) == LOW) {
      // Auf Taste loslassen warten
      while (digitalRead(BOOT_PIN) == LOW) {
        delay(10);
        esp_task_wdt_reset();
      }
      Serial.println("BOOT gedrückt - starte Menu");
      bootMenuActive = true;
      runBootMenu();
      bootMenuActive = false;
    }
  }
  
  // Normale Taster nur wenn nicht im Shutdown-Modus und nicht im Menü
  if (shutdownMode || bootMenuActive) return;
  
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
  tft.println("V. 0.17c");
}

void drawStatsOnDisplay() {
  if (!backlightOn) return;
  
  tft.fillScreen(BLACK_COLOR);
  tft.drawRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, BLUE_COLOR);

  tft.setTextColor(WHITE_COLOR);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("BOOT-STATISTIK");

  tft.setTextSize(1);
  tft.setTextColor(CYAN_COLOR);
  
  char timeBuffer[20];
  
  // Gesamt-Boots
  tft.setCursor(10, 35);
  tft.print("Boots: ");
  tft.setTextColor(WHITE_COLOR);
  tft.print(resetStats.totalBoots);
  
  // Gesamt-Crashes
  tft.setCursor(120, 35);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Crash: ");
  tft.setTextColor(WHITE_COLOR);
  tft.print(resetStats.totalCrashes);
  
  // Graceful Shutdowns
  tft.setCursor(10, 50);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Graceful: ");
  tft.setTextColor(GREEN_COLOR);
  tft.print(resetStats.gracefulShutdowns);
  
  // Crash-Rate
  if (resetStats.totalBoots > 0) {
    float crashRate = (float)resetStats.totalCrashes / resetStats.totalBoots * 100.0;
    tft.setCursor(120, 50);
    tft.setTextColor(CYAN_COLOR);
    tft.print("Rate: ");
    tft.setTextColor(WHITE_COLOR);
    tft.print(crashRate, 1);
    tft.println("%");
  }
  
  // Letzte Uptime
  formatTime(resetStats.lastUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  tft.setCursor(10, 65);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Letzte: ");
  tft.setTextColor(WHITE_COLOR);
  tft.println(timeBuffer);
  
  // Längste Uptime
  formatTime(resetStats.maxUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  tft.setCursor(10, 80);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Max: ");
  tft.setTextColor(WHITE_COLOR);
  tft.println(timeBuffer);
  
  // Gesamt-Laufzeit
  formatTime(resetStats.totalUptimeSeconds, timeBuffer, sizeof(timeBuffer));
  tft.setCursor(10, 95);
  tft.setTextColor(CYAN_COLOR);
  tft.print("Gesamt: ");
  tft.setTextColor(WHITE_COLOR);
  tft.println(timeBuffer);
  
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

  tft.setCursor(MODE_DISPLAY_X + 6, MODE_DISPLAY_Y);
  tft.print("M");
  tft.print((int)currentMode);
  /*
  switch(currentMode) {
    case M1_SIMPLE: tft.print(" 1"); break;
    case M2_EMA: tft.print(" 2"); break;
    case M3_DUAL: tft.print(" 3"); break;
  }
  */

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
    } else if (strcmp(input, "shutdown") == 0) {
      Serial.println("Manueller Graceful Shutdown...");
      performGracefulShutdown();
    } else if (strcmp(input, "menu") == 0) {
      Serial.println("Manuelles Boot Menu...");
      bootMenuActive = true;
      runBootMenu();
      bootMenuActive = false;
    } else if (strcmp(input, "help") == 0) {
      Serial.println("Befehle:");
      Serial.println("  threshold X - Setze Schwellenwert (z.B. 'threshold 2.5')");
      Serial.println("  th X - Kurzform (z.B. 'th 1.5' oder 'th 9,8')");
      Serial.println("  mode X - Setze Modus (1=EINFACH, 2=EMA, 3=DUAL)");
      Serial.println("  memory - Zeige Speicherbelegung");
      Serial.println("  stack - Zeige Stack-Informationen");
      Serial.println("  resetbmp - BMP280 manuell resetten");
      Serial.println("  resetsettings - Einstellungen auf Default zurücksetzen");
      Serial.println("  stats - Zeige Reset-Statistik");
      Serial.println("  shutdown - Manuelles Graceful Shutdown");
      Serial.println("  menu - Manuelles Boot Menu starten");
      Serial.println("  help - Zeige diese Hilfe");
      Serial.println("Watchdog: ESP32-IDF 30s No-Panic, Memory Guard aktiv");
      Serial.println("Settings: Persistent (überleben Stromtrennung)");
      Serial.println("Statistik: Reset-Tracking mit Uptime-Analyse + Crash-Recovery");
      Serial.println("BOOT Menu V.17c: 3-Sekunden-Timer pro Menüpunkt mit While-Loop");
      Serial.println("Graceful Shutdown: Display AUS + TH-Buttons (>2s)");
      Serial.println("Taster: D5=Threshold↑, D41=Threshold↓, D42=Hintergrundbeleuchtung, D6=Modus");
      Serial.println("BOOT: 3-Sekunden-Menü (4 Menüpunkte mit While-Loop)");
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
      Serial.print("  Gesamt-Boots: ");
      Serial.println(resetStats.totalBoots);
      Serial.print("  Gesamt-Crashes: ");
      Serial.println(resetStats.totalCrashes);
      Serial.print("  Graceful Shutdowns: ");
      Serial.println(resetStats.gracefulShutdowns);
      Serial.println("  BOOT-Taste: 3-Sekunden-Menü starten");
      Serial.println("  Zum sicheren Ausschalten: Display AUS + TH-Buttons 2s halten");
    } else {
      Serial.println("Unbekannter Befehl. 'help' für Hilfe.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESPVARIO RG 01/2026 - V. 0.17c start...");
  Serial.println("V.17c FEATURES: SIMPLE WHILE-LOOP MENU SYSTEM");
  Serial.println("BOOT-Taste: 3-Sekunden-Menü mit 4 Menüpunkten");
  Serial.println("1) Display off 2) Threshold 3) Modus 4) Power off");
  Serial.println("Einfache While-Loop für BOOT-Taste Erkennung im Menü");
  Serial.println("Watchdog: 30s No-Panic, BMP280 Safe-Mode, Memory Guard");
  Serial.println("Taster: D5=Threshold↑, D41=Threshold↓, D42=Hintergrundbeleuchtung, D6=Modus");
  
  // ERST: Settings laden!
  loadSettings();
  
  // Boot-Zeitstempel setzen
  resetStats.bootTimestamp = millis();
  settings.putUInt("bootTime", resetStats.bootTimestamp);
  
  printMemoryInfo();

  // BOOT-Taste als Input konfigurieren
  pinMode(BOOT_PIN, INPUT_PULLUP);

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
  Serial.println("=== KALT-BOOT STATISTIK V.17c ===");
  printResetStats();
  Serial.println("===================================");
  Serial.println();

  updateCounter = 0;
  loopCounter = 0;
  lastLoopReport = millis();
  lastMemoryCheck = millis();
  lastBMP280Check = millis();
  lastUptimeSave = millis();  // Uptime-Saving Timer starten
  lastMemoryGuard = millis(); // Memory Guard Timer starten

  // ESP32-IDF Watchdog initialisieren
  initWatchdog();

  // Initiale Stack-Überwachung
  checkStackUsage();

  Serial.println("Setup abgeschlossen, starte Watchdog-geschützten Betrieb...");
  Serial.println("Einstellungen und Statistik sind persistent und überleben Stromtrennung!");
  Serial.println("BOOT Menu V.17c mit 4 Menüpunkten verfügbar - BOOT-Taste drücken!");
  Serial.println("Einfache While-Loop für zuverlässige BOOT-Taste Erkennung im Menü!");
}

void loop() {
  loopCounter++;
  
  // ESP32-IDF Watchdog explizit füttern
  esp_task_wdt_reset();
  
  // Regelmäßiges yield() für Scheduler
  yield();
  
  // Stack-Überwachung alle 1000 Loops
  if (loopCounter % 1000 == 0) {
    checkStackUsage();
  }
  
  unsigned long currentMillis = millis();

  // Wenn Boot Menu aktiv, normale Loop pausieren
  if (bootMenuActive) {
    delay(10);  // Kleine Delay für Stabilität
    return;
  }

  checkTaster();
  checkSerialInput();

  checkBMP280Health();

  // Alle 30 Sekunden aktuelle Uptime speichern (Crash-Recovery)
  if (currentMillis - lastUptimeSave >= uptimeSaveInterval) {
    lastUptimeSave = currentMillis;
    saveCurrentUptime();
  }

  // Memory Guard alle 60 Sekunden
  if (currentMillis - lastMemoryGuard >= memoryGuardInterval) {
    lastMemoryGuard = currentMillis;
    checkMemoryGuard();
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
        Serial.print(" B:");
        Serial.print(resetStats.totalBoots);
        Serial.print(" C:");
        Serial.print(resetStats.totalCrashes);
        Serial.print(" G:");
        Serial.print(resetStats.gracefulShutdowns);
        Serial.print(" Menu:");
        Serial.print(bootMenuActive ? "ON" : "OFF");
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
