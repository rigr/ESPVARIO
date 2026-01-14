/*
 * ESPVARIO - Vario mit ESP32
 * ===========================================
 * 
 * Board: ESP32 DevKit (oder kompatibles ESP32 Board)
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
 * - Stufen: <2.0 Pa = 0.1er Schritte
 *           <10.0 Pa = 0.5er Schritte
 *           >=10.0 Pa = 1er Schritte
 * 
 * SERIELLE BEFEHLE:
 * -----------------
 * th X.X      - Threshold setzen (z.B. "th 1.5" oder "th 9,8")
 * threshold X - Lange Form (z.B. "threshold 2.3")
 * help        - Hilfe anzeigen
 * 
 * ANWENDUNG:
 * ----------
 * Der Vario hilft Segelfliegern beim Finden von Thermik.
 * Steigende Luft (Thermik) zeigt mit "STEIGEN" und grüner LED,
 * sinkende Luft mit "SINKEN" und roter LED.
 * 
 * Der Threshold filtert kleine Druckschwankungen heraus und
 * kann je nach Bedingungen angepasst werden.
 * 
 * VERSION:
 * --------
 * V. 0.9c - Mit Taster-Steuerung und variablem Threshold
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

// Schwellenwert für LED-Aktivierung (kann per Taster geändert werden)
float pressureThreshold = 1.5;

const int HISTORY_SIZE = 50;
float pressureHistory[HISTORY_SIZE];
int historyIndex = 0;
bool historyFull = false;

bool pressureRising = false;
float verticalSpeed = 0.0;

unsigned long previousMeasureMillis = 0;
const unsigned long measureInterval = 200; // measurement and screen update rate done here - had it on 300, prefer 200

int updateCounter = 0;
const int FULL_REFRESH_EVERY = 20;

bool showStartup = true;
unsigned long startupStartTime = 0;
const unsigned long startupDuration = 4000;

const int GRAPH_X_LEFT = 10;
const int GRAPH_WIDTH = tft.width() - 20;

#define GRAPH_TOP       87
#define GRAPH_BOTTOM    (tft.height() - 5)

// Angepasste Positionen für Temperatur und Druck (5px nach links verschoben)
#define TEMP_VALUE_X    115  // war 120
#define TEMP_VALUE_Y    10
#define PRESS_VALUE_X   105  // war 110
#define PRESS_VALUE_Y   30
#define MODE_TEXT_X     5
#define MODE_TEXT_Y     55
#define SPEED_VALUE_X   140
#define SPEED_VALUE_Y   55

// Position für gestapeltes m/s – sicher außerhalb des Löschbereichs
#define MS_X            (tft.width() - 22)
#define MS_Y_M          58
#define MS_Y_LINE       67
#define MS_Y_S          69

// Positionen für Threshold-Anzeige
#define THRESH_VALUE_X   170  // Rechts vom °C
#define THRESH_VALUE_Y   10
#define THRESH_HPA_X     175  // Rechts vom hPa
#define THRESH_HPA_Y     30

// Taster-Entprellung
unsigned long lastUpPress = 0;
unsigned long lastDownPress = 0;
unsigned long lastBlPress = 0;
const unsigned long debounceDelay = 200; // 200ms Entprellzeit

bool backlightOn = true;

void setLeds() {
  // LEDs nur leuchten lassen, wenn Druckunterschied >= Schwellenwert
  float avgPressure = getAveragePressure();
  float currentPressure = pressureHistory[(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE];
  float pressureDifference = fabs(currentPressure - avgPressure);
  
  if (pressureDifference >= pressureThreshold) {
    digitalWrite(ONBOARD_RED_LED, pressureRising ? HIGH : LOW);
    if (pressureRising) {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));  // Rot bei Sinken
    } else {
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));  // Grün bei Steigen
    }
  } else {
    // LEDs aus bei weniger als Schwellenwert Unterschied
    digitalWrite(ONBOARD_RED_LED, LOW);
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));  // Aus
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
  
  // Sofort update erzwingen, um neuen Threshold anzuzeigen
  updateCounter = FULL_REFRESH_EVERY;
}

void checkTaster() {
  unsigned long currentMillis = millis();
  
  // T_UP_PIN prüfen
  if (digitalRead(T_UP_PIN) == LOW && currentMillis - lastUpPress > debounceDelay) {
    lastUpPress = currentMillis;
    adjustThreshold(true);
  }
  
  // T_DOWN_PIN prüfen
  if (digitalRead(T_DOWN_PIN) == LOW && currentMillis - lastDownPress > debounceDelay) {
    lastDownPress = currentMillis;
    adjustThreshold(false);
  }
  
  // T_BL_PIN prüfen (Hintergrundbeleuchtung)
  if (digitalRead(T_BL_PIN) == LOW && currentMillis - lastBlPress > debounceDelay) {
    lastBlPress = currentMillis;
    backlightOn = !backlightOn;
    digitalWrite(TFT_BACKLIGHT, backlightOn ? HIGH : LOW);
    Serial.println(backlightOn ? "Hintergrundbeleuchtung EIN" : "Hintergrundbeleuchtung AUS");
  }
}

void drawStartup() {
  tft.fillScreen(BLACK_COLOR);
  tft.drawRect(0, 0, tft.width(), tft.height(), BLUE_COLOR);

  tft.setTextColor(WHITE_COLOR);
  tft.setTextSize(3);
  tft.setCursor(20, tft.height() / 2 - 44);
  tft.println("ESPVARIO");

  tft.setTextColor(DARK_BLUE);
  tft.setCursor(20, tft.height() / 2 - 10);
  tft.println("RG 12/2025");

  tft.setTextSize(2);
  tft.setTextColor(DARK_RED);
  tft.setCursor(20, tft.height() / 2 + 30);
  tft.println("V. 0.9c");
}

void drawStaticParts() {
  tft.fillScreen(BLACK_COLOR);
  tft.drawRect(0, 0, tft.width(), tft.height(), BLUE_COLOR);  // Voller Rahmen

  tft.setTextSize(2);
  tft.setTextColor(WHITE_COLOR);

  tft.setCursor(15, 10);
  tft.print("Temp:");
  tft.setCursor(15, PRESS_VALUE_Y);
  tft.print("Druck:");

  // °C und hPa - jetzt 5px weiter links
  tft.setCursor(TEMP_VALUE_X + 45, 10);  // war +50
  tft.print(" C");
  tft.setCursor(PRESS_VALUE_X + 55, 30);  // war +60
  tft.print(" hPa");

  // Threshold-Anzeige
  tft.setTextColor(CYAN_COLOR);
  tft.setTextSize(1);
  tft.setCursor(THRESH_VALUE_X, THRESH_VALUE_Y + 5);
  tft.print(pressureThreshold, 1);
  tft.print("Pa");
  
  tft.setCursor(THRESH_HPA_X, THRESH_HPA_Y + 5);
  tft.print(pressureThreshold, 1);
  tft.print("Pa");

  // Gestapeltes m/s – sicher und dauerhaft sichtbar
  tft.setTextSize(1);
  tft.setTextColor(DARK_BLUE);
  tft.setCursor(MS_X, MS_Y_M);
  tft.print("m");
  tft.drawFastHLine(MS_X-2, MS_Y_LINE, 10, DARK_BLUE);  // Unterstrich
  tft.setCursor(MS_X, MS_Y_S);
  tft.print("s");
}

void updateDynamicParts(float temp, float pressure) {
  char buf[10];

  // Temperatur
  tft.fillRect(TEMP_VALUE_X, TEMP_VALUE_Y, 48, 22, BLACK_COLOR);
  tft.setTextSize(2);
  tft.setTextColor(WHITE_COLOR);
  tft.setCursor(TEMP_VALUE_X, TEMP_VALUE_Y);
  sprintf(buf, "%.1f", temp);
  tft.print(buf);

  // Druck
  tft.fillRect(PRESS_VALUE_X, PRESS_VALUE_Y, 60, 22, BLACK_COLOR);
  tft.setTextColor(WHITE_COLOR);
  tft.setCursor(PRESS_VALUE_X, PRESS_VALUE_Y);
  sprintf(buf, "%.1f", pressure / 100.0);
  tft.print(buf);

  // STEIGEN / SINKEN + Geschwindigkeit
  // WICHTIG: Breite reduziert → rechter Rahmen bleibt erhalten, kein Artefakt unter "Druck:"
  // NEU: sauber begrenzt, kein Überlappen mit "Druck:" und kein Rahmenverlust
  tft.fillRect(MODE_TEXT_X, MODE_TEXT_Y, tft.width() - 36, 32, BLACK_COLOR);

  tft.setTextSize(3);
  tft.setTextColor(pressureRising ? RED_COLOR : GREEN_COLOR);

  String modeText = pressureRising ? "SINKEN " : "STEIGEN ";
  tft.setCursor(MODE_TEXT_X + 10, MODE_TEXT_Y);
  tft.print(modeText);

  String speedStr = (verticalSpeed >= 0 ? "+" : "") + String(verticalSpeed, 1);
  tft.setCursor(SPEED_VALUE_X, SPEED_VALUE_Y);
  tft.print(speedStr);

  // m/s wird NICHT hier überschrieben → bleibt stehen!
  
  // SERIELLE AUSGABE
  Serial.print("T: ");
  Serial.print(temp, 1);
  Serial.print(" °C, P: ");
  Serial.print(pressure, 1);
  Serial.print(" Pa, ");
  Serial.print(pressureRising ? "DOWN" : "UP  ");
  Serial.print(" ");
  Serial.print(speedStr);
  Serial.print(" m/s, Schwelle: ");
  Serial.print(pressureThreshold, 1);
  Serial.print(" Pa, BL: ");
  Serial.println(backlightOn ? "ON" : "OFF");
}

void drawGraph() {
  int count = historyFull ? HISTORY_SIZE : historyIndex;
  if (count < 2) return;

  tft.fillRect(GRAPH_X_LEFT, GRAPH_TOP, GRAPH_WIDTH, GRAPH_BOTTOM - GRAPH_TOP + 1, BLACK_COLOR);

  float reference = pressureHistory[(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE];
  const float MAX_DEVIATION_PA = 20.0f;  // Hohe Empfindlichkeit
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

float getAveragePressure() {
  float sum = 0;
  int count = historyFull ? HISTORY_SIZE : historyIndex;
  if (count == 0) return pressureHistory[0];
  for (int i = 0; i < count; i++) sum += pressureHistory[i];
  return sum / count;
}

void checkSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // "threshold X" oder "th X" akzeptieren
    if (input.startsWith("threshold ")) {
      float newThreshold = input.substring(10).toFloat();
      if (newThreshold >= 0.1 && newThreshold <= 50.0) {
        pressureThreshold = newThreshold;
        Serial.print("Schwellenwert gesetzt auf: ");
        Serial.print(pressureThreshold, 1);
        Serial.println(" Pa");
        updateCounter = FULL_REFRESH_EVERY;  // Display sofort aktualisieren
      } else {
        Serial.println("Fehler: Schwellenwert muss zwischen 0.1 und 50.0 Pa liegen");
      }
    } else if (input.startsWith("th ")) {
      float newThreshold = input.substring(3).toFloat();
      if (newThreshold >= 0.1 && newThreshold <= 50.0) {
        pressureThreshold = newThreshold;
        Serial.print("Schwellenwert gesetzt auf: ");
        Serial.print(pressureThreshold, 1);
        Serial.println(" Pa");
        updateCounter = FULL_REFRESH_EVERY;  // Display sofort aktualisieren
      } else {
        Serial.println("Fehler: Schwellenwert muss zwischen 0.1 und 50.0 Pa liegen");
      }
    } else if (input == "help") {
      Serial.println("Befehle:");
      Serial.println("  threshold X - Setze Schwellenwert (z.B. 'threshold 2.5')");
      Serial.println("  th X - Kurzform (z.B. 'th 1.5' oder 'th 9.8')");
      Serial.println("  help - Zeige diese Hilfe");
      Serial.println("Taster: D5=erhöhen, D41=senken, D42=Hintergrundbeleuchtung");
      Serial.print("  Aktueller Schwellenwert: ");
      Serial.print(pressureThreshold, 1);
      Serial.println(" Pa");
    } else {
      Serial.println("Unbekannter Befehl. 'help' für Hilfe.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESPVARIO RG 12/2025 - V. 0.9c mit Taster-Steuerung start...");
  Serial.println("Taster: D5=Threshold↑, D41=Threshold↓, D42=Hintergrundbeleuchtung");
  Serial.print("Start-Schwellenwert: ");
  Serial.print(pressureThreshold, 1);
  Serial.println(" Pa");

  // Taster als Eingang mit Pullup
  pinMode(T_UP_PIN, INPUT_PULLUP);
  pinMode(T_DOWN_PIN, INPUT_PULLUP);
  pinMode(T_BL_PIN, INPUT_PULLUP);

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

  SPI.begin();
  tft.init(135, 240);
  tft.setRotation(3);

  Wire.begin();
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("BMP280 nicht gefunden!");
      while (1);
    }
  }

  float initialPressure = bmp.readPressure();
  for (int i = 0; i < HISTORY_SIZE; i++) {
    pressureHistory[i] = initialPressure;
  }

  drawStartup();
  startupStartTime = millis();

  updateCounter = 0;
}

void loop() {
  unsigned long currentMillis = millis();

  // Taster prüfen
  checkTaster();
  
  // Serielle Eingabe prüfen
  checkSerialInput();

  if (showStartup) {
    if (currentMillis - startupStartTime >= startupDuration) {
      showStartup = false;
      tft.fillScreen(BLACK_COLOR);
      drawStaticParts();  // inkl. m/s, Rahmen und Threshold
      drawGraph();
    }
    return;
  }

  if (currentMillis - previousMeasureMillis >= measureInterval) {
    previousMeasureMillis = currentMillis;

    float temp = bmp.readTemperature();
    float pressure = bmp.readPressure();

    float avgPrevious = getAveragePressure();

    if (historyIndex >= 6) {
      int oldIdx = (historyIndex - 6 + HISTORY_SIZE) % HISTORY_SIZE;
      float oldPressure = pressureHistory[oldIdx];
      float dP = pressure - oldPressure;
      float dt = 3.0;
      verticalSpeed = - (dP / 100.0) * 8.3 / dt;
    } else {
      verticalSpeed = 0.0;
    }

    pressureHistory[historyIndex] = pressure;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    if (historyIndex == 0) historyFull = true;

    bool newRising = (pressure > avgPrevious);
    if (newRising != pressureRising) {
      pressureRising = newRising;
      setLeds();
    }

    if (updateCounter >= FULL_REFRESH_EVERY) {
      drawStaticParts();  // Frisches m/s, Rahmen und Threshold
      updateCounter = 0;
    } else {
      updateCounter++;
    }

    updateDynamicParts(temp, pressure);
    drawGraph();
  }
}
