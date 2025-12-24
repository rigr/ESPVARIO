// Battery Percentage Project 
// TS-ESP32-S3-1.14TFT | VBAT via ADC (GPIO 4)

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ================= PINS =================
#define TFT_POWER     21
#define TFT_CS        7
#define TFT_DC        39
#define TFT_RST       40
#define TFT_BACKLIGHT 45

#define BAT_ADC_PIN   4       // GPIO für Akku
#define BAT_DIVIDER   2.0f    // 1:1 Spannungsteiler

// ================= TFT ==================
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// ================= FILTER =================
float filteredVoltage = 0.0;
const float FILTER_ALPHA = 0.15f;  // 0..1 (kleiner = ruhiger)

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("ESPVARIO Battery Percentage (boot-safe)");

  // ---- Boot-safe TFT Pins initialisieren ----
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(TFT_RST, HIGH);

  pinMode(TFT_POWER, OUTPUT);
  digitalWrite(TFT_POWER, HIGH);

  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, LOW);   // erst später einschalten

  delay(10);

  SPI.begin();
  tft.init(135, 240);      // exakt wie ESPVARIO
  tft.setRotation(3);      // exakt wie ESPVARIO

  digitalWrite(TFT_BACKLIGHT, HIGH);

  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), ST77XX_BLUE);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 10);
  tft.println("Battery Status");

  // ADC vorbereiten
  analogReadResolution(12);                 // 0..4095
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

  // Startwert setzen
  int raw = analogRead(BAT_ADC_PIN);
  float adcV = raw * 3.3f / 4095.0f;
  filteredVoltage = adcV * BAT_DIVIDER;
}

// ================= LiPo Prozent =================
int voltageToPercent(float v) {
  if (v >= 4.20) return 100;
  if (v >= 4.10) return map(v * 100, 410, 420, 90, 100);
  if (v >= 4.00) return map(v * 100, 400, 410, 75, 90);
  if (v >= 3.90) return map(v * 100, 390, 400, 60, 75);
  if (v >= 3.80) return map(v * 100, 380, 390, 40, 60);
  if (v >= 3.70) return map(v * 100, 370, 380, 20, 40);
  if (v >= 3.60) return map(v * 100, 360, 370, 10, 20);
  if (v >= 3.50) return map(v * 100, 350, 360, 0, 10);
  return 0;
}

// ================= LOOP =================
void loop() {
  int raw = analogRead(BAT_ADC_PIN);

  float adcVoltage = raw * 3.3f / 4095.0f;
  float batteryVoltage = adcVoltage * BAT_DIVIDER;

  // Glättung
  filteredVoltage =
    filteredVoltage * (1.0f - FILTER_ALPHA) +
    batteryVoltage * FILTER_ALPHA;

  int percent = voltageToPercent(filteredVoltage);

  // Farbe wählen
  uint16_t color;
  if (percent >= 50)      color = ST77XX_GREEN;
  else if (percent >= 20) color = ST77XX_YELLOW;
  else                    color = ST77XX_RED;

  // ---- TFT ----
  tft.fillRect(10, 40, tft.width() - 20, 120, ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 40);
  tft.print("Voltage:");

  tft.setCursor(10, 55);
  tft.print(filteredVoltage, 2);
  tft.print(" V");

  tft.setCursor(10, 80);
  tft.print("Charge:");

  tft.setTextSize(3);
  tft.setTextColor(color);
  tft.setCursor(10, 100);
  tft.print(percent);
  tft.print(" %");

  // ---- Serial ----
  Serial.print("RAW=");
  Serial.print(raw);
  Serial.print("  VBAT=");
  Serial.print(filteredVoltage, 2);
  Serial.print(" V  ");
  Serial.print(percent);
  Serial.println(" %");

  delay(1000);
}
