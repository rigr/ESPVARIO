# ESPVARIO RG – Version 0.9a (12/2025)

Ein einfaches, aber effektives Variometer basierend auf dem **Adafruit Feather ESP32-S3 TFT** (Original von Adafruit oder kompatibler günstiger Klon von **Tenstar Robot** auf AliExpress). Die Tenstar ROBOT Varianten funktioniert sicher mit diesem Code.

Das Gerät zeigt Temperatur, Luftdruck, eine Steig-/Sink-Anzeige mit numerischer Steigrate in m/s sowie einen Verlaufsgraph der letzten ca 15 Sekunden. Zusätzlich leuchten die onboard LEDs entsprechend der aktuellen Tendenz.

## Hardware

- **Board**: "TENSTAR ROBOT" ESP32-S3 mit integriertem 1.14" IPS-TFT-Display (240 × 135 Pixel) und BMP280-Drucksensor  
  (offenbar ähnlich dem Original von Adafruit)
- **Onboard RGB-LED** (Pin 33): Leuchtet grün bei Steigen, rot bei Sinken
- **Onboard rote LED** (Pin 13): Leuchtet bei Sinken
- **Integriertes TFT-Display** (ST7789-Controller, fest verlötet)
- **BMP280-Drucksensor** (I²C, Adresse meist 0x76, manchmal 0x77)

**Kein zusätzlicher Anschluss nötig** – Display und Sensor sind bereits fest integriert.

## Pin-Belegung (wie im Code verwendet)

| Funktion            | Pin | Hinweis                                      |
|---------------------|-----|----------------------------------------------|
| TFT_POWER           | 21  | Versorgung Display (HIGH = an)               |
| TFT_CS              | 7   | Chip Select                                  |
| TFT_DC              | 39  | Data/Command                                 |
| TFT_RST             | 40  | Reset                                        |
| TFT_BACKLIGHT       | 45  | Backlight (HIGH = an)                        |
| ONBOARD_RED_LED     | 13  | Kleine rote LED neben USB                    |
| RGB_LED_PIN         | 33  | Onboard NeoPixel (WS2812)                    |
| BMP280              | I²C | Standard-I²C-Pins (SDA = 8, SCL = 9)          |

## Features

- Startbildschirm mit Gerätename und Versionsangabe
- Anzeige von Temperatur und aktuellem Luftdruck (in hPa)
- Große, farbige Anzeige „STEIGEN“ (grün) oder „SINKEN“ (rot) mit aktueller Steigrate direkt daneben (±x.x m/s)
- Kleine „m/s“-Einheit rechtsbündig
- Verlaufsgraph der letzten 20 Sekunden mit wandernder Mittellinie (automatische Zentrierung auf gleitenden Mittelwert)
- Blaue Umrandung um das gesamte Display
- Keine zusätzlichen Bibliotheken außer den Standard-Adafruit-Bibliotheken erforderlich

## Benötigte Bibliotheken (Arduino IDE)

Alle Bibliotheken sind über **Werkzeuge → Bibliotheken verwalten** installierbar:

1. **Adafruit ST7789 Library** (von Adafruit)
2. **Adafruit GFX Library** (von Adafruit)
3. **Adafruit BMP280 Library** (von Adafruit)
4. **Adafruit NeoPixel Library** (von Adafruit)

## Board-Einstellungen in der Arduino IDE

- **Board**: „Adafruit Feather ESP32-S3 TFT“  
  (bei Klonen ggf. „ESP32S3 Dev Module“ verwenden)
- **Upload Speed**: 921600
- **Partition Scheme**: „Default 4MB with spiffs“ (oder vergleichbar)
- **PSRAM**: „Enabled“ (falls verfügbar)

## Installation

1. Arduino IDE öffnen
2. Das passende Board auswählen (siehe oben)
3. Die genannten Bibliotheken installieren
4. Den Sketch auf das Board hochladen
5. Nach dem Start erscheint zunächst für ca. 3 Sekunden der Startbildschirm, danach die Variometer-Anzeige

**Fertig!**

## Wichtige Code-Auszüge (Setup-Beispiel)

```cpp
#define TFT_POWER     21
#define TFT_CS        7
#define TFT_DC        39
#define TFT_RST       40
#define TFT_BACKLIGHT 45

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  pinMode(TFT_POWER, OUTPUT);
  digitalWrite(TFT_POWER, HIGH);
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  
  SPI.begin();
  tft.init(135, 240);        // WICHTIG: 135x240 für dieses Display!
  tft.setRotation(3);        // Ausrichtung (0–3 testen, meist ist 3 korrekt)
}
```


## Hinweise zur Funktion

Der BMP280 wird automatisch unter den Adressen 0x76 oder 0x77 gesucht.
Der Verlaufsgraph baut sich in den ersten 20 Sekunden auf und zentriert sich automatisch auf den gleitenden Mittelwert – dadurch bleiben auch kleine Schwankungen gut sichtbar.
Die Steigrate wird über ca. 3 Sekunden gemittelt (gute Reaktionszeit bei minimalem Flackern).

## Lizenz & Credits

Code korrigiert mit Hilfe von KI (Grok und chatGPT -  die beiden vergessen kein ";" am Zeilenende) im Dezember 2025
Frei verwendbar und modifizierbar für private und nicht-kommerzielle Zwecke. 

Viel Spaß in der Thermik und immer schöne Pluswerte! 🪂💚
— RG, 12/2025
