#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Wire.h>
#include <INA226.h>
#include <RP2040_PWM.h>
#include <Pushbutton.h>

#define TFT_CS     17
#define TFT_RST    15
#define TFT_DC     14

#define UP_BTN_PIN    11
#define DOWN_BTN_PIN  10
#define STEP_BTN_PIN   9
#define SET_BTN_PIN    8

Pushbutton FIncButton(UP_BTN_PIN, DEFAULT_STATE_HIGH, PULL_UP_ENABLED);
Pushbutton FDecButton(DOWN_BTN_PIN, DEFAULT_STATE_HIGH, PULL_UP_ENABLED);
Pushbutton FStepButton(STEP_BTN_PIN, DEFAULT_STATE_HIGH, PULL_UP_ENABLED);
Pushbutton FSetButton(SET_BTN_PIN, DEFAULT_STATE_HIGH, PULL_UP_ENABLED);

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
INA226 ina;

float lastVoltage = -1.0;
float lastAmperage = -1.0;
float lastWattage = -1.0;
float lastFrequency = -1.0;
float setFreq = 0.0;
float step = 1.0;

enum ScreenMode { MAIN_SCREEN, SET_FREQ_SCREEN };
ScreenMode currentScreen = MAIN_SCREEN;
unsigned long lastInteraction = 0;
const unsigned long screenTimeout = 10000;

#define BLACK    0x0000
#define BLUE     0x001F
#define RED2     0xD28B
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

void writeRegister(uint16_t data) {
  digitalWrite(12, LOW);
  SPI.transfer(highByte(data));
  SPI.transfer(lowByte(data));
  digitalWrite(12, HIGH);
}

void initAD9833(float frequency) {
  const uint32_t freqWord = (frequency * 268435456.0) / 25000000.0;
  uint16_t freqLSB = (freqWord & 0x3FFF) | 0x4000;
  uint16_t freqMSB = ((freqWord >> 14) & 0x3FFF) | 0x4000;

  writeRegister(0x2100);
  writeRegister(freqLSB);
  writeRegister(freqMSB);
  writeRegister(0xC000);
  writeRegister(0x2000);
  writeRegister(0x2028);
}

void updateFreq() {
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  initAD9833(setFreq);
  SPI.endTransaction();
}

void printCenteredText(String text, uint16_t y, uint16_t bgColor, uint16_t textColor, uint8_t textSize = 2) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.setTextSize(textSize);
  tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int x = (tft.width() - w) / 2;
  tft.setTextColor(textColor, bgColor);
  tft.fillRoundRect(0, y - 8, 128, 30, 12, bgColor);
  tft.setCursor(x, y);
  tft.print(text);
}

void printCenteredFreq(float freq) {
  char buffer[10];
  String unit;
  float disp_freq;

  if (freq >= 1000000) {
    disp_freq = freq / 1000000;
    unit = "MHz";
  } else if (freq >= 1000) {
    disp_freq = freq / 1000;
    unit = "KHz";
  } else {
    disp_freq = freq;
    unit = "Hz";
  }

  snprintf(buffer, sizeof(buffer), "%.1f", disp_freq);
  String displayStr = String(buffer) + unit;

  int16_t x1, y1;
  uint16_t w, h;
  tft.setTextSize(2);
  tft.getTextBounds(displayStr, 0, 0, &x1, &y1, &w, &h);
  int x = (tft.width() - w) / 2;
  int y = 104;
  tft.setTextColor(WHITE, RED2);
  tft.fillRoundRect(0, 96, 128, 30, 12, RED2);
  tft.setCursor(x, y);
  tft.print(displayStr);
}


void drawButtonLabels() {
  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);

  // Right-aligned labels near edge
  tft.setCursor(120, 4);    // Top button (+)
  tft.print("+");

  tft.setCursor(120, 37);   // Second button (–)
  tft.print("-");

  tft.setCursor(103, 95);    // Third button (step)
  tft.print("step");

  tft.setCursor(107, 120);   // Bottom button (set)
  tft.print("set");
}


void setup() {
  SPI.begin();
  delay(500);
  tft.initR(INITR_144GREENTAB);
  tft.fillScreen(BLACK);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);
  ina.begin();

  ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US,
                INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  ina.calibrate(0.1, 0.8);

  tft.fillRoundRect(0, 0, 128, 30, 12, BLUE);
  tft.fillRoundRect(0, 32, 128, 30, 12, RED);
  tft.fillRoundRect(0, 64, 128, 30, 12, MAGENTA);
  tft.fillRoundRect(0, 96, 128, 30, 12, RED2);

  updateFreq();
}
void drawSetFreqScreen() {
  tft.fillScreen(BLACK);
  printCenteredFreq(setFreq);

  String stepStr;
  if (step >= 1000000) stepStr = String(step / 1000000.0, 1) + "MHz";
  else if (step >= 1000) stepStr = String(step / 1000.0, 1) + "KHz";
  else stepStr = String(step, 1) + "Hz";
  printCenteredText("Step: " + stepStr, 60, MAGENTA, WHITE, 1);
  drawButtonLabels();
}

void loop() {


  // Store press states once per loop
  bool incPressed  = FIncButton.getSingleDebouncedPress();
  bool decPressed  = FDecButton.getSingleDebouncedPress();
  bool stepPressed = FStepButton.getSingleDebouncedPress();
  bool setPressed  = FSetButton.getSingleDebouncedPress();

  unsigned long now = millis();

  if (currentScreen == MAIN_SCREEN) {
    if (incPressed || decPressed || stepPressed || setPressed) {
      currentScreen = SET_FREQ_SCREEN;
      lastInteraction = now;
      drawSetFreqScreen();
    } else {
      float voltage = ina.readBusVoltage();
      if (abs(voltage - lastVoltage) > 0.03) {
        printCenteredText(String(voltage, 2) + "V", 8, BLUE, WHITE);
        lastVoltage = voltage;
      }

      float wattage = ina.readBusPower();
      if (abs(wattage - lastWattage) > 0.03) {
        printCenteredText(String(wattage, 1) + "W", 40, RED, WHITE);
        lastWattage = wattage;
      }

      float amperage = ina.readShuntCurrent();
      if (abs(amperage - lastAmperage) > 0.03) {
        printCenteredText(String(amperage, 1) + "A", 72, MAGENTA, WHITE);
        lastAmperage = amperage;
      }

      if (abs(setFreq - lastFrequency) > 0.03) {
        printCenteredFreq(setFreq);
        lastFrequency = setFreq;
      }
    }
  } else if (currentScreen == SET_FREQ_SCREEN) {
    if (incPressed || decPressed || stepPressed || setPressed) {
      lastInteraction = now;

      if (incPressed) {
        setFreq += step;
        if (setFreq > 5000000.0) setFreq = 5000000.0;
        if (setFreq < 0) setFreq = 0;
        updateFreq();
        drawSetFreqScreen();
      }

      if (decPressed) {
        setFreq -= step;
        if (setFreq > 5000000.0) setFreq = 5000000.0;
        if (setFreq < 0) setFreq = 0;
        updateFreq();
        drawSetFreqScreen();
      }

      if (stepPressed) {
        static float stepSizes[] = {1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0};
        static int index = 0;
        index = (index + 1) % 7;
        step = stepSizes[index];
        drawSetFreqScreen();
      }

      if (setPressed) {
        currentScreen = MAIN_SCREEN;
        tft.fillScreen(BLACK);
        tft.fillRoundRect(0, 0, 128, 30, 12, BLUE);
        tft.fillRoundRect(0, 32, 128, 30, 12, RED);
        tft.fillRoundRect(0, 64, 128, 30, 12, MAGENTA);
        tft.fillRoundRect(0, 96, 128, 30, 12, RED2);
        lastVoltage = lastWattage = lastAmperage = lastFrequency = -1.0;
      }
    }

    if (now - lastInteraction > screenTimeout) {
      currentScreen = MAIN_SCREEN;
      tft.fillScreen(BLACK);
      tft.fillRoundRect(0, 0, 128, 30, 12, BLUE);
      tft.fillRoundRect(0, 32, 128, 30, 12, RED);
      tft.fillRoundRect(0, 64, 128, 30, 12, MAGENTA);
      tft.fillRoundRect(0, 96, 128, 30, 12, RED2);
      lastVoltage = lastWattage = lastAmperage = lastFrequency = -1.0;
    }
  }
}
