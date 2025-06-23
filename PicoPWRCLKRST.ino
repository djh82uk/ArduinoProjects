#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Wire.h>
#include <INA226.h>
#include <RP2040_PWM.h>
#include <Pushbutton.h>



#define TFT_CS     17 // Chip select line for TFT display
#define TFT_RST    15  // Reset line for TFT (or connect to +3V3)
#define TFT_DC     14 // Data/command line for TFT

#define UP_BTN_PIN 11
#define DOWN_BTN_PIN 10
#define STEP_BTN_PIN 9
#define SET_BTN_PIN 8

Pushbutton FIncButton(UP_BTN_PIN,DEFAULT_STATE_HIGH,PULL_UP_ENABLED);
Pushbutton FDecButton(DOWN_BTN_PIN,DEFAULT_STATE_HIGH,PULL_UP_ENABLED);
Pushbutton FStepButton(STEP_BTN_PIN,DEFAULT_STATE_HIGH,PULL_UP_ENABLED);
Pushbutton FSetButton(SET_BTN_PIN,DEFAULT_STATE_HIGH,PULL_UP_ENABLED);
// Initialize libraries

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
INA226 ina;

float lastVoltage = -1.0; 
float lastAmperage= -1.0; 
float lastWattage = -1.0; 
float lastFrequency = -1.0; 
float setFreq = 0.0;
float step = 1.0;

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
  digitalWrite(12, LOW); // CS LOW
  SPI.transfer(highByte(data));
  SPI.transfer(lowByte(data));
  digitalWrite(12, HIGH); // CS HIGH
}

void initAD9833(float frequency) {
  const uint32_t freqWord = (frequency * 268435456.0) / 25000000.0; // 28-bit word

  uint16_t freqLSB = (freqWord & 0x3FFF) | 0x4000; // Lower 14 bits + control bits
  uint16_t freqMSB = ((freqWord >> 14) & 0x3FFF) | 0x4000;

  writeRegister(0x2100);     // Reset + B28 mode
  writeRegister(freqLSB);    // Frequency LSB
  writeRegister(freqMSB);    // Frequency MSB
  writeRegister(0xC000);     // Phase 0
  writeRegister(0x2028);     // Exit reset, enable square wave
}

void checkConfig()
{
  Serial.print("Mode:                  ");
  switch (ina.getMode())
  {
    case INA226_MODE_POWER_DOWN:      Serial.println("Power-Down"); break;
    case INA226_MODE_SHUNT_TRIG:      Serial.println("Shunt Voltage, Triggered"); break;
    case INA226_MODE_BUS_TRIG:        Serial.println("Bus Voltage, Triggered"); break;
    case INA226_MODE_SHUNT_BUS_TRIG:  Serial.println("Shunt and Bus, Triggered"); break;
    case INA226_MODE_ADC_OFF:         Serial.println("ADC Off"); break;
    case INA226_MODE_SHUNT_CONT:      Serial.println("Shunt Voltage, Continuous"); break;
    case INA226_MODE_BUS_CONT:        Serial.println("Bus Voltage, Continuous"); break;
    case INA226_MODE_SHUNT_BUS_CONT:  Serial.println("Shunt and Bus, Continuous"); break;
    default: Serial.println("unknown");
  }
  
  Serial.print("Samples average:       ");
  switch (ina.getAverages())
  {
    case INA226_AVERAGES_1:           Serial.println("1 sample"); break;
    case INA226_AVERAGES_4:           Serial.println("4 samples"); break;
    case INA226_AVERAGES_16:          Serial.println("16 samples"); break;
    case INA226_AVERAGES_64:          Serial.println("64 samples"); break;
    case INA226_AVERAGES_128:         Serial.println("128 samples"); break;
    case INA226_AVERAGES_256:         Serial.println("256 samples"); break;
    case INA226_AVERAGES_512:         Serial.println("512 samples"); break;
    case INA226_AVERAGES_1024:        Serial.println("1024 samples"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Bus conversion time:   ");
  switch (ina.getBusConversionTime())
  {
    case INA226_BUS_CONV_TIME_140US:  Serial.println("140uS"); break;
    case INA226_BUS_CONV_TIME_204US:  Serial.println("204uS"); break;
    case INA226_BUS_CONV_TIME_332US:  Serial.println("332uS"); break;
    case INA226_BUS_CONV_TIME_588US:  Serial.println("558uS"); break;
    case INA226_BUS_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226_BUS_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226_BUS_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226_BUS_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Shunt conversion time: ");
  switch (ina.getShuntConversionTime())
  {
    case INA226_SHUNT_CONV_TIME_140US:  Serial.println("140uS"); break;
    case INA226_SHUNT_CONV_TIME_204US:  Serial.println("204uS"); break;
    case INA226_SHUNT_CONV_TIME_332US:  Serial.println("332uS"); break;
    case INA226_SHUNT_CONV_TIME_588US:  Serial.println("558uS"); break;
    case INA226_SHUNT_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226_SHUNT_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226_SHUNT_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226_SHUNT_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }
  
  Serial.print("Max possible current:  ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max current:           ");
  Serial.print(ina.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltage:     ");
  Serial.print(ina.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max power:             ");
  Serial.print(ina.getMaxPower());
  Serial.println(" W");
}

void updateFreq() {
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  initAD9833(setFreq); 
  SPI.endTransaction();
}

void setup() {
  SPI.begin();
  delay(5000);
  //if (PWM_Instance)
  //{
    //PWM_Instance->setPWM();
  //}
  tft.initR(INITR_144GREENTAB); // Initialize display with the correct settings
  tft.fillScreen(ST7735_BLACK);  // Clear the screen to black

  Serial.begin(115200);

  Serial.println("Initialize INA226");
  Serial.println("-----------------------------------------------");
  Wire.begin();
  Wire.setClock(400000L);

  // Default INA226 address is 0x40
  ina.begin();

  // Configure INA226
  ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);

  // Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
  ina.calibrate(0.1, 0.8);

  // Display configuration
  checkConfig();

  Serial.println("-----------------------------------------------");
  tft.fillRoundRect(0, 0, 128, 30, 12, BLUE);
  tft.fillRoundRect(0, 32, 128, 30, 12, RED);
  tft.fillRoundRect(0, 64, 128, 30, 12, MAGENTA);
  tft.fillRoundRect(0, 96, 128, 30, 12, RED2);

  updateFreq();
}


void loop() {


float voltage = ina.readBusVoltage();
if (abs(voltage - lastVoltage) > 0.03) {  // Only update if it changed by more than 10mV
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE, BLUE);
  tft.fillRoundRect(0, 0, 128, 30, 12, BLUE);  // Clear the area to avoid leftover pixels
  tft.setCursor(36, 8);
  tft.print(voltage, 2);  // Optional: round to 2 decimals
  tft.print("V");
  lastVoltage = voltage;
}

float wattage = ina.readBusPower();
if (abs(wattage - lastWattage) > 0.03) {  // Only update if it changed by more than 10mV
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE, RED);
  tft.fillRoundRect(0, 32, 128, 30, 12, RED);  // Clear the area to avoid leftover pixels
  tft.setCursor(36, 40);
  tft.print(wattage, 1);  // Optional: round to 2 decimals
  tft.print("W");
  lastWattage = wattage;
}
  
float amperage = ina.readShuntCurrent();
if (abs(amperage - lastAmperage) > 0.03) {  
  tft.setCursor(36, 72);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE, MAGENTA);
  tft.fillRoundRect(0, 64, 128, 30, 12, MAGENTA);  // Clear the area to avoid leftover pixels
  tft.setCursor(36, 72);
  tft.print(amperage, 1);  
  tft.print("A");
  lastAmperage = amperage;
}

float frequency = setFreq;
if (abs(frequency - lastFrequency) > 0.03) {  
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE, RED2);
  tft.fillRoundRect(0, 96, 128, 30, 12, RED2);;  // Clear the area to avoid leftover pixels
  tft.setCursor(26, 104);
  tft.print(frequency, 1);  
  tft.print("Hz");
  lastFrequency = frequency;
}


if (FIncButton.getSingleDebouncedPress()){
  Serial.println("FInc Button Pressed");
  setFreq = setFreq + step;
  updateFreq();
}

if (FDecButton.getSingleDebouncedPress()){
  Serial.println("FDec Button Pressed");
  setFreq = setFreq - step;
  updateFreq();
}

if (FStepButton.getSingleDebouncedPress()){
  Serial.println("FStep Button Pressed");
  static float stepSizes[] = {1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0};
  static int index = 0;
  index = (index + 1) % 6;
  Serial.print("New step size: ");
  step = stepSizes[index];
  Serial.println(stepSizes[index]);
}

if (FSetButton.getSingleDebouncedPress()){
  Serial.println("FSet Button Pressed");
}


//TODO Set Freq display to change to KHz, MHz etc, Have 2nd screen for selecting freq, step and set with timeout, update step sizes, add boot screen

}
