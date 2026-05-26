
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ======================================================
// HARDWARE
// ======================================================

constexpr uint8_t PIN_LED      = 8;
constexpr uint8_t PIN_CURRENT  = A0;
constexpr uint8_t PIN_VOLTAGE  = A1;

// ======================================================
// DAC
// ======================================================

Adafruit_MCP4725 dac;

// ======================================================
// ADC / DAC CONSTANTS
// ======================================================

constexpr float ADC_MAX        = 1023.0f;
constexpr float DAC_MAX        = 4095.0f;

// ======================================================
// ELECTRICAL LIMITS
// ======================================================

constexpr float MAX_CURRENT_A  = 5.0f;
constexpr float MAX_VOLTAGE_V  = 30.0f;
constexpr float MAX_POWER_W    = 150.0f;

// ======================================================
// CALIBRATION
// ======================================================

// Voltage divider calibration
constexpr float VOLTAGE_GAIN   = 6.030f;

// Current sense calibration
constexpr float CURRENT_GAIN   = 1.000f;

// DAC current calibration
constexpr float DAC_CAL        = 1.095f;

// Internal 1.1V reference calibration
constexpr long VCC_CAL         = 1106706L;

// ======================================================
// TIMING
// ======================================================

constexpr uint32_t LED_INTERVAL_MS    = 500;
constexpr uint32_t PRINT_INTERVAL_MS  = 250;
constexpr uint32_t CONTROL_INTERVAL_MS = 20;

// ======================================================
// MODES
// ======================================================

enum class LoadMode : uint8_t
{
    CC = 1,
    CP = 2,
    CR = 3
};

// ======================================================
// MEASUREMENTS
// ======================================================

struct Measurements
{
    float voltage;
    float current;
    float power;
};

Measurements meas;

// ======================================================
// SETPOINT
// ======================================================

LoadMode mode = LoadMode::CC;
float setpoint = 0.0f;

// ======================================================
// TIMERS
// ======================================================

uint32_t lastLedToggle   = 0;
uint32_t lastPrint       = 0;
uint32_t lastControl     = 0;

// ======================================================
// DISPLAY UI
// ======================================================
#define USE_OLED

#if defined(USE_OLED)

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// control buttons connected to digital pins 4 and 5
const int BTN_UP   = 4;
const int BTN_OK   = 5;

// control potentiometer connected to analog pin A2
const int potpin = A2;
int potValue = 0; 
long potMaped = 0;

int menu = 0;

#endif

// ======================================================
// SERIAL
// ======================================================

//String serialLine;
constexpr uint8_t CMD_BUFFER_SIZE = 64;
char serialLine[CMD_BUFFER_SIZE];
uint8_t serialIndex = 0;
bool editingCommand = false;
bool quiet = false;

// ======================================================
// SPI comms
// ======================================================

//#define USE_SPICOMMS

#if defined(USE_SPICOMMS)

struct __attribute__((packed)) CommandPacket
{
    uint32_t sync;
    uint32_t cmd;
    uint32_t arg;
};

struct __attribute__((packed)) StatusPacket
{
    uint32_t sync;
    uint32_t mode;
    uint32_t setpoint;
    uint32_t voltage;
    uint32_t current;
};

volatile uint8_t txIndex = 0;
volatile uint8_t rxIndex = 0;

constexpr uint8_t COMMAND_SIZE = sizeof(CommandPacket);
constexpr uint8_t STATUS_SIZE  = sizeof(StatusPacket);

uint8_t txData[STATUS_SIZE];
uint8_t txPending[STATUS_SIZE];

volatile uint8_t rxData[COMMAND_SIZE];

volatile bool packetReceived = false;
volatile bool txUpdatePending = false;

volatile StatusPacket txPacket;

#endif

// ======================================================
// SPI ISR
// ======================================================

#if defined(USE_SPICOMMS)

/*
ISR(SPI_STC_vect) {

  uint8_t in = SPDR;

  if (rxIndex > 0 && rxIndex <= 16) {
    rxData[rxIndex - 1] = in;
  }

  if (txIndex < 16) {
    SPDR = txData[txIndex++];
  } else {
    SPDR = 0x00;
  }
  rxIndex++;

  if (rxIndex >= 17) {
    packetReceived = true;
    rxIndex = 0;
    txIndex = 0;
  }
}
*/

ISR(SPI_STC_vect)
{
    uint8_t in = SPDR;

    // guardar RX do master
    if (rxIndex < COMMAND_SIZE)
    {
        rxData[rxIndex++] = in;
    }

    // preparar TX para master
    if (txIndex < STATUS_SIZE)
    {
        SPDR = txData[txIndex++];
    }
    else
    {
        SPDR = 0x00;
    }
}

#endif

#if defined(USE_OLED)

// ======================================================
// HELPER FUNCTIONS
// ======================================================

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float quantizeRound(float x, float step)
{
  return round(x / step) * step;
}

float quantizeRoundZero(float x, float step)
{
  if (x < step*1.3)
      return 0;

  return round(x / step) * step;
}

// ======================================================
// Display info
// ======================================================

void oledInfo (void) {
  if (menu != 0) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  switch (mode) {
    case LoadMode::CC:
      display.print("CC ");
      display.print(setpoint, 3);
      display.print(" A");
      break;
    case LoadMode::CP:
      display.print("CP ");
      display.print(setpoint, 3);
      display.print(" W");
      break;
    case LoadMode::CR:
      display.print("CR ");
      display.print(setpoint);
      display.print(" Ohm");
      break;
  }

  display.setCursor(0,10);
  display.print("V: ");
  display.print(meas.voltage, 3);
  display.print(" V");

  display.setCursor(0,20);
  display.print("I: ");
  display.print(meas.current, 3);
  display.print(" A");
  
  display.setCursor(0,30);
  display.print("P: ");
  display.print(meas.power, 3);
  display.print(" W");
  
  display.display();
}

// ======================================================
// display menu
// ======================================================

void oledMenu (void) { 

  if (menu == 0) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  if (menu == 1) {
    potMaped = (long) quantizeRound(floatMap(potValue, 0, 1023, 0, 5000), 100);
    display.print("CC");
    display.setCursor(0,10);
    display.print("I: ");
    display.print((float)(potMaped/1000.0), 3);
    display.print(" A");
    display.display();
    return;
  }
  
  if (menu == 2) {
    potMaped =  (long) quantizeRoundZero(floatMap(potValue, 0, 1023, 0, 150000), 1000);
    display.print("CP");
    display.setCursor(0,10);
    display.print("P: ");
    display.print((float)(potMaped/1000.0), 3);
    //display.print(potMaped);
    display.print(" W");
    display.display();
    return;
  }

  if (menu == 3) {
    potMaped = (long) quantizeRound(floatMap(potValue, 0, 1023, 0, 1000), 10);
    display.print("CR");
    display.setCursor(0,10);
    display.print("R: ");
    display.print(potMaped);
    display.print(" Ohm");
    display.display();
    return;
  }

}

#endif


// ======================================================
// READ VCC
// ======================================================

float readVcc()
{
  ADMUX =
    _BV(REFS0) |
    _BV(MUX3)  |
    _BV(MUX2)  |
    _BV(MUX1);

  delay(2);

  ADCSRA |= _BV(ADSC);

  while (bit_is_set(ADCSRA, ADSC));

  uint16_t result = ADCL;
  result |= ADCH << 8;

  return (float)VCC_CAL / (float)result / 1000.0f;
}

// ======================================================
// UPDATE MEASUREMENTS
// ======================================================

void updateMeasurements()
{
  // read internal reference voltage
  const float vcc = readVcc();

  // read raw ADC values
  const uint16_t rawVoltage = analogRead(PIN_VOLTAGE);
  const uint16_t rawCurrent = analogRead(PIN_CURRENT);

  // ADC pin voltage
  const float adcVoltage = ((float)rawVoltage / ADC_MAX) * vcc;
  const float adcCurrent = ((float)rawCurrent / ADC_MAX) * vcc;

  // real load values
  meas.voltage = adcVoltage * VOLTAGE_GAIN;
  meas.current = adcCurrent * CURRENT_GAIN;
  meas.power = meas.voltage * meas.current;
}

// ======================================================
// SET DAC CURRENT
// ======================================================

void setCurrent(float currentA)
{
  currentA = constrain(currentA, 0.0f, MAX_CURRENT_A);
  const float dacValue =  (currentA / MAX_CURRENT_A) * DAC_MAX * DAC_CAL;
  dac.setVoltage((uint16_t)dacValue, false);
}

// ======================================================
// CONTROL LOOP
// ======================================================

void updateControl()
{
  switch (mode)
  {
    // ==============================================
    // CONSTANT CURRENT
    // ==============================================

    case LoadMode::CC:
    {
      setCurrent(setpoint);
      break;
    }

    // ==============================================
    // CONSTANT POWER
    // ==============================================

    case LoadMode::CP:
    {
      if (meas.voltage < 0.05f)
      {
        setCurrent(0.0f);
        return;
      }
      float targetCurrent = setpoint / meas.voltage;
      setCurrent(targetCurrent);
      break;
    }

    // ==============================================
    // CONSTANT RESISTANCE
    // ==============================================

    case LoadMode::CR:
    {
      if (setpoint < 0.05f)
      {
          setCurrent(0.0f);
          return;
      }

      float targetCurrent =
          meas.voltage / setpoint;

      setCurrent(targetCurrent);

      break;
    }
  }
}

// ======================================================
// STATUS LED
// ======================================================

void updateLed()
{
  static bool ledState = false;
  if (millis() - lastLedToggle >= LED_INTERVAL_MS)
  {
      lastLedToggle = millis();
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
  }
}

// ======================================================
// PRINT STATUS
// ======================================================

void printMeasurements()
{
  
  //return;

  if (editingCommand)
    return;

  if (quiet)
      return;
  
  if (millis() - lastPrint < PRINT_INTERVAL_MS)
      return;

  lastPrint = millis();

  switch (mode)
  {
    case LoadMode::CC:
      Serial.print("CC");
      break;

    case LoadMode::CP:
      Serial.print("CP");
      break;

    case LoadMode::CR:
      Serial.print("CR");
      break;
  }

  Serial.print(": ");
  
  switch (mode)
  {
    case LoadMode::CC:
      Serial.print(setpoint, 3);
      Serial.print(" A");
      break;

    case LoadMode::CP:
      Serial.print(setpoint, 3);
      Serial.print(" W");
      break;

    case LoadMode::CR:
      Serial.print(setpoint, 2);
      Serial.print(" Ohm");
      break;
  }

  Serial.print(" | V: ");
  Serial.print(meas.voltage, 3);

  Serial.print(" V | I: ");
  Serial.print(meas.current, 3);

  Serial.print(" A | P: ");
  Serial.print(meas.power, 3);

  Serial.print(" W");

  // ==============================================
  // WARNINGS
  // ==============================================

  if (meas.current > MAX_CURRENT_A)
  {
    Serial.print(" | WARNING: CURRENT LIMIT");
  }

  if (meas.voltage > MAX_VOLTAGE_V)
  {
    Serial.print(" | WARNING: VOLTAGE LIMIT");
  }

  if (meas.power > MAX_POWER_W)
  {
    Serial.print(" | WARNING: POWER LIMIT");
  }

  Serial.println();
}

// ======================================================
// COMMAND PARSER
// ======================================================

void parseCommand(const char* cmd)
{
  // ==============================================
  // CONSTANT CURRENT
  // cc1.5 = 1.5A or ou cc2 = 2W
  // ==============================================

  if (strncmp(cmd, "cc", 2) == 0)
  {       
      float value = atof(cmd + 2);
      mode = LoadMode::CC;
      if (value > MAX_CURRENT_A)
      {
        Serial.println("ERROR: MAX CURRENT");
        value = MAX_CURRENT_A;
      }
      setpoint = value;
  }

  // ==============================================
  // CONSTANT POWER
  // cp0.5 = 0.5W ou cp100 = 100W
  // ==============================================

  else if (strncmp(cmd, "cp", 2) == 0)
  {
      float value = atof(cmd + 2);
      mode = LoadMode::CP;
      if (value > MAX_POWER_W)
      {
        Serial.println("ERROR: MAX POWER");
        value = MAX_POWER_W;
      }
      setpoint = value;
  }

  // ==============================================
  // CONSTANT RESISTANCE
  // cr0.22 = 0.22 ohm cr10 = 10 ohm
  // ==============================================

  else if (strncmp(cmd, "cr", 2) == 0)
  {
      float value = atof(cmd + 2);
      mode = LoadMode::CR;
      if (value < 0.1f)
      {
        Serial.println("ERROR: MIN RESISTANCE");
        value = 0.1f;
      }
      setpoint = value;
  }

  // ==============================================
  // OUTPUT OFF
  // ==============================================

  else if (strcmp(cmd, "off") == 0)
  {
    setCurrent(0.0f);
    setpoint = 0.0f;
  }
  
  // ==============================================
  // Serial info OFF
  // ==============================================
  else if (strcmp(cmd, "quiet") == 0)
  {
    quiet = !quiet;
    Serial.print("Serial info: ");
    Serial.println(quiet ? "OFF" : "ON");
  }

  // ==============================================
  // HELP
  // ==============================================

  else if (strcmp(cmd, "help") == 0)
  {
    Serial.println();
    Serial.println("Commands: CC, CP, CR, OFF, QUIET");
    Serial.println();
    Serial.println("cc0.1  -> 0.1A");
    Serial.println("cc2    -> 2A");
    Serial.println();
    Serial.println("cp0.5  -> 0.5W");
    Serial.println("cp50   -> 50W");
    Serial.println();
    Serial.println("cr0.22 -> .22 Ohm");
    Serial.println("cr10   -> 10 Ohm");
    Serial.println();
    Serial.println("off    -> disable device");
    Serial.println("quiet  -> toggle serial output");
    Serial.println();
  }

  else
  {
    Serial.println("ERROR");
  }
}

// ======================================================
// HANDLE SERIAL
// ======================================================

void handleSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();
    editingCommand = true;

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      Serial.println();
      serialLine[serialIndex] = '\0';

      if (serialIndex > 0)
      {
        parseCommand(serialLine);
      }

      serialIndex = 0;
      editingCommand = false;
    }

    // BACKSPACE
    else if (c == 8 || c == 127)
    {
      if (serialIndex > 0)
      {
        serialIndex--;
        Serial.print("\b \b");
      }
    }

    else
    {
      if (serialIndex < CMD_BUFFER_SIZE - 1)
      {
        //serialLine += c;
        serialLine[serialIndex++] = c;
        Serial.print(c);
      }
    }
  }
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200);
  Wire.begin();
  dac.begin(0x60);

  setCurrent(0.0f);

  #if defined(USE_OLED)

  // Wait for display
  delay(500);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  menu = 1;
  oledMenu(); 
  
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);

  #endif

  #if defined(USE_SPICOMMS)

  // prepare SPI data
  uint32_t values[5] = {
    0x12345678,
    1,
    2,
    3,
    4
  };
  memcpy(txData, values, sizeof(values));
  
  // setup SPI
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT_PULLUP);
  pinMode(SCK, INPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();
  txIndex = 0;
  SPDR = txData[0];

  #endif


  Serial.println();
  Serial.println("Programmable Load Ready");
  Serial.println("Type 'help'");
  Serial.println();
}

// ======================================================
// LOOP
// ======================================================

void loop()
{

  #if defined(USE_SPICOMMS)

  static bool lastSS = HIGH;
  bool ss = digitalRead(SS);
  if (ss == LOW && lastSS == HIGH)
  {
      noInterrupts();
      rxIndex = 0;
      txIndex = 1;
      SPDR = txData[0];
      interrupts();
  }

  if (ss == HIGH && lastSS == LOW)
  {
      if (rxIndex >= COMMAND_SIZE)
      {
          packetReceived = true;
      }

      if (txUpdatePending)
      {
          noInterrupts();
          memcpy(txData, txPending, STATUS_SIZE);
          txUpdatePending = false;
          interrupts();
      }
  }

  lastSS = ss;

/*
  // SPI Slave circular buffer handling
  static bool lastSS = HIGH;
  bool ss = digitalRead(SS);
  // transaction start
  if (ss == LOW && lastSS == HIGH) {
      noInterrupts();
      rxIndex = 0;
      txIndex = 0;
      SPDR = txData[0];
      interrupts();
  }

  // transaction end
  if (ss == HIGH && lastSS == LOW) {

    Serial.println(rxIndex);

      if (txUpdatePending) {

          noInterrupts();
          memcpy(txData, txPending, sizeof(txData));
          txUpdatePending = false;
          interrupts();
      }
  }

  lastSS = ss;
*/
  
  packetReceived = false;
  if (packetReceived) {

    noInterrupts();
    uint8_t buffer[12];
    memcpy(buffer, (const void*)rxData, sizeof(buffer));
    packetReceived = false;
    interrupts();

    uint32_t rxSync =
      ((uint32_t)buffer[0]) |
      ((uint32_t)buffer[1] << 8) |
      ((uint32_t)buffer[2] << 16) |
      ((uint32_t)buffer[3] << 24);

    uint32_t rxCmd =
      ((uint32_t)buffer[4]) |
      ((uint32_t)buffer[5] << 8) |
      ((uint32_t)buffer[6] << 16) |
      ((uint32_t)buffer[7] << 24);

    uint32_t rxArg =
      ((uint32_t)buffer[8]) |
      ((uint32_t)buffer[9] << 8) |
      ((uint32_t)buffer[10] << 16) |
      ((uint32_t)buffer[11] << 24);

    if (rxSync != 0x55AA55AA)
    {
        Serial.println("BAD SYNC");
        return;
    }

    // apply command  
    switch (rxCmd)
    {
      case 1:
        mode = LoadMode::CC;
        setpoint = rxArg / 1000.0f;
        break;

      case 2:
        mode = LoadMode::CP;
        setpoint = rxArg / 1000.0f;
        break;

      case 3:
        mode = LoadMode::CR;
        setpoint = rxArg / 1000.0f;
        break;

      case 4:
        setCurrent(0);
        break;

      case 5:
        uint32_t txPacket[5];

        /*
        txPacket[0] = (uint32_t)mode;
        txPacket[1] = (uint32_t)(setpoint * 1000.0f);
        txPacket[2] = (uint32_t)(meas.voltage * 1000.0f);
        txPacket[3] = (uint32_t)(meas.current * 1000.0f);
        */

        txPacket[0] = 0xAA55AA55;
        txPacket[1] = (uint32_t)1; // mode dummy
        txPacket[2] = (uint32_t)2; // setpoint dummy
        txPacket[3] = (uint32_t)3; // voltage dummy
        txPacket[4] = (uint32_t)4; // current dummy

        memcpy(txPending, txPacket, sizeof(txPacket));

        txUpdatePending = true;

        break;

      case 6:
        quiet = !quiet;
        break;
    }
        
  }

  #endif


  #if defined(USE_OLED)

  static bool oldUp = HIGH;
  static bool oldOk = HIGH;

  bool up = digitalRead(BTN_UP);
  bool ok = digitalRead(BTN_OK);

  // detectar clique UP no menu
  if(oldUp == HIGH && up == LOW) {
      menu++;
      if(menu > 3)
          menu = 1;
      delay(20);
  }

  // detectar clique OK
  if(oldOk == HIGH && ok == LOW) {
      switch (menu) {
        case 1:
          mode = LoadMode::CC;
          setpoint = potMaped;
          break;
        case 2:
          mode = LoadMode::CP;
          setpoint = potMaped; 
          break;
        case 3:
          mode = LoadMode::CR;
          setpoint = potMaped;
          break;
      }

      if (menu != 0) {
        menu = 0;
        oledInfo ();
      }

      delay(30);
  }

  // guardar estado dos botões para detectar mudanças
  oldUp = up;
  oldOk = ok;

  if (menu != 0) {
    potValue = analogRead(potpin);
    oledMenu();
  }

  #endif

  updateMeasurements();
  handleSerial();




  if (millis() - lastControl >= CONTROL_INTERVAL_MS)
  {
    lastControl = millis();
    updateControl();
  }

  updateLed();
  printMeasurements();
  oledInfo();
}
