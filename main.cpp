
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#include <SPI.h>

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
constexpr float CURRENT_GAIN   = 1.034f;

// DAC current calibration
constexpr float DAC_CAL        = 1.006f;

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
volatile uint8_t txIndex = 0;
volatile uint8_t rxIndex = 0;

uint8_t txData[16];
volatile uint8_t rxData[16];

volatile bool packetReceived = false;

// ======================================================
// SPI ISR
// ======================================================
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

  const float dacValue =
    (currentA / MAX_CURRENT_A)
    * DAC_MAX
    * DAC_CAL;

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

  // prepare SPI data
  uint32_t values[4] = {
    1000,
    2000,
    3000,
    4000
  };
  memcpy(txData, values, sizeof(values));
  
  // setup SPI
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();
  txIndex = 0;
  SPDR = txData[0];

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

  // SPI Slave circular buffer handling
  static bool lastSS = HIGH;
  bool ss = digitalRead(SS);
  if (ss == LOW && lastSS == HIGH) {
      noInterrupts();
      rxIndex = 0;
      txIndex = 0;
      SPDR = txData[0];
      interrupts();
  }
  lastSS = ss;

  updateMeasurements();
  handleSerial();

  if (packetReceived) {
    noInterrupts();
    uint8_t buffer[16];
    memcpy(buffer, (const void*)rxData, sizeof(buffer));
    packetReceived = false;
    interrupts();

    // debug print
    Serial.print("RX: ");
    for (uint8_t i = 0; i < sizeof(buffer); i++) {
      Serial.print(buffer[i]);
      Serial.print(' ');
    }
    Serial.println();

    // get cmd and arg
    uint32_t rxCmd =
      ((uint32_t)buffer[0]) |
      ((uint32_t)buffer[1] << 8) |
      ((uint32_t)buffer[2] << 16) |
      ((uint32_t)buffer[3] << 24);

    uint32_t rxArg =
      ((uint32_t)buffer[4]) |
      ((uint32_t)buffer[5] << 8) |
      ((uint32_t)buffer[6] << 16) |
      ((uint32_t)buffer[7] << 24);

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

      /*
      case 5:
      noInterrupts();

      txPacket.mode = (uint8_t)mode;
      txPacket.setpoint = setpoint * 1000.0f;
      txPacket.voltage_mV = meas.voltage * 1000.0f;
      txPacket.current_mA = meas.current * 1000.0f;
      txPacket.power_mW = meas.power * 1000.0f;

      uint8_t* txPtr = (uint8_t*)&txPacket;

      spiTxIndex = 0;
      SPDR = txPtr[0];

      interrupts();
        break;
      */

      case 6:
        quiet = !quiet;
        break;
    }
        
  }

  if (millis() - lastControl >= CONTROL_INTERVAL_MS)
  {
    lastControl = millis();
    updateControl();
  }

  updateLed();
  printMeasurements();
}
