#include <Arduino.h>
#include <SPI.h>

// ======================================================
// SPI
// ======================================================

constexpr uint8_t PIN_SS = 10;

// ======================================================
// COMMANDS
// ======================================================

enum Command : uint8_t
{
    CMD_CC    = 1,
    CMD_CP    = 2,
    CMD_CR    = 3,
    CMD_OFF   = 4,
    CMD_GET   = 5,
    CMD_QUIET = 6
};

// ======================================================
// STATUS PACKET
// ======================================================
struct __attribute__((packed)) CommandPacket
{
    uint32_t sync;
    uint32_t cmd;
    uint32_t arg;
};

struct __attribute__((packed)) ReadPacket
{
    uint32_t sync;
    uint32_t mode;
    uint32_t setpoint;
    uint32_t voltage;
    uint32_t current;
};

// ======================================================
// GLOBALS
// ======================================================

ReadPacket rxPacket;
uint8_t txCounter = 0;
bool quietMode = false;
uint8_t currentMode = 5;
uint32_t currentArg = 0;


// ======================================================
// CLI
// ======================================================

constexpr uint8_t CMD_BUFFER_SIZE = 64;

char serialLine[CMD_BUFFER_SIZE];
uint8_t serialIndex = 0;
bool editingCommand = false;

// ======================================================
// REQUEST STATUS
// ======================================================

void requestStatus(uint8_t cmd, uint32_t arg = 0)
{
    CommandPacket tx;

    memset(&tx, 0, sizeof(tx));

    tx.sync = 0x12345678;
    tx.cmd  = cmd;
    tx.arg  = arg;

    uint8_t* txPtr = (uint8_t*)&tx;

    uint8_t raw[20];

    SPI.beginTransaction(
        SPISettings(200000, MSBFIRST, SPI_MODE0)
    );

    digitalWrite(PIN_SS, LOW);

    delayMicroseconds(15);

    // TX 12 bytes
    // RX simultâneo
    for (uint8_t i = 0; i < 12; i++)
    {
        raw[i] = SPI.transfer(txPtr[i]);
    }

    // clocks restantes
    for (uint8_t i = 12; i < 20; i++)
    {
        raw[i] = SPI.transfer(0x00);
    }

    digitalWrite(PIN_SS, HIGH);

    SPI.endTransaction();

    memcpy(&rxPacket, &raw[0], sizeof(ReadPacket));

    /*
    Serial.print("RAW: ");

    for (uint8_t i = 0; i < 20; i++)
    {
        if (raw[i] < 16)
            Serial.print('0');

        Serial.print(raw[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
    */
    
}

// ======================================================
// PRINT STATUS
// ======================================================

void printStatus()
{

    //Serial.print("SYNC = 0x");
    //Serial.println(rxPacket.sync, HEX);

    if (rxPacket.sync != 0x12345678)
    {
        Serial.println("ERROR: BAD SYNC");
        return;
    }

    Serial.print("");
    Serial.print(rxPacket.mode);

    Serial.print(" | ");
    Serial.print(rxPacket.setpoint);

    Serial.print(" | ");
    Serial.print(rxPacket.voltage / 1000.0f, 3);
    Serial.print(" V");

    Serial.print(" | ");
    Serial.print(rxPacket.current / 1000.0f, 3);
    Serial.print(" A");

    Serial.println();
  }

// ======================================================
// PARSE COMMAND
// ======================================================

void parseCommand(const char* cmd)
{
    // ==========================================
    // CC
    // ==========================================

    if (strncmp(cmd, "cc", 2) == 0)
    {
        float value = atof(cmd + 2);
        currentMode = CMD_CC;
        currentArg = (uint32_t)(value * 1000.0f);
        requestStatus(CMD_CC, (uint32_t)(value * 1000.0f));
        requestStatus(CMD_CC, (uint32_t)(value * 1000.0f));
        requestStatus(CMD_CC, (uint32_t)(value * 1000.0f));
    }

    // ==========================================
    // CP
    // ==========================================

    else if (strncmp(cmd, "cp", 2) == 0)
    {
        float value = atof(cmd + 2);
        currentMode = CMD_CP;
        currentArg = (uint32_t)(value * 1000.0f);
        requestStatus(CMD_CP, (uint32_t)(value * 1000.0f));
        requestStatus(CMD_CP, (uint32_t)(value * 1000.0f));
        requestStatus(CMD_CP, (uint32_t)(value * 1000.0f));

    }

    // ==========================================
    // CR
    // ==========================================

    else if (strncmp(cmd, "cr", 2) == 0)
    {
        float value = atof(cmd + 2);
        currentMode = CMD_CR;
        currentArg = (uint32_t)(value * 1000.0f);
        requestStatus(CMD_CR, (uint32_t)(value * 1000.0f));
        requestStatus(CMD_CP, (uint32_t)(value * 1000.0f));
        requestStatus(CMD_CP, (uint32_t)(value * 1000.0f));
    }

    // ==========================================
    // OFF
    // ==========================================

    else if (strcmp(cmd, "off") == 0)
    {
        currentMode = CMD_CR;
        requestStatus(CMD_OFF);
    }

    // ==========================================
    // QUIET
    // ==========================================

    else if (strcmp(cmd, "quiet") == 0)
    {
        quietMode = !quietMode;
        requestStatus(CMD_QUIET);
        Serial.print("Quiet mode: ");

        Serial.println(
            quietMode ? "ON" : "OFF"
        );
    }

    // ==========================================
    // STATUS
    // ==========================================

    else if (strcmp(cmd, "status") == 0)
    {
        requestStatus(CMD_GET);
        printStatus();
    }

    // ==========================================
    // HELP
    // ==========================================

    else if (strcmp(cmd, "help") == 0)
    {
        Serial.println();
        Serial.println("Commands:");
        Serial.println();

        Serial.println("cc0.5");
        Serial.println("cp2.5");
        Serial.println("cr100");
        Serial.println("off");
        Serial.println("quiet");
        Serial.println("status");

        Serial.println();
    }

    // ==========================================
    // UNKNOWN
    // ==========================================

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

        // ENTER
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

        // NORMAL CHAR
        else
        {
            if (serialIndex < CMD_BUFFER_SIZE - 1)
            {
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
    Serial.begin(115200);

    pinMode(PIN_SS, OUTPUT);

    digitalWrite(PIN_SS, HIGH);

    SPI.begin();

    delay(1000);

    Serial.println();
    Serial.println("SPI MASTER READY");
    Serial.println();

    Serial.println("Type 'help'");
    Serial.println();
}

// ======================================================
// LOOP
// ======================================================

uint32_t lastStatus = 0;

void loop()
{
    handleSerial();

    if (editingCommand)
        return;

    if (!quietMode)
    {
        if (millis() - lastStatus >= 1000)
        {
            lastStatus = millis();

            //requestStatus(currentMode, currentArg);
            requestStatus(CMD_GET, 0);

            printStatus();
        }
    }
}
