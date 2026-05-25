
#include <Arduino.h>
#include <SPI.h>

volatile uint8_t txIndex = 0;
volatile uint8_t rxIndex = 0;

uint8_t txData[16];
volatile uint8_t rxData[16];

volatile bool packetReceived = false;

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

void setup() {

  Serial.begin(115200);
  uint32_t values[4] = {
    1000,
    2000,
    3000,
    4000
  };

  memcpy(txData, values, sizeof(values));
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();
  txIndex = 0;
  SPDR = txData[0];
  Serial.println("SLAVE READY");
}

void loop() {

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

  if (packetReceived) {
    noInterrupts();
    uint8_t buffer[16];
    memcpy(buffer, (const void*)rxData, sizeof(buffer));
    packetReceived = false;
    interrupts();

    Serial.print("RX: ");
    for (uint8_t i = 0; i < sizeof(buffer); i++) {
      Serial.print(buffer[i]);
      Serial.print(' ');
    }
    Serial.println();
  }
}

