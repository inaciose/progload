#include <Arduino.h>
#include <SPI.h>

constexpr uint8_t SS_PIN = 10;

struct ReadPacket {
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
};

uint8_t txCounter = 0;

void setup() {
  Serial.begin(115200);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  SPI.begin();
  SPI.beginTransaction(
    SPISettings(1000, MSBFIRST, SPI_MODE0)
  );
  Serial.println("MASTER READY");
}

void loop() {

  ReadPacket rx;
  uint8_t* ptr = (uint8_t*)&rx;
  digitalWrite(SS_PIN, LOW);
  // skip first byte
  SPI.transfer(0x00);

  // enviar bytes incrementais e receber resposta
  for (uint8_t i = 0; i < sizeof(ReadPacket); i++) {
      ptr[i] = SPI.transfer(txCounter);
      txCounter++;
  }
  digitalWrite(SS_PIN, HIGH);

  Serial.println("RESPONSE:");

  Serial.println(rx.a);
  Serial.println(rx.b);
  Serial.println(rx.c);
  Serial.println(rx.d);

  Serial.println();

  delay(1000);
}
