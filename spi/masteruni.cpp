#include<SPI.h>
unsigned short x = 0x1234;

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128); //bit rate = 16 MHz/128 = 125 kbit/sec
  digitalWrite(SS, LOW);   //Slave is selected
}

void loop()
{
  SPI.transfer(highByte(x));
  SPI.transfer(lowByte(x));
  //-----------------------
  Serial.println(x, HEX);
  delay(3000);  //test interval
}
*/


/*
#include <SPI.h>

constexpr uint8_t SS_PIN = 10;

struct ReadPacket {
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
};

void setup() {

  Serial.begin(115200);

  pinMode(SS_PIN, OUTPUT);

  digitalWrite(SS_PIN, HIGH);

  SPI.begin();

  SPI.beginTransaction(
    //SPISettings(125000, MSBFIRST, SPI_MODE0)
    SPISettings(4000, MSBFIRST, SPI_MODE0)
  );

  Serial.println("MASTER READY");
}

void loop() {

  ReadPacket rx;

  uint8_t* ptr = (uint8_t*)&rx;

  digitalWrite(SS_PIN, LOW);

  // descartar primeiro byte
  SPI.transfer(0x00);

  // ler dados reais
  for (uint8_t i = 0; i < sizeof(ReadPacket); i++) {

      ptr[i] = SPI.transfer(0x00);
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
