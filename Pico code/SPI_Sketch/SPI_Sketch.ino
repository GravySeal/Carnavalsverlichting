
#include "SPI.h"
#include "Arduino.h"

#define dataOrder = MSBFIRST;
#define SPIMode = SPI_MODE0;

//#define SPI_PORT spi0

const int _MOSI = 19;
const int _MISO = 16;
const int _CS = 17;
const int _SCK = 18;

const int msec_to_wait = 1000;

char message[] = "henlo!!!";

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  
  //Setup SPI
  SPI.setSCK(_SCK);
  SPI.setCS(_CS);
  SPI.setRX(_MISO);
  SPI.setTX(_MOSI);

  SPI.begin();
  pinMode(_CS, OUTPUT);
  digitalWrite(_CS, HIGH);

  //Start SPI
  
}

void loop() {
  
  //// Wait seconds...
  //delay(msec_to_wait);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // Tell friend we're sending
  digitalWrite(_CS, LOW);

  // Message to send
  for (int i = 0; i < sizeof(message); i++) {
    SPI.transfer(message[i]);  // Send each character in the message
  }

  // Tell we're done sending
  digitalWrite(_CS, HIGH);
  SPI.endTransaction();

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
