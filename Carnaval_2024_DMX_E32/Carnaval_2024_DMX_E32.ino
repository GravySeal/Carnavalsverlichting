#include <Arduino.h>
#include "DmxInput.h"
DmxInput dmxInput;

#define START_CHANNEL 0
#define NUM_CHANNELS 512

#define CHANNEL_OFFSET 477 //36 channels remaining

#define BAUD_RATE 115200//19200

#define RE1 2//228
#define DE1 3//527

#define RE2 21//276
#define DE2 20//267

//Grootte is 513, maar 1e is altijd leeg.
volatile uint8_t dmx_buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

uint8_t uart_buffer[36];

// Interrupt wanneer DMX data wordt ontvangen...
void __isr dmxDataReceived(DmxInput* dmxInput) {

  //Do something...
  //memcpy(&buffer_to_send, &dmx_buffer, sizeof(dmx_buffer));
  //maybe do nothing?
  //Toggle LED
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

}

void setup()
{
    // Setup our DMX Input to read on GPIO 0, from channel 1 to 3
    dmxInput.begin(1, START_CHANNEL, NUM_CHANNELS);

    pinMode(RE1, OUTPUT);
    pinMode(DE1, OUTPUT);
    pinMode(RE2, OUTPUT);
    pinMode(DE2, OUTPUT);

    digitalWrite(RE1, LOW);  // sets the digital pin RE1 off
    digitalWrite(DE1, LOW);  // sets the digital pin DE1 off
    digitalWrite(RE2, HIGH);  // sets the digital pin RE2 off
    digitalWrite(DE2, HIGH);  // sets the digital pin DE2 off

    //Serial
    Serial2.begin(BAUD_RATE);

    // Setup the onboard LED so that we can blink when we receives packets
    pinMode(LED_BUILTIN, OUTPUT);

    //dmxInput.read_async(dmx_buffer, dmxDataReceived);
    //dmxInput.read_async(dmx_buffer);
}

void loop()
{
  dmxInput.read(dmx_buffer);
  for (uint i = CHANNEL_OFFSET; i < sizeof(dmx_buffer); i++)
    {
      uart_buffer[i-CHANNEL_OFFSET] = dmx_buffer[i];
      //USB Debug
      //Serial.print(uart_buffer[i-CHANNEL_OFFSET]);
      //Serial.print(dmx_buffer[i]);
    }
  //USB debug
  //Serial.println("");

  //digitalWrite(LED_BUILTIN, HIGH);

  //Actual sending through MAX485
  Serial2.write(uart_buffer, sizeof(uart_buffer));

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  delay(44); //Verander baudrate van E32 zender... Dit is te langzaaam.
  // Blink the LED to indicate that a packet was received
  //digitalWrite(LED_BUILTIN, LOW);
}