#include <Arduino.h>
#include "DmxInput.h"

#define START_CHANNEL 0
#define NUM_CHANNELS 512

#define SENDER_NUMBER 0

#define BAUD_RATE 115200

#define RE1 2//228
#define DE1 3//527

#define RE2 21//276
#define DE2 20//267

DmxInput dmxInput;

volatile uint8_t buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

uint8_t uart_buffer[54]; //Set ebyte to transparent transmission with 115200 baud uart speed

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
}

void loop()
{
    // Wait for next DMX packet
    dmxInput.read(buffer);

    for (uint i = 0; i < 54; i++)
    {
        uart_buffer[i] = buffer[54 * SENDER_NUMBER + i]; //54*0 = 0 dus eerste 54 DMX kanalen, 54*1 = 54 dus 54 tot 108 etc. etc.
        Serial.print(uart_buffer[i]);
    }

    Serial.println("");
    Serial2.write(uart_buffer, 54);

    // Blink the LED to indicate that a packet was received
    digitalWrite(LED_BUILTIN, HIGH);
    delay(60);
    digitalWrite(LED_BUILTIN, LOW);
}
