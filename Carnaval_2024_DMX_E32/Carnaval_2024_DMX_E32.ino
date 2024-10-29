#include <Arduino.h>
#include "DmxInput.h"
DmxInput dmxInput;

#define START_CHANNEL 0
#define NUM_CHANNELS 512

#define CHANNEL_OFFSET 507

#define BAUD_RATE 19200

#define RE1 2//228
#define DE1 3//527

#define RE2 21//276
#define DE2 20//267

volatile uint8_t buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

uint8_t uart_buffer[6];

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

    for (uint i = CHANNEL_OFFSET; i < sizeof(buffer); i++)
    {
        
        uart_buffer[i-CHANNEL_OFFSET] = buffer[i];

        Serial.print(uart_buffer[i-CHANNEL_OFFSET]);

    }

    Serial.println("");

    Serial2.write(uart_buffer, sizeof(uart_buffer));

    // Blink the LED to indicate that a packet was received
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
}