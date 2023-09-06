#include <Arduino.h>
#include "DmxInput.h"
DmxInput dmxInput;

#define START_CHANNEL 1
#define NUM_CHANNELS 512

#define BAUD_RATE 19200

uint8_t dmx_buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

uint8_t buffer_to_send[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

uint8_t uart_buffer[58];

bool transmission_finished;


// Interrupt wanneer DMX data wordt ontvangen...
void __isr dmxDataReceived(DmxInput* dmxInput) {

  //Do something...
  if (transmission_finished == true) {
    memcpy(&buffer_to_send, &dmx_buffer, sizeof(dmx_buffer));
    transmission_finished = false;
  }

}

void setup() {
  // Setup the onboard LED so that we can blink when we receives packets
  pinMode(LED_BUILTIN, OUTPUT);


  //Serial
  Serial1.setRX(5);//ik gebruik hier GP5 en GP4, maar idk of die nummers gebruikt moeten worden of de actuele pin nummers.
  Serial1.setTX(4);
  Serial1.begin(BAUD_RATE);

  // Setup our DMX Input to read on GPIO 0, from channel 1 to 3
  dmxInput.begin(1, START_CHANNEL, NUM_CHANNELS);

  dmxInput.read_async(dmx_buffer, dmxDataReceived);
}

void loop() {
  // Do nothing...

  while(!transmission_finished){

    //No. of transmissions
    uint8_t num_of_transmissions = NUM_CHANNELS/56;
    if (NUM_CHANNELS % 56 > 0) {
      num_of_transmissions += 1;
    }

    //For each part of the array transmit data
    for (uint8_t i = 0; i <= num_of_transmissions; i++) {
      uart_buffer[0] = i;
      uart_buffer[1] = num_of_transmissions;
      uint8_t offset = i * 56;
      memcpy(&uart_buffer[2], &buffer_to_send[offset], sizeof(uart_buffer)-2);
      //now send that binch in blocking mode before sending again.
      Serial1.write(uart_buffer, sizeof(uart_buffer));
    }
    
    transmission_finished = true;
  }
  
}
