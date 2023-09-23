#include <Arduino.h>
#include "DmxInput.h"
DmxInput dmxInput;

#define START_CHANNEL 1
#define NUM_CHANNELS 56

#define BAUD_RATE 19200

#define RE1 28
#define DE1 27

#define RE2 6
#define DE2 7

uint8_t dmx_buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

uint8_t buffer_to_send[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

uint8_t uart_buffer[58];

volatile bool transmission_finished;

volatile int numba;

void setup() {
  // Setup the onboard LED so that we can blink when we receives packets
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RE1, OUTPUT);
  pinMode(DE1, OUTPUT);
  pinMode(RE2, OUTPUT);
  pinMode(DE2, OUTPUT);

  digitalWrite(RE1, HIGH);  // sets the digital pin RE1 off
  digitalWrite(DE1, HIGH);  // sets the digital pin DE1 off
  digitalWrite(RE2, LOW);  // sets the digital pin RE2 off
  digitalWrite(DE2, LOW);  // sets the digital pin DE2 off

  //Serial
  Serial2.begin(BAUD_RATE);

  // Setup our DMX Input to read on GPIO 0, from channel 1 to 3
  dmxInput.begin(1, START_CHANNEL, NUM_CHANNELS);
  numba = 0;
}

void loop() {
  // Do nothing...

  //dmxInput.read(dmx_buffer);

    //Do something...
  if (transmission_finished == true) {
    uint8_t test_buffer[] = {0xFF, 0x00 , 0xFF, 0x32, 0x00, 0xFF, 0x7B, 0xFF};

    numba = numba+1;
    
    switch (numba) {
      case 1:
        // Define a new array for case 1
        static uint8_t case1_buffer[] = {0xFF, 0x00, 0xFF, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case1_buffer, sizeof(case1_buffer));
        break;
      case 2:
        // Define a new array for case 2
        static uint8_t case2_buffer[] = {0xFF, 0x00, 0xC8, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case2_buffer, sizeof(case2_buffer));
        break;
      case 3:
        // Define a new array for case 3
        static uint8_t case3_buffer[] = {0xFF, 0x00, 0x7B, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case3_buffer, sizeof(case3_buffer));
        break;
      case 4:
        // Define a new array for case 4
        static uint8_t case4_buffer[] = {0xFF, 0x00, 0x32, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case4_buffer, sizeof(case4_buffer));
        break;
      case 5:
        // Define a new array for case 4
        static uint8_t case5_buffer[] = {0xFF, 0x00, 0x00, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case5_buffer, sizeof(case5_buffer));
        break;
      case 6:
        // Define a new array for case 4
        static uint8_t case6_buffer[] = {0xFF, 0x00, 0x00, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case6_buffer, sizeof(case6_buffer));
        break;
      case 7:
        // Define a new array for case 4
        static uint8_t case7_buffer[] = {0xFF, 0x00, 0x32, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case7_buffer, sizeof(case7_buffer));
        break;
      case 8:
        // Define a new array for case 4
        static uint8_t case8_buffer[] = {0xFF, 0x00, 0x7B, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case8_buffer, sizeof(case8_buffer));
        break;
      case 9:
        // Define a new array for case 4
        static uint8_t case9_buffer[] = {0xFF, 0x00, 0xC8, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case9_buffer, sizeof(case9_buffer));
        break;
      case 10:
        // Define a new array for case 4
        static uint8_t case10_buffer[] = {0xFF, 0x00, 0xFF, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, case10_buffer, sizeof(case10_buffer));
        numba = 0;
        break;
      default:
        // Define a new array for the default case
        static uint8_t default_buffer[] = {0xFF, 0x00, 0xFF, 0x32, 0x00, 0xFF, 0x7B, 0xFF};
        memcpy(test_buffer, default_buffer, sizeof(default_buffer));
    }

    memcpy(&buffer_to_send, &test_buffer, sizeof(test_buffer));

    transmission_finished = false;
  }

  while(!transmission_finished){

    //No. of transmissions
    uint8_t num_of_transmissions = NUM_CHANNELS/56;
    if (NUM_CHANNELS % 56 > 0) {
      num_of_transmissions += 1;
    }

    //For each part of the array transmit data
    for (uint8_t i = 0; i < num_of_transmissions; i++) {
      uart_buffer[0] = i+1;
      uart_buffer[1] = num_of_transmissions;
      uint16_t offset = i * 56;
      memcpy(&uart_buffer[2], &buffer_to_send[offset], 56);
      //now send that binch in blocking mode before sending again.
      Serial2.write(uart_buffer, 58);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(88);
      digitalWrite(LED_BUILTIN, LOW); 
    }
    transmission_finished = true;
  }
  
}
