//Arduino-TFT_eSPI board-template main routine. There's a TFT_eSPI create+flush driver already in LVGL-9.1 but we create our own here for more control (like e.g. 16-bit color swap).

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <DmxInput.h>
#include <ebyte_e32.h>
#include <EEPROM.h>

/*Don't forget to set Sketchbook location in File/Preferences to the path of your UI project (the parent foder of this INO file)*/

//DMX
#define START_CHANNEL 1
#define NUM_CHANNELS 512
//DMX MAX485
#define RE1 2//228
#define DE1 3//527
//E32 MAX485
#define RE2 21//276
#define DE2 20//267
//BAUD RATE FOR E32
#define BAUD_RATE 115200
//ARRAY SIZE TO SEND
#define ARRAY_SIZE 54
//Persistent storage addresses
#define FLASH_INITIALISED_ADDR 0
#define FLASH_UART_BAUD_ADDR 1
#define FLASH_UART_PARITY_ADDR 2
#define FLASH_AIRRATE_ADDR 3
#define FLASH_POWER_ADDR 4
#define FLASH_FEC_ADDR 5
#define FLASH_TRANSPARENCY_ADDR 6
#define FLASH_WAKE_UP_TIME_ADDR 7
#define FLASH_IO_MODE_ADDR 8
#define FLASH_CHANNEL_ADDR 9
#define FLASH_SEND_DELAY_ADDR 10
#define FLASH_DMX_CHANNEL_OFFSET_ADDR 11

#define DEBUG_DATA 1

bool core1_separate_stack = true;

volatile bool is_configuring = false;

bool async_dmx = true;

DmxInput dmxInput;

uint8_t sender_number = 0;

uint8_t delay_amount = 64;

unsigned long start_time = 0;

int start_index = 0;

volatile uint8_t dmx_buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

uint8_t uart_buffer[54] = {}; //Set ebyte to transparent transmission with 115200 baud uart speed

int32_t data_array[54] = {};

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];
static lv_color_t buf2 [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI( screenWidth, screenHeight ); /* TFT instance */


void __isr dmxDataReceived(DmxInput* instance) {
  // A DMX frame has been received :-)
  memcpy(uart_buffer, (const uint8_t*)&dmx_buffer[start_index], 54);

  if (millis() - start_time >= delay_amount && !is_configuring) {

    digitalWrite(LED_BUILTIN, HIGH);

    Serial2.write(uart_buffer, 54);

    start_time = millis();
  }
}


/* Display flushing */
void my_disp_flush (lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap)
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( (uint16_t*) pixelmap, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read (lv_indev_t * indev_driver, lv_indev_data_t * data)
{
    uint16_t touchX = 0, touchY = 0;

    //bool touched = false;
    bool touched = tft.getTouch( &touchX, &touchY, 500 );

    if (!touched)
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = screenWidth - touchX;
        data->point.y = touchY;
    }
}

/*Set tick routine needed for LVGL internal timings*/
static uint32_t my_tick_get_cb (void) { return millis(); }


void updateChart(uint8_t data[54]){

      for (int i = 0; i < 54; i++) {
        data_array[i] = (int32_t)data[i];
      }

      lv_chart_refresh(ui_Chart1);
}

EbyteSettings decodeEbyteSettings(const uint8_t* rx_buffer) {
    EbyteSettings settings;

    // Decode settings from rx_buffer
    settings.head = static_cast<Head>(rx_buffer[0]); // First byte is head

    // rx_buffer[1] and rx_buffer[2] are unused

    // Decode parity, baud, and air_rate from byte 3
    settings.parity = static_cast<UartParity>((rx_buffer[3] >> 6) & 0x03); // Top 2 bits
    settings.baud = static_cast<UartBaudRate>((rx_buffer[3] >> 3) & 0x07); // Middle 3 bits
    settings.air_rate = static_cast<AirRate>(rx_buffer[3] & 0x07);         // Bottom 3 bits

    // Decode channel from byte 4
    settings.channel = static_cast<CommunicationChannel>(rx_buffer[4]);

    // Decode mode, drive, wake, FEC, and power from byte 5
    settings.mode = static_cast<TransparencyMode>((rx_buffer[5] >> 7) & 0x01); // Bit 7
    settings.drive = static_cast<IoDrive>((rx_buffer[5] >> 6) & 0x01);         // Bit 6
    settings.wake = static_cast<WakeUpTime>((rx_buffer[5] >> 3) & 0x07);       // Bits 3-5
    settings.fec = static_cast<ForwardErrorCorrection>((rx_buffer[5] >> 1) & 0x01); // Bit 1
    settings.power = static_cast<TransmissionPower>(rx_buffer[5] & 0x01);     // Bit 0

    return settings;
}

void printEbyteSettings(const EbyteSettings& settings) {
    Serial.println("EbyteSettings:");

    // Print head
    Serial.print("  Head: ");
    Serial.println(settings.head == SAVE_ON_EXIT ? "SAVE_ON_EXIT" : "NO_SAVE_ON_EXIT");

    // Print parity
    Serial.print("  Parity: ");
    switch (settings.parity) {
        case NO_PARITY: Serial.println("NO_PARITY"); break;
        case ODD_PARITY: Serial.println("ODD_PARITY"); break;
        case EVEN_PARITY: Serial.println("EVEN_PARITY"); break;
    }

    // Print baud rate
    Serial.print("  Baud Rate: ");
    switch (settings.baud) {
        case BAUD_1200: Serial.println("1200"); break;
        case BAUD_2400: Serial.println("2400"); break;
        case BAUD_4800: Serial.println("4800"); break;
        case BAUD_9600: Serial.println("9600"); break;
        case BAUD_19200: Serial.println("19200"); break;
        case BAUD_38400: Serial.println("38400"); break;
        case BAUD_57600: Serial.println("57600"); break;
        case BAUD_115200: Serial.println("115200"); break;
    }

    // Print air rate
    Serial.print("  Air Rate: ");
    switch (settings.air_rate) {
        case AIR_RATE_2400: Serial.println("2400"); break;
        case AIR_RATE_2400_1: Serial.println("2400_1"); break;
        case AIR_RATE_2400_2: Serial.println("2400_2"); break;
        case AIR_RATE_4800: Serial.println("4800"); break;
        case AIR_RATE_9600: Serial.println("9600"); break;
        case AIR_RATE_19200: Serial.println("19200"); break;
        case AIR_RATE_19200_1: Serial.println("19200_1"); break;
        case AIR_RATE_19200_2: Serial.println("19200_2"); break;
    }

    // Print channel
    Serial.print("  Channel: ");
    Serial.println(settings.channel);

    // Print mode
    Serial.print("  Mode: ");
    Serial.println(settings.mode == TRANSPARENT ? "TRANSPARENT" : "FIXED");

    // Print drive
    Serial.print("  Drive: ");
    Serial.println(settings.drive == OPEN_COLLECTOR ? "OPEN_COLLECTOR" : "PUSH_PULL");

    // Print wake time
    Serial.print("  Wake Time: ");
    switch (settings.wake) {
        case WAKE_250MS: Serial.println("250ms"); break;
        case WAKE_500MS: Serial.println("500ms"); break;
        case WAKE_750MS: Serial.println("750ms"); break;
        case WAKE_1000MS: Serial.println("1000ms"); break;
        case WAKE_1250MS: Serial.println("1250ms"); break;
        case WAKE_1500MS: Serial.println("1500ms"); break;
        case WAKE_1750MS: Serial.println("1750ms"); break;
        case WAKE_2000MS: Serial.println("2000ms"); break;
    }

    // Print FEC
    Serial.print("  FEC: ");
    Serial.println(settings.fec == FEC_OFF ? "OFF" : "ON");

    // Print power
    Serial.print("  Transmission Power: ");
    switch (settings.power) {
        case POWER_30DBM: Serial.println("30dBm"); break;
        case POWER_27DBM: Serial.println("27dBm"); break;
        case POWER_24DBM: Serial.println("24dBm"); break;
        case POWER_21DBM: Serial.println("21dBm"); break;
    }
}

bool isArrayZero(const uint8_t *arr, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (arr[i] != 0) {
            return false; // Found a non-zero element
        }
    }
    return true; // All elements are zero
}

void settingsToFlash(EbyteSettings settings){
  EEPROM[FLASH_UART_BAUD_ADDR] = settings.baud;
  EEPROM[FLASH_UART_PARITY_ADDR] = settings.parity;
  EEPROM[FLASH_AIRRATE_ADDR] = settings.air_rate;
  EEPROM[FLASH_POWER_ADDR] = settings.power;
  EEPROM[FLASH_FEC_ADDR] = settings.fec;
  EEPROM[FLASH_TRANSPARENCY_ADDR] = settings.mode;
  EEPROM[FLASH_WAKE_UP_TIME_ADDR] = settings.wake;
  EEPROM[FLASH_IO_MODE_ADDR] = settings.drive;
  EEPROM[FLASH_CHANNEL_ADDR] = settings.channel;
  EEPROM.commit();
}

EbyteSettings flashToSettings(){
  EbyteSettings settings;
  settings.baud = static_cast<UartBaudRate>(EEPROM[FLASH_UART_BAUD_ADDR]);
  settings.parity = static_cast<UartParity>(EEPROM[FLASH_UART_PARITY_ADDR]);
  settings.air_rate = static_cast<AirRate>(EEPROM[FLASH_AIRRATE_ADDR]);
  settings.power = static_cast<TransmissionPower>(EEPROM[FLASH_POWER_ADDR]);
  settings.fec = static_cast<ForwardErrorCorrection>(EEPROM[FLASH_FEC_ADDR]);
  settings.mode = static_cast<TransparencyMode>(EEPROM[FLASH_TRANSPARENCY_ADDR]);
  settings.wake = static_cast<WakeUpTime>(EEPROM[FLASH_WAKE_UP_TIME_ADDR]);
  settings.drive = static_cast<IoDrive>(EEPROM[FLASH_IO_MODE_ADDR]);
  settings.channel = static_cast<CommunicationChannel>(EEPROM[FLASH_CHANNEL_ADDR]);
  return settings;
}

void settingsToScreen(EbyteSettings settings){
  lv_dropdown_set_selected(ui_UARTValue, settings.baud);
  lv_dropdown_set_selected(ui_ParityValue, settings.parity);
  lv_dropdown_set_selected(ui_AirrateValue, settings.air_rate);
  lv_dropdown_set_selected(ui_PowerValue, settings.power);
  lv_dropdown_set_selected(ui_FECValue, settings.fec);
  lv_dropdown_set_selected(ui_TransparencyValue, settings.mode);
  lv_dropdown_set_selected(ui_WORValue, settings.wake);
  lv_dropdown_set_selected(ui_IOModeValue, settings.drive);
  lv_dropdown_set_selected(ui_ChannelValue, settings.channel);
}

void saveGeneralSettingsFunc(lv_event_t * e){
  EEPROM[FLASH_SEND_DELAY_ADDR] = delay_amount;
  EEPROM[FLASH_DMX_CHANNEL_OFFSET_ADDR] = sender_number;
  EEPROM.commit();
}

void retrieveConfigurationFunc(lv_event_t * e){

  Serial2.end(); //Stop current baudrate
  Serial2.begin(9600); //config mode at 9600

  uint8_t rx_buffer[10] = {};
  uint8_t tx_buffer[10] = {};

  //read command
  memset(tx_buffer, 0xC1, 3);

  //Write and make sure data is finished sending before switching from write to read
  Serial2.write(tx_buffer, 3);
  Serial2.flush();

  //MAX485 needs to enable read!
  digitalWrite(RE2, LOW);  // sets the digital pin RE2 off
  digitalWrite(DE2, LOW);  // sets the digital pin DE2 off

  Serial2.readBytes(rx_buffer, 6);

  EbyteSettings rx_settings = decodeEbyteSettings(rx_buffer);

  if(!isArrayZero((const uint8_t*)rx_buffer, sizeof(rx_buffer))){
    settingsToFlash(rx_settings);
    settingsToScreen(rx_settings);
  }

  #if(DEBUG_DATA)
    Serial.print("Retrieved: ");
    printEbyteSettings(rx_settings);
  #endif

  digitalWrite(RE2, HIGH);  // sets the digital pin RE2 off
  digitalWrite(DE2, HIGH);  // sets the digital pin DE2 off

  Serial2.end(); //Stop config baudrate
  Serial2.begin(BAUD_RATE); //start at set baudrate
}

void submitConfigurationFunc(lv_event_t * e)
{

  Serial2.end(); //Stop current baudrate
  Serial2.begin(9600); //config mode at 9600

  uint8_t rx_buffer[10] = {};
  uint8_t tx_buffer[10] = {};

  //read command
  memset(tx_buffer, 0xC1, 3);

  //Write and make sure data is finished sending before switching from write to read
  Serial2.write(tx_buffer, 3);
  Serial2.flush();

  //MAX485 needs to enable read!
  digitalWrite(RE2, LOW);  // sets the digital pin RE2 off
  digitalWrite(DE2, LOW);  // sets the digital pin DE2 off

  Serial2.readBytes(rx_buffer, 6);

  EbyteSettings rx_settings = decodeEbyteSettings(rx_buffer);

  Serial.print("Retrieved: ");
  printEbyteSettings(rx_settings);

  if(isArrayZero((const uint8_t*)rx_buffer, sizeof(rx_buffer))){
    Serial.print("Received empty settings!");
    return;
  }

  digitalWrite(RE2, HIGH);  // sets the digital pin RE2 off
  digitalWrite(DE2, HIGH);  // sets the digital pin DE2 off

  EbyteSettings tx_settings = {
		.head = SAVE_ON_EXIT,
		.parity = static_cast<UartParity>(lv_dropdown_get_selected(ui_ParityValue)),
		.baud = static_cast<UartBaudRate>(lv_dropdown_get_selected(ui_UARTValue)),
		.air_rate = static_cast<AirRate>(lv_dropdown_get_selected(ui_AirrateValue)),
		.channel = static_cast<CommunicationChannel>(lv_dropdown_get_selected(ui_ChannelValue)),
		.mode = static_cast<TransparencyMode>(lv_dropdown_get_selected(ui_TransparencyValue)), //Transparant misschien sneller???
		.drive = static_cast<IoDrive>(lv_dropdown_get_selected(ui_IOModeValue)),
		.wake = static_cast<WakeUpTime>(lv_dropdown_get_selected(ui_WORValue)),
		.fec = static_cast<ForwardErrorCorrection>(lv_dropdown_get_selected(ui_FECValue)),
		.power = static_cast<TransmissionPower>(lv_dropdown_get_selected(ui_PowerValue)),
	};

  if (rx_settings == tx_settings){
    Serial.print("Config is already the same!");
    Serial2.end(); //Stop config baudrate
    Serial2.begin(BAUD_RATE); //start at set baudrate
    return;
  }

  tx_buffer[0] = tx_settings.head;
  tx_buffer[1] = tx_buffer[2] = 0;
  tx_buffer[3] = (tx_settings.parity << 6) | (tx_settings.baud << 3) | tx_settings.air_rate;
  tx_buffer[4] = tx_settings.channel;
  tx_buffer[5] = (tx_settings.mode << 7) | (tx_settings.drive << 6) | (tx_settings.wake << 3) | (tx_settings.fec << 1) | tx_settings.power;

  Serial2.write(tx_buffer, 6);
  Serial2.flush();

  //Config gets returned if successful so we read it!
  //MAX485 needs to enable read!
  digitalWrite(RE2, LOW);  // sets the digital pin RE2 off
  digitalWrite(DE2, LOW);  // sets the digital pin DE2 off

  Serial2.readBytes(rx_buffer, 6);

  Serial.print("Sent: ");
  printEbyteSettings(tx_settings);

  digitalWrite(RE2, HIGH);  // sets the digital pin RE2 off
  digitalWrite(DE2, HIGH);  // sets the digital pin DE2 off
  
  //check if the config is the same
  rx_settings = decodeEbyteSettings(rx_buffer);

  Serial.print("Confirmation: ");
  printEbyteSettings(rx_settings);

  //save the config if same
  if (rx_settings == tx_settings){
    settingsToFlash(tx_settings);
  }

  Serial2.end(); //Stop config baudrate
  Serial2.begin(BAUD_RATE); //start at set baudrate
}

void initialise_flash(){
  EEPROM[FLASH_UART_BAUD_ADDR] = BAUD_115200;
  EEPROM[FLASH_UART_PARITY_ADDR] = NO_PARITY;
  EEPROM[FLASH_AIRRATE_ADDR] = AIR_RATE_19200;
  EEPROM[FLASH_POWER_ADDR] = POWER_30DBM;
  EEPROM[FLASH_FEC_ADDR] = FEC_OFF;
  EEPROM[FLASH_TRANSPARENCY_ADDR] = TRANSPARENT;
  EEPROM[FLASH_WAKE_UP_TIME_ADDR] = OPEN_COLLECTOR;
  EEPROM[FLASH_IO_MODE_ADDR] = OPEN_COLLECTOR;
  EEPROM[FLASH_CHANNEL_ADDR] = CHANNEL_0;
  EEPROM[FLASH_SEND_DELAY_ADDR] = (uint8_t)64;
  EEPROM[FLASH_DMX_CHANNEL_OFFSET_ADDR] = (uint8_t)0;
  EEPROM[FLASH_INITIALISED_ADDR] = true;
  EEPROM.commit();
}

void setup ()
{
  //DMX
  // Setup our DMX Input to read on GPIO 1, from channel 1 to 3
  dmxInput.begin(1, START_CHANNEL, NUM_CHANNELS);

  pinMode(RE1, OUTPUT);
  pinMode(DE1, OUTPUT);
  pinMode(RE2, OUTPUT);
  pinMode(DE2, OUTPUT);

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);  // Backlight on

  digitalWrite(RE1, LOW);  // sets the digital pin RE1 off
  digitalWrite(DE1, LOW);  // sets the digital pin DE1 off
  digitalWrite(RE2, HIGH);  // sets the digital pin RE2 off
  digitalWrite(DE2, HIGH);  // sets the digital pin DE2 off

  //Serial
  Serial2.begin(BAUD_RATE);

  // Setup the onboard LED so that we can blink when we receives packets
  pinMode(LED_BUILTIN, OUTPUT);


  if(async_dmx){
    //Async reading
    dmxInput.read_async(dmx_buffer, dmxDataReceived);
  }
}

void setup1(){

    lv_init();

    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */

    static lv_disp_t* disp;
    disp = lv_display_create( screenWidth, screenHeight );

    //ALS HET SCHERM NIETS DOET, KIJK OF OPTIMIZE OP -O STAAT, -Os WERKT NIET!!!
    lv_display_set_buffers( disp, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL );
    digitalWrite(LED_BUILTIN, HIGH);
    lv_display_set_flush_cb( disp, my_disp_flush );

    static lv_indev_t* indev;
    indev = lv_indev_create();
    lv_indev_set_type( indev, LV_INDEV_TYPE_POINTER );
    lv_indev_set_read_cb( indev, my_touchpad_read );

    lv_tick_set_cb( my_tick_get_cb );

    ui_init();

    //Retrieve data
    EEPROM.begin(256);

    bool flash_initialised = EEPROM.read(FLASH_INITIALISED_ADDR);

    if (!flash_initialised){
      initialise_flash();
    }

    EbyteSettings settings = flashToSettings();
    settingsToScreen(settings);

    //set data for screen
    //get chart series
    lv_chart_series_t * serie = lv_chart_get_series_next(ui_Chart1, NULL);

    //Set Chart Array
    lv_chart_set_ext_y_array(ui_Chart1, serie, data_array);

    //get settings from persistent data
    delay_amount = EEPROM[FLASH_SEND_DELAY_ADDR];
    sender_number = EEPROM[FLASH_DMX_CHANNEL_OFFSET_ADDR];

    //Set General Config Options
    lv_spinbox_set_value(ui_delaySpinbox, delay_amount);
    lv_roller_set_selected(ui_channelSelector , sender_number, LV_ANIM_ON);
}

void loop ()
{
  //loop handles DMX and sender related things
  start_index = (54 * sender_number) + 1;

  if(async_dmx){
    /*
    memcpy(uart_buffer, (const uint8_t*)&dmx_buffer[start_index], 54);

    if (millis() - start_time >= delay_amount && !is_configuring) {

      digitalWrite(LED_BUILTIN, HIGH);

      Serial2.write(uart_buffer, 54);

      start_time = millis();
    }
    */

    digitalWrite(LED_BUILTIN, LOW);
  } else {
    // Wait for next DMX packet
    dmxInput.read(dmx_buffer);

    memcpy(uart_buffer, (const uint8_t*)&dmx_buffer[start_index], 54);

    if (millis() - start_time >= delay_amount && !is_configuring) {
      digitalWrite(LED_BUILTIN, HIGH);

      Serial2.write(uart_buffer, 54);
      
      start_time = millis();
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void loop1()
{
  //loop1 handles screen
  lv_timer_handler(); /* let the GUI do its work */

  sender_number = lv_roller_get_selected(ui_channelSelector);

  delay_amount = lv_spinbox_get_value(ui_delaySpinbox);

  lv_obj_t * s = lv_scr_act(); //get active screen
  if(s == ui_ConfigEbyte) {
    is_configuring = true;
  } else {
    is_configuring = false;
  }

  updateChart((uint8_t*)&dmx_buffer[start_index]);
}