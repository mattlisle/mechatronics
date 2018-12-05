

/* Lab 3.2: DriveSide
 * Author: Shiv, Alejo, Matt
 * Copyright: I don't have enough money to sue you anyway
 * License: See above
 */

/* -------------------- Includes -------------------- */
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPServo.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

//LED DEFINE STUFF++++++++++++++++++++++++++++++++++++++++
long int TEAMCOLOR;
#include "FastLED.h"
FASTLED_USING_NAMESPACE

#define ROBOTNUM 2            // robot number
#define RED 0xFF0000          // color for the red team
#define BLUE 0x0000FF         // color for the blue team
#define HEALTHCOLOR 0x00FF00  // color for the health LEDs
#define WHITECOLOR 0xFFFFFF  // color of white for healing
#define FLASHHALFPERIOD 250   // the blue team is supposed to flash this is half of the period of that flash
#define READPERIOD 400        // Slows down the I2C

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif
//TODO: CONFIRM DATA PIN
#define DATA_PIN    18  //What pin is the LED ring data on
#define LED_TYPE    WS2812  //APA102
#define COLOR_ORDER GRB  // changes the order so we can use standard RGB for the values we set.
#define NUM_LEDS    24  //Number of LEDs in the ring
CRGB leds[NUM_LEDS];  // this is the place you set the value of the LEDs each LED is 24 bits

#define BRIGHTNESS          60   // lower the brighness a bit so it doesn't look blown out on the camera.
#define FRAMES_PER_SECOND  120   // some number this is likely faster than needed

// -- The core to run FastLED.show()
#define FASTLED_SHOW_CORE 0

// -- Task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;

//END LED DEFINE STUFF++++++++++++++++++++++++++++++++++++++++

// I2C DEFINES STUFF===========================================
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128           /*!< Data buffer length of test buffer */
#define W_LENGTH 1                /*!< Data length for w, [0,DATA_LENGTH] */
#define R_LENGTH 16               /*!< Data length for r, [0,DATA_LENGTH] */

// TODO: double check these are correct
#define I2C_MASTER_SCL_IO (gpio_num_t)22             /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO (gpio_num_t)21               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(1) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL I2C_MASTER_ACK                             /*!< I2C ack value */
#define NACK_VAL I2C_MASTER_NACK                           /*!< I2C nack value */

// END I2C DEFINES STUFF===========================================

/* -------------------- OUR Defines -------------------- */
// Pins
#define LED_BUILTIN  2
#define EN1   13
#define EN3   12
// direction of zero is backwards in both cases
#define DIR1  14
#define DIR2  27
//Servo pins
#define BASE_SERVO 25 //TODO ADD PINS HERE
#define ARM_SERVO 26 //TODO ADD PINS HERE
//Weapon Pins
#define WEAPON_IN 39
#define WEAPON_OUT 19

// Pins
#define HEALING_PIN 36
byte healingFreq;

// Healing light constants
#define TIMER_COUNTS 400000
#define PRESCALER 10
#define MIN230 (80000000 / (PRESCALER * (230 + 50)))
#define MAX230 (80000000 / (PRESCALER * (230 - 50)))
#define MIN1600 (80000000 / (PRESCALER * (1600 + 50)))
#define MAX1600 (80000000 / (PRESCALER * (1600 - 50)))

// Wifi
#define LOCALPORT       2800
#define REMOTEPORT      2801
#define UDP_PACKET_SIZE 10

/* -------------------- Global Variables -------------------- */
// Motor command variables
byte dc_left;
byte dc_right;
byte dir_left;
byte dir_right;
byte base_pos;
byte arm_pos;

//Intialize Team Boolean, true is blue, false is red
int teamIsBlue; 

// For setting up ledcChannel
byte resolution = 8;
byte left_channel = 9;
byte right_channel = 10;
int freq = 200;

// Timer
//hw_timer_t * timer = NULL;

// Healing
volatile byte times_up = LOW;
byte healing_status = 0;

// WiFi
const char* ssid = "Mechatronics";
const char* password = "YayFunFun";
//const char* ssid = "iPhoneHotspot";
//const char* password = "sexpanther";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 158);
IPAddress ipTarget(192, 168, 1, 120);
char udpBuffer[UDP_PACKET_SIZE];
byte packetBuffer[UDP_PACKET_SIZE + 1];

// Servos
Servo baseServo;
Servo armServo;

/* -------------------- ISRs -------------------- */
/***********************************
 * Function: IRAM_ATTR
 * ISR for LED state control
 ***********************************/
void IRAM_ATTR onTimer(){
  times_up = HIGH;
}

/* -------------------- Functions -------------------- */
// I2C FXN STUFF===========================================
/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t nsize)
{
    if (nsize == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN); 
    if (nsize > 1) {
        i2c_master_read(cmd, data_rd, nsize - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + nsize - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS); // send all queued commands
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t nsize)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, nsize, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        Serial.printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            Serial.printf("\n");
        }
    }
    Serial.printf("\n");
}

uint8_t data_wr[DATA_LENGTH];
uint8_t data_rd[DATA_LENGTH];

static void i2c_read_test()
{
  int ret;

  ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);

  if (ret == ESP_ERR_TIMEOUT) {
    ESP_LOGE(TAG, "I2C Timeout");
    Serial.println("I2C Timeout");
  } else if (ret == ESP_OK) {
    Serial.printf(" MASTER READ FROM SLAVE ******\n");
    disp_buf(data_rd, DATA_LENGTH);
    digitalWrite(2,LOW);
  } else {
    ESP_LOGW(TAG, " %s: Master read slave error, IO not connected...\n",
             esp_err_to_name(ret));
  }
}

static void i2c_write_test()
{ 
  int ret;
                                                                             
  ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, W_LENGTH);
  if (ret == ESP_ERR_TIMEOUT) {
    ESP_LOGE(TAG, "I2C Timeout");
  } else if (ret == ESP_OK) {
    Serial.printf(" MASTER WRITE TO SLAVE\n");
    disp_buf(data_wr, W_LENGTH);
  } else {
    ESP_LOGW(TAG, "%s: Master write slave error, IO not connected....\n",
            esp_err_to_name(ret));
  }
}

// END I2C FXN STUFF===========================================

//LED STUFF++++++++++++++++++++++++++++++++++++++++
/** show() for ESP32
 *  Call this function instead of FastLED.show(). It signals core 0 to issue a show, 
 *  then waits for a notification that it is done.
 */
void FastLEDshowESP32()
{
    if (userTaskHandle == 0) {
        // -- Store the handle of the current task, so that the show task can
        //    notify it when it's done
        userTaskHandle = xTaskGetCurrentTaskHandle();

        // -- Trigger the show task
        xTaskNotifyGive(FastLEDshowTaskHandle);

        // -- Wait to be notified that it's done
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
        ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        userTaskHandle = 0;
    }
}

/** show Task
 *  This function runs on core 0 and just waits for requests to call FastLED.show()
 */
void FastLEDshowTask(void *pvParameters)
{
    // -- Run forever...
    for(;;) {
        // -- Wait for the trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // -- Do the show (synchronously)
        FastLED.show();

        // -- Notify the calling task
        xTaskNotifyGive(userTaskHandle);
    }
}

void SetupFastLED(void){
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  int core = xPortGetCoreID();
    Serial.print("Main code running on core ");
    Serial.println(core);

    // -- Create the FastLED show task
    xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);

}

void ShowRobotNum(void){
  int flashTime = millis(); // what time is it
  static int flashTimeOld = 0; // when was the last toggle
  static bool ledsOn = 1;  // are the robot number LEDs on
  int robotLeds[] = {0,6,12,18};  // location of the LEDs used to display the robot number
//  static int RONUM = 0;
//  RONUM;
  
  if (teamIsBlue){ // if the team is blue you need to flash the LEDs
    if ((flashTime-FLASHHALFPERIOD) > flashTimeOld){ // if the correct amount of time has passed toggle the robot number LEDs
//      Serial.print("changing LED state from: ");
//      Serial.println(ledsOn);
//      Serial.println();
      if (ledsOn){  // if they are on turn them off
        ledsOn = 0;
      }
      else {  // if they are off turn them on
        ledsOn = 1; 
      }
      flashTimeOld = flashTime;   //store when we changed the state
    }
    
  }

  
  if(teamIsBlue){
    TEAMCOLOR = BLUE;
  }
  else{
    TEAMCOLOR = RED;
  }
  
  leds[robotLeds[0]]=TEAMCOLOR*ledsOn;  // The first LED is always displayed with the robot color
  

  switch (ROBOTNUM){  //Change the LEDs based on the robot number
  case 1:
    leds[robotLeds[1]]=0;
    leds[robotLeds[2]]=0;
    leds[robotLeds[3]]=0;
    break;
  case 2:
    leds[robotLeds[1]]=0;
    leds[robotLeds[2]]=TEAMCOLOR*ledsOn;
    leds[robotLeds[3]]=0;
    break;
  case 3:
    leds[robotLeds[1]]=TEAMCOLOR*ledsOn;
    leds[robotLeds[2]]=0;
    leds[robotLeds[3]]=TEAMCOLOR*ledsOn;
    break;
  case 4:
    leds[robotLeds[1]]=TEAMCOLOR*ledsOn;
    leds[robotLeds[2]]=TEAMCOLOR*ledsOn;
    leds[robotLeds[3]]=TEAMCOLOR*ledsOn;
    //RONUM = 1;
    break;
  }
}

void ShowHealth(int health){
  int healthLeds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23}; // the location of the 24 LEDs used for health
  
  leds[healthLeds[0]] = HEALTHCOLOR*(health > 0);  // last LED doesn't go off till the health is 0
  leds[healthLeds[19]] = HEALTHCOLOR*(health == 100);  // first LED goes off as soon as the health is not 100

  for(int i=1; i<19; i++){
    leds[healthLeds[i]] = HEALTHCOLOR*(health > (i*5));  // the other leds go off in increments of 5
  }

  
  
}

void clearLEDs(void){
  for(int i=0; i<NUM_LEDS; i++){
    leds[i] = 0; // Turn off everything 
  }
}
// END LED STUFF++++++++++++++++++++++++++++++++++++++++


/* --------------------Main Code -------------------- */

void setup() {
  Serial.begin(115200);
  delay(10);
  // I2C STUFF===========================================
  pinMode(LED_BUILTIN, OUTPUT);  // make the onboard LED an output

  ESP_ERROR_CHECK(i2c_master_init());  // Initialize the I2C
  // END I2C STUFF===========================================
  
  //LED STUFF++++++++++++++++++++++++++++++++++++++++
  SetupFastLED();  // Setup the LEDs
  //END LED STUFF++++++++++++++++++++++++++++++++++++++++
  
  // Pins
  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(EN3, OUTPUT);
  // Weapon pins
  pinMode(WEAPON_IN, INPUT);
  pinMode(WEAPON_OUT, OUTPUT);
  // Healing
  pinMode(HEALING_PIN, INPUT);
  // Healing Timer setup
//  timer = timerBegin(2, PRESCALER, true); // changed this to timer 2
//  timerAttachInterrupt(timer, &onTimer, true);
//  timerAlarmWrite(timer, TIMER_COUNTS, true);


  // Attach servos
  baseServo.attach(BASE_SERVO);
  armServo.attach(ARM_SERVO);

  // PWM for motors
  ledcSetup(left_channel, freq, resolution);
  ledcSetup(right_channel, freq, resolution);
  ledcAttachPin(EN1, left_channel);
  ledcAttachPin(EN3, right_channel);


  // WiFi
  WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  udp.begin(LOCALPORT);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  packetBuffer[UDP_PACKET_SIZE] = 0;
  
}

void loop() {
    int currentTime = millis();  // Get the current time
    static int readTime = currentTime; // Timewhen we last read
    
    static bool gameStatus = 0;              // game on 1, game off 0
    static bool reset = 0;                   // 1 for reseting, not sure what the intention is here, check with Diego
    static bool autoMode = 0;                // 0 not autonomous, 1 is autonomous
     
    static bool syncStatus = 0;              // 0 sync still hasn't happend, 1 sync has happend
    static byte coolDownStatus = 0;          // 0 ready to hit, 1 cooling down  In the same order as the health (Red 1, 2, 3, 4, Blue 1, 2, 3, 4)
  
    static byte healthRobot[8];      // health of each robot as a byte (Red 1, 2, 3, 4, Blue 1, 2, 3, 4)
    static byte healthNexus[4];      // health of the two nexi each is 10 bit (Red L, H, Blue L, H)
    static byte towerStatus[2];      // status of the two towers.  Not sure why this needs 4 bits each.
  
    //static byte healingFreq;  // First bit is for the low frequency (1), the second bit is for the high frequency (2), zero can be sent to the hat to request the information.
    
    // I2C STUFF===========================================
    if ((currentTime - READPERIOD) >= readTime){  //if we haven't read for the correct amount of time we can do it now.
        readTime=currentTime;  // update when we last read
        // TODO: change the healingFreq value
//        switch (healingFreq) { 
//          //  This just cycles through the different information we can send, students should make it approriate to what they are sensing     
//          case 0:
//            healingFreq = 1;  //the low frequency is present
//            break;
//          case 1:
//            healingFreq = 2;  // the high freq. is present
//            break;
//          case 2:
//            healingFreq = 0;  // no healing but data requested
//            break;
//        }
        data_wr[0]=healingFreq;  // put the healing information into the buffer
        i2c_write_test();       // write the buffer
        //delay(1);
        i2c_read_test();        // read the data only do this after a write or the slave buffer can fill up
  
        gameStatus = 1 & (data_rd[0]>>0);      // game on 1, game off 0
        reset = 1 & (data_rd[0]>>1);           // 1 for reseting, not sure what the intention is here, check with Diego
        autoMode = 1 & (data_rd[0]>>2);        // 0 not autonomous, 1 is autonomous
        syncStatus = 1 & (data_rd[0]>>3);      // 0 sync still hasn't happend, 1 sync has happened,  this makes sure the times each robots sends corresponds if this 
        
  
        if (0xFF != data_rd[6]){// if the robot health is FF something is wrong and disregard the incoming data
          coolDownStatus = data_rd[1];  // 0 ready to hit, 1 cooling down  in robot order red then blue
          
          healthNexus[0] = data_rd[2];
          healthNexus[1] = data_rd[3];
          healthNexus[2] = data_rd[4];
          healthNexus[3] = data_rd[5];
    
          healthRobot[0] = data_rd[6];
          healthRobot[1] = data_rd[7];
          healthRobot[2] = data_rd[8];
          healthRobot[3] = data_rd[9];
    
          healthRobot[4] = data_rd[10];
          healthRobot[5] = data_rd[11];
          healthRobot[6] = data_rd[12];
          healthRobot[7] = data_rd[13];
    
          towerStatus[1] = 0x0F & (data_rd[14]>>0);      // This can be cleaned up because you just need the and for the first one and the shift for the second but I like the consistency.
          towerStatus[2] = 0x0F & (data_rd[14]>>4);
        }
        else{  // blink to show something went wrong
          digitalWrite(2,LOW);
          delay(250);
          digitalWrite(2,HIGH);
        }
        
    }
  
    // END I2C STUFF===========================================
  
    //LED STUFF++++++++++++++++++++++++++++++++++++++++
    static int health;  // what this robots health is
    
    // send the 'leds' array out to the actual LED strip
    ShowRobotNum();  // set the LEDs for the robot number
    health = healthRobot[ROBOTNUM-1+(teamIsBlue)*4];  // Get the position based on the robot number it is zero indexed so we need to lower everything by 1, if it is a blue robot we need to start after the read robots
    
    if (0 == health){  // If we are dead turn off the lights.
      clearLEDs();
    }
    Serial.print("health: "); 
    Serial.println(health);

    // TODO: uncomment/comment this delay?
    // SMALL BUG: due to uncommenting this line 
    // the ESP must be rebooted when team is switches
    //delay(FLASHHALFPERIOD/2);  // wait a bit so the LEDs don't cycle to fast
    
    //END LED STUFF++++++++++++++++++++++++++++++++++++++++
      
    // Read the packet sent by the controller
    readPacket();
    
    // Use values in packet to control motors, but only if we stayin alive
    if(health){
      controlMotors();
    }
      

    // If we've got a falling edge, need to check the frequency of the LED light
    byte healing_pin_state = digitalRead(HEALING_PIN);
    Serial.print("Healing Status: ");
    Serial.println(healing_status);
    if ((!healing_pin_state) | (healing_status)) {
      healing_status = get_healing_status();
      Serial.print("Healing Status after Checking: ");
      Serial.println(healing_status);
      // Set healingFreq
      healingFreq = healing_status;
    }
    if (healing_status) {
      // Set neopixel
      digitalWrite(LED_BUILTIN,HIGH);
      healLEDs(health);
    } 
    else{
      digitalWrite(LED_BUILTIN,LOW);
      // If not healing show health normally
      ShowHealth(health); //set the LEDs for the health
    }

    // Check cooldown and see if we got a hit
    // coolDownStatus, 1 if on cooldown, 0 if ready to hit
    // TODO: impliment a cooldown time, for now just ignore it
    // Hit will pull pin low -- did we hit em? Pull output low
    if(!digitalRead(WEAPON_IN)){
      hitLEDs();
      digitalWrite(WEAPON_OUT, LOW);
      //Serial.println("HIT");
    }
    else{
      digitalWrite(WEAPON_OUT, HIGH);
    }
       
    FastLEDshowESP32(); //Actually send the values to the ring
    
    FastLED.delay(1000/FRAMES_PER_SECOND); // insert a delay to keep the framerate modest
    
}


/* -------------------- Our Functions -------------------- */

/*******************************************************
 * Function: readPacket
 * Reads packet from Controller into dc and dir variables
 *******************************************************/
void readPacket () {
  
  // Check if there's a packet to read
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // We've received a packet, so let's celebrate
    digitalWrite(LED_BUILTIN,HIGH);

    // Read the packet into the buffer
    udp.read(packetBuffer, UDP_PACKET_SIZE);

    // Parse the first to bytes into duty cycles
    // motors
    dc_right = packetBuffer[0];
    dc_left = packetBuffer[1];
    // servos
    base_pos = packetBuffer[3];
    arm_pos = packetBuffer[4];
    
    // Parse the directions from the third byte
    byte dir = packetBuffer[2];
    dir_right = bitRead(dir, 1);
    dir_left = bitRead(dir,2);
    teamIsBlue = bitRead(dir,3);
    

  }
  // No packet received, sad.
  else{
    digitalWrite(LED_BUILTIN,LOW);
  }
}


/*******************************************************
 * Function: controlMotors
 * Controls duty cycle, direction of PWM to each motor and servo
 *******************************************************/
void controlMotors () {

  // Set and clear H-bridge pins to control direction
  // Set and clear H-bridge pins to control direction
  if (dir_left) {
    digitalWrite(DIR1, LOW);
  } else {
    digitalWrite(DIR1, HIGH);
    
  }
  if (dir_right) {
    digitalWrite(DIR2, LOW);
  } else {
    digitalWrite(DIR2, HIGH);
  }

  // Generate PWM signal based on supplied duty cycle
  ledcWrite(left_channel, dc_left);
  ledcWrite(right_channel, dc_right);
  // Servos
  // Base is reversed 
  baseServo.write(180-base_pos);
  armServo.write(arm_pos);
}

/*******************************************************
 * Function: hitLEDs
 * Flashes LEDs with our team color for a moment
 *******************************************************/
void hitLEDs(void){
  for(int i=0; i<NUM_LEDS; i++){
    leds[i] = TEAMCOLOR; // Turn on everything
    //delay(50); 
  }
}

/*******************************************************
 * Function: healLEDs
 * Flashes LEDs with our white while healing
 *******************************************************/
void healLEDs(int health){
  for(int i=0; i<NUM_LEDS; i++){
    //TODO TEST PULSING WHITE LED WHILE HEALING
    int healthLeds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23}; // the location of the 24 LEDs used for health
    int flashTime = millis(); // what time is it
    static int flashTimeOld = 0; // when was the last toggle
    static bool ledsOn = 1;  // are the robot number LEDs on
    
    leds[healthLeds[0]] = WHITECOLOR*(health > 0);  // last LED doesn't go off till the health is 0
    leds[healthLeds[19]] = WHITECOLOR*(health == 100);  // first LED goes off as soon as the health is not 100
    if ((flashTime-FLASHHALFPERIOD*2) > flashTimeOld){ // if the correct amount of time has passed toggle the robot number LEDs
      if (ledsOn){  // if they are on turn them off
        ledsOn = 0;
      }
      else {  // if they are off turn them on
        ledsOn = 1; 
      }
      flashTimeOld = flashTime;   //store when we changed the state
    }
    for(int i=1; i<19; i++){
      //leds[healthLeds[i]] = WHITECOLOR*!(health > (i*5))*ledsOn;  // the other leds go off in increments of 5
      if(!(health > (i*5))){
        leds[healthLeds[i]] = WHITECOLOR*ledsOn;  // the other leds go off in increments of 5
        }
      else{
        leds[healthLeds[i]] = HEALTHCOLOR*(health > (i*5))*ledsOn;  
      }
      
    }
    //delay(50); 
  }
}

/*******************************************************
 * Function: get_healing_status
 * Output: boolean value indicating if we're healing
 *******************************************************/
byte get_healing_status() {
//  byte num_periods = 0;
  int last_state;
  byte this_state;
  int this_adc;
  int adc_val;
  long period = 0;
  int period_counts[100];
  int this_period_counts = 0;
  byte valid_periods_230 = 0;
  byte valid_periods_1600 = 0;
  long total_counts = 1;
  int start_time;
  int this_time;
  static byte result;  // Changed this to static byte

  // Forces second while loop to break unless we detect more than 1 period
  for (int p = 0; p < 100; p++) {
    period_counts[p] = 0;
  }
  
  period_counts[1] = 0;
    
  // Wait for the beginning of the next timer loop
//  timerAlarmEnable(timer);

  // Get our initial state
  last_state = digitalRead(HEALING_PIN);

  // Loop until the interrupt fires
  byte i = 0;
  start_time = millis();
  this_time = millis();
//  times_up = LOW;
//  while(!times_up) {
  while (this_time - start_time <= 50) {
    this_state = digitalRead(HEALING_PIN);
    this_period_counts += 1;
    total_counts += 1;

    // Add the number of rising edges over time
    if (this_state) {
      if (!last_state) {
        
        // If we've seen more than 100, we have noise, ignore subsequent edges
        if (i < 100) {
          // Save the counts of the period and start over
          period_counts[i] = this_period_counts;
          this_period_counts = 0;
          i++;
        }
      }
    }
    last_state = this_state;
    this_time = millis();
  }
  Serial.print("Time elapsed for measuring health: ");
  Serial.println(this_time - start_time);
  // Turn off the timer
//  timerAlarmDisable(timer);
//  times_up = LOW;

  // Iterate through the periods detected and convert them to timer values
  // Then compare those values to the min and max constants for each frequency
  // If the frequency is either 1600 or 230, save it
  byte j = 0;
  period_counts[99] = 0;
  while (1) {
    period = period_counts[j] * TIMER_COUNTS / total_counts;
    if (!period) {
      break;
    }
    if ((period > MIN230) & (period < MAX230)) {
      valid_periods_230 += 1;
    } 
    if ((period > MIN1600) & (period < MAX1600)) {
      valid_periods_1600 += 1;
    } 
    Serial.print("Period: ");
    Serial.println(period);
    j++;
  }

  // If we have a lot of one frequency and no more than one occurrence of the other, we're healing
  if ((valid_periods_230 > 8) & (valid_periods_1600 < 2)) {
    result = 1;
  } else if ((valid_periods_230 < 2) & (valid_periods_1600 > 20)) {
    result = 2;
  } else {
    Serial.println("Set status back to zero");
    result = 0;
  }

  return result;
}
