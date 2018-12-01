
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

//LED STUFF++++++++++++++++++++++++++++++++++++++++
#include "FastLED.h"
//#include <FastLED.h>

FASTLED_USING_NAMESPACE

#define ROBOTNUM 4            // robot number
#define RED 0xFF0000          // color for the red team
#define BLUE 0x0000FF         // color for the blue team
#define HEALTHCOLOR 0x00FF00  // color for the health LEDs
#define TEAMCOLOR BLUE         // robot team
#define FLASHHALFPERIOD 250   // the blue team is supposed to flash this is half of the period of that flash
#define READPERIOD 400        // Slows down the I2C

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    12  //What pin is the LED ring data on
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
//END LED STUFF++++++++++++++++++++++++++++++++++++++++


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
  
  if (TEAMCOLOR == BLUE){ // if the team is blue you need to flash the LEDs
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

  
  
  leds[robotLeds[0]]=TEAMCOLOR*ledsOn;  // The first LED is always displayed with the robot number

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

void setup() {
  Serial.begin(115200);
  SetupFastLED();  // Setup the LEDs
  
}

void loop() {
  //LED STUFF++++++++++++++++++++++++++++++++++++++++
  static int health = 0;  // what this robots health is
  
  // send the 'leds' array out to the actual LED strip
  ShowRobotNum();  // set the LEDs for the robot number
  //health = healthRobot[ROBOTNUM-1+(TEAMCOLOR == BLUE)*4];  // Get the position based on the robot number it is zero indexed so we need to lower everything by 1, if it is a blue robot we need to start after the read robots
  ShowHealth(health); //set the LEDs for the health
  if (0 == health){  // If we are dead turn off the lights.
    clearLEDs();
  }
  Serial.print("health: "); 
  Serial.println(health);
  
  delay(FLASHHALFPERIOD/2);  // wait a bit so the LEDs don't cycle to fast
  FastLEDshowESP32(); //Actually send the values to the ring
  
  FastLED.delay(1000/FRAMES_PER_SECOND); // insert a delay to keep the framerate modest
  //END LED STUFF++++++++++++++++++++++++++++++++++++++++

}
