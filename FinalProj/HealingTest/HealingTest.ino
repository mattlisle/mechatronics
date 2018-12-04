/* Lab 4: Healing Light Test
*/

/* -------------------- Includes -------------------- */
#include <driver/adc.h>

/* -------------------- Defines -------------------- */
// Pins
#define HEALING_PIN 36
#define LED_BUILTIN 2

// Healing light constants
#define LOW_THRESH (17 * 4095 / 33)  // Sets LOW_THRESHold for falling edge, equiv to 2V
#define HIGH_THRESH (25 * 4095 / 33)
#define TIMER_COUNTS 8000000
#define PRESCALER 1
#define MIN230 (80000000 / (PRESCALER * (230 + 100)))
#define MAX230 (80000000 / (PRESCALER * (230 - 100)))
#define MIN1600 (80000000 / (PRESCALER * (1600 + 100)))
#define MAX1600 (80000000 / (PRESCALER * (1600 - 100)))


/* -------------------- Global Variables -------------------- */
// Timer
hw_timer_t * timer = NULL;

// Healing
int healing_adc = 4095;
volatile byte times_up = LOW;


/* -------------------- ISRs -------------------- */
/***********************************
 * Function: IRAM_ATTR
 * ISR for LED state control
 ***********************************/
void IRAM_ATTR onTimer(){
  times_up = HIGH;
}


/* -------------------- Code -------------------- */

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(HEALING_PIN, INPUT);

  // Timer setup
  timer = timerBegin(0, PRESCALER, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_COUNTS, true);
//  timerAlarmWrite(timer, counts, true);
  timerAlarmEnable(timer);

  // ADC setup
//  adc1_config_width(ADC_WIDTH_BIT_12);
//  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
//  analogSetSamples(1);
//  adc_set_clk_div(1);

  // Debugging
  Serial.print("Min 230: ");
  Serial.println(MIN230);
  Serial.print("Max 230: ");
  Serial.println(MAX230);
  Serial.print("Min 1600: ");
  Serial.println(MIN1600);
  Serial.print("Max 1600: ");
  Serial.println(MAX1600);

}

void loop() {
  // If we've got a falling edge, need to check the frequency of the LED light
//  healing_adc = analogRead(HEALING_PIN);
//  healing_adc = 0;
  
//  if (healing_adc < LOW_THRESH) {
//    byte healing_status = get_healing_status();
//    
//    if (healing_status) {
//      digitalWrite(LED_BUILTIN, HIGH);
//    } else {
//      digitalWrite(LED_BUILTIN, LOW);
//    } 
//  }

  byte healing_status = get_healing_status();
  
  if (healing_status) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  } 
  
  delay(1);
}


/* -------------------- Functions -------------------- */

/*******************************************************
 * Function: readPacket
 * Reads packet from Controller into dc and dir variables
 *******************************************************/
byte get_healing_status() {
  byte num_periods = 0;
  int last_adc;
  byte pin_state;
  int this_adc;
  int adc_val;
  long period = 0;
  byte result = 0;
  long loops_total = 0;
  
  // Start the timer
//  timerAlarmEnable(timer);

  // Wait for the beginning of the next timer loop
  while(!times_up) {}
  Serial.println("Begin");
  times_up = LOW;

  last_adc = analogRead(HEALING_PIN);
//  last_adc = adc1_get_raw(ADC1_CHANNEL_0);
  pin_state = (last_adc > LOW_THRESH);
  while(!times_up) {
    loops_total += 1;

//    this_adc = analogRead(HEALING_PIN);
//    adc_val = (this_adc + last_adc) / 2;
//    last_adc = this_adc;

    adc_val = digitalRead(HEALING_PIN);
//    adc_val = adc1_get_raw(ADC1_CHANNEL_0);
    
//    // Pin was previously low
//    if (!pin_state) {
//      // Read the rising edge
//      if (adc_val > HIGH_THRESH) {
//        pin_state = 1;
//        num_periods += 1;
//      }
//
//    // Pin was previously high
//    } else {
//      // If the signal is now below the low threshold, change the state
//      if (adc_val < LOW_THRESH) {
//        pin_state = 0;
//      }
//    }
  }
  times_up = LOW;

  if (num_periods) {
    period = TIMER_COUNTS / num_periods;
  } else {
    period = TIMER_COUNTS;
  }

  if ( ((period > MIN230) & (period < MAX230)) | ((period > MIN1600) & (period < MAX1600)) ) {
    result = 1;
  } else {
    result = 0; 
  }
  
  Serial.print("Period Detected: ");
  Serial.println(num_periods);
  Serial.print("Result: ");
  Serial.println(result);
  Serial.print("Total Loops: ");
  Serial.println(loops_total);
  
  return result;
}
