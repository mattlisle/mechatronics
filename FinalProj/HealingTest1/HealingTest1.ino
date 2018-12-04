/* Lab 4: Healing Light Test
*/

/* -------------------- Includes -------------------- */

/* -------------------- Defines -------------------- */
// Pins
#define HEALING_PIN 36
#define LED_BUILTIN 2

// Healing light constants
#define TIMER_COUNTS 800000
#define PRESCALER 10
#define MIN230 (80000000 / (PRESCALER * (230 + 50)))
#define MAX230 (80000000 / (PRESCALER * (230 - 50)))
#define MIN1600 (80000000 / (PRESCALER * (1600 + 50)))
#define MAX1600 (80000000 / (PRESCALER * (1600 - 50)))


/* -------------------- Global Variables -------------------- */
// Timer
hw_timer_t * timer = NULL;

// Healing
volatile byte times_up = LOW;
byte healing_status = 0;


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
  byte healing_pin_state = digitalRead(HEALING_PIN);
  if (!healing_pin_state | healing_status) {
    healing_status = get_healing_status();
    
    if (healing_status) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    } 
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
  int last_state;
  byte this_state;
  int this_adc;
  int adc_val;
  long period = 0;
  byte result = 0;
  long loops_total = 0;
  
  // Wait for the beginning of the next timer loop
  Serial.println("Begin");
  times_up = LOW;
  timerAlarmEnable(timer);

  // Get our initial state
  last_state = digitalRead(HEALING_PIN);

  // Loop until the interrupt fires
  while(!times_up) {
    this_state = digitalRead(HEALING_PIN);

    // Add the number of rising edges over time
    if (this_state) {
      if (!last_state) {
        num_periods += 1;
      }
    }
    last_state = this_state;
  }

  // Turn off the timer
  timerAlarmDisable(timer);
  times_up = LOW;

  // Calculate the number of counts per period we saw
  if (num_periods) {
    period = TIMER_COUNTS / (num_periods);
  } else {
    period = TIMER_COUNTS;
  }

  // See if it falls into acceptable range
  if ( ((period > MIN230) & (period < MAX230)) | ((period > MIN1600) & (period < MAX1600)) ) {
    result = 1;
  } else {
    result = 0; 
  }

  // Debugging
  Serial.print("Period Detected: ");
  Serial.println(period);
  Serial.print("Result: ");
  Serial.println(result);
  
  return result;
}
