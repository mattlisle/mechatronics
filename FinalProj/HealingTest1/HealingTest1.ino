/* Lab 4: Healing Light Test
*/

/* -------------------- Includes -------------------- */

/* -------------------- Defines -------------------- */
// Pins
#define HEALING_PIN 36
#define LED_BUILTIN 2

// Healing light constants
#define TIMER_COUNTS 400000
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
  byte result = 0;

  // Forces second while loop to break unless we detect more than 1 period
  period_counts[1] = 0;
    
  // Wait for the beginning of the next timer loop
  Serial.println("Begin");
  timerAlarmEnable(timer);

  // Get our initial state
  last_state = digitalRead(HEALING_PIN);

  // Loop until the interrupt fires
  byte i = 0;
  times_up = LOW;
  while(!times_up) {
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
  }

  // Turn off the timer
  timerAlarmDisable(timer);
  times_up = LOW;

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
    j++;
  }

  // If we have a lot of one frequency and no more than one occurrence of the other, we're healing
  if ((valid_periods_230 > 8) & (valid_periods_1600 < 2)) {
    result = 1;
  } else if ((valid_periods_230 < 2) & (valid_periods_1600 > 20)) {
    result = 1;
  } else {
    result = 0;
  }

  return result;
}
