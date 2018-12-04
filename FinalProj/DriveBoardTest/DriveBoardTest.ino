#include <Servo.h>

#define PWM1 13
#define PWM2 12
#define DIR1 14
#define DIR2 27

static const int servoPin = 26;

// For setting up ledcChannel
byte resolution = 8;
byte left_channel = 2;
byte right_channel = 3;
int freq = 200;

byte dc_left[4] = {0, 100, 100, 0};
byte dc_right[4] = {0, 0, 100, 100};
byte dir_left[4] = {0, 0, 0, 0};
byte dir_right[4] = {0, 0, 0, 0};

Servo servo1;

void setup() {
    Serial.begin(115200);
    servo1.attach(servoPin);

    pinMode(DIR1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
//
//    // PWM
    ledcSetup(left_channel, freq, resolution);
    ledcSetup(right_channel, freq, resolution);
    ledcAttachPin(PWM1, left_channel);
    ledcAttachPin(PWM2, right_channel);
}

void loop() {
    for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
        servo1.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }

    for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
        servo1.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }
    for (int i = 0; i < 4; i++) {

      if (dir_left[i]) {
        digitalWrite(DIR1, HIGH);
      } else {
        digitalWrite(DIR1, LOW);
      }
      if (dir_right[i]) {
        digitalWrite(DIR2, HIGH);
      } else {
        digitalWrite(DIR2, LOW);
      }
      
      // Generate PWM signal based on supplied duty cycle
      ledcWrite(left_channel, dc_left[i]);
      ledcWrite(right_channel, dc_right[i]);

      delay(1000);
    }
}
