/* Problem: 3.1.4
 * Author: Shiv Dalla
 * Copyright: I don't have enough money to sue you anyway
 * License: See above
 */

/* -------------------- Includes -------------------- */
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/* -------------------- Defines -------------------- */
#define LED_BUILTIN   2
#define EN1   23 //4
#define H1A   21
#define H2A   17
#define EN3   22
#define H3A   19
#define H4A   18
#define UD    35
#define LR    34

/* -------------------- Global Variables -------------------- */
int mag_val;
int dir_val;
byte dc_left;
byte dc_right;
byte dir_left;
byte dir_right;

volatile byte state = LOW;

// For setting up ledcChannel
byte resolution = 8;
byte left_channel = 1;
byte right_channel = 2;
int freq = 200;

/* -------------------- ISRs -------------------- */

/* -------------------- Code -------------------- */

void setup() {
//  Serial.begin(115200);
//  delay(10);

  pinMode(UD, INPUT);
  pinMode(LR, INPUT);
  pinMode(EN1, OUTPUT);
  pinMode(H1A, OUTPUT);
  pinMode(H2A, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(H3A, OUTPUT);
  pinMode(H4A, OUTPUT);
  
  ledcSetup(left_channel, freq, resolution);
  ledcSetup(right_channel, freq, resolution);
  ledcAttachPin(EN1, left_channel);
  ledcAttachPin(EN3, right_channel);
}

void loop() {
  mag_val = map(analogRead(UD), 0, 4095, -255, 255);
  dir_val = map(analogRead(LR), 0, 4095, -100, 100);

  dc_right = _min(255, abs(mag_val + dir_val));
  dc_left = _min(255, abs(mag_val - dir_val));

  dir_right = (mag_val + dir_val) < 0;
  dir_left = (mag_val - dir_val) > 0;
//  
//  Serial.println("----------");
//  Serial.print("UD: ");
//  Serial.println(mag_val);
//  Serial.print("LR: ");
//  Serial.println(dir_val);
//  Serial.print("dir_left: ");
//  Serial.println(dir_left);
//  Serial.print("dir_right: ");
//  Serial.println(dir_right);
//  Serial.print("dc_left: ");
//  Serial.println(dc_left);
//  Serial.print("dc_right: ");
//  Serial.println(dc_right);

//  digitalWrite(EN1, state);
//  state = !state;
  if (dir_left) {
    digitalWrite(H1A, LOW);
    digitalWrite(H2A, HIGH);
  } else {
    digitalWrite(H2A, LOW);
    digitalWrite(H1A, HIGH);
  }
  if (dir_right) {
    digitalWrite(H3A, LOW);
    digitalWrite(H4A, HIGH);
  } else {
    digitalWrite(H4A, LOW);
    digitalWrite(H3A, HIGH);
  }
  
  ledcWrite(left_channel, dc_left);
  ledcWrite(right_channel, dc_right);

  delay(100);
}


/* -------------------- Functions -------------------- */
