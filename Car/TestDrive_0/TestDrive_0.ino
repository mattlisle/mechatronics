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
#define EN1   13
#define H1A   12
#define H2A   14
#define EN3   25
#define H3A   26  
#define H4A   27

/* -------------------- Global Variables -------------------- */
int dir_val;
byte dc_left[8] = {0, 100, 100, 0, 0, 100, 100, 0};
byte dc_right[8] = {0, 0, 100, 100, 0, 0, 100, 100};
byte dir_left[8] = {1, 1, 1, 1, 0, 0, 0, 0};
byte dir_right[8] = {1, 1, 1, 1, 0, 0, 0, 0};
byte i;

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

//  
//  Serial.println("----------");
//  Serial.print("UD: ");
//  Serial.println(mag_val);
//  Serial.print("LR: ");
//  Serial.println(dir_val);
//  Serial.print("dir_left: ");
//  Serial.println(dir_left[i]);
//  Serial.print("dir_right: ");
//  Serial.println(dir_right[i]);
//  Serial.print("dc_left: ");
//  Serial.println(dc_left[i]);
//  Serial.print("dc_right: ");
//  Serial.println(dc_right[i]);

//  digitalWrite(EN1, state);
//  state = !state;
  if (dir_left[i]) {
    digitalWrite(H1A, LOW);
    digitalWrite(H2A, HIGH);
  } else {
    digitalWrite(H2A, LOW);
    digitalWrite(H1A, HIGH);
  }
  if (dir_right[i]) {
    digitalWrite(H3A, LOW);
    digitalWrite(H4A, HIGH);
  } else {
    digitalWrite(H4A, LOW);
    digitalWrite(H3A, HIGH);
  }
  
  ledcWrite(left_channel, dc_left[i]);
  ledcWrite(right_channel, dc_right[i]);

  if (i == 7) {
    i = 0;
  } else {
    i += 1;
  }

  delay(1000);
}


/* -------------------- Functions -------------------- */
