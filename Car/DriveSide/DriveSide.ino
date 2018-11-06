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
#define V1  23 //4
#define V2  21
#define V3  22
#define V4  19
#define UD  35
#define LR  34

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
  Serial.begin(115200);
  delay(10);

  pinMode(UD, INPUT);
  pinMode(LR, INPUT);
  pinMode(V1, OUTPUT);
  pinMode(V2, OUTPUT);
  pinMode(V3, OUTPUT);
  pinMode(V4, OUTPUT);
  
  ledcSetup(left_channel, freq, resolution);
  ledcSetup(right_channel, freq, resolution);
  ledcAttachPin(V1, left_channel);
  ledcAttachPin(V3, right_channel);
}

void loop() {
  mag_val = map(analogRead(UD), 0, 4095, -255, 255);
  dir_val = map(analogRead(LR), 0, 4095, -100, 100);

  dc_right = _min(255, abs(mag_val + dir_val));
  dc_left = _min(255, abs(mag_val - dir_val));

  dir_right = (mag_val - dir_val) < 0;
  dir_left = (mag_val + dir_val) > 0;
  
  Serial.println("----------");
  Serial.print("UD: ");
  Serial.println(mag_val);
  Serial.print("LR: ");
  Serial.println(dir_val);
  Serial.print("dir_left: ");
  Serial.println(dir_left);
  Serial.print("dir_right: ");
  Serial.println(dir_right);
//  Serial.print("dc_left: ");
//  Serial.println(dc_left);
//  Serial.print("dc_right: ");
//  Serial.println(dc_right);

//  digitalWrite(V1, state);
//  state = !state;
  digitalWrite(V2, dir_left);
  digitalWrite(V4, dir_right);
  ledcWrite(left_channel, dc_left);
  ledcWrite(right_channel, dc_right);

  delay(100);
}


/* -------------------- Functions -------------------- */
