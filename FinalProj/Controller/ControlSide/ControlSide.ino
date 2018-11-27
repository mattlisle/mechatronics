/* Lab 3.2: ControlSide
 * Author: Shiv, Alejandro, Matt
 * Copyright: We don't have enough money to sue you anyway
 * License: See above
 */

/* -------------------- Includes -------------------- */
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Wire.h"
#include "SH1106.h" 
/* -------------------- Defines -------------------- */
// Pins
#define LED_BUILTIN   2
#define WEAPON_UD     34
#define WEAPON_LR     39
#define STEERING_PIN  35
#define THROTTLE_PIN  32

// WiFi
#define LOCALPORT     2390
#define REMOTEPORT    2800

// Motor Calibration
//TODO: UPDATE THESE VALUES
#define THROTTLE_OFFSET 175
#define STEERING_OFFSET 155
#define WEAP_UD_OFFSET 175
#define WEAP_LR_OFFSET 155

//Initialize Joystick
float UD_center = 2047 - WEAP_UD_OFFSET;
float LR_center = 2047 - WEAP_LR_OFFSET;
float x = UD_center;
float y = LR_center;


/* -------------------- Initialize LCD -------------------- */
//SDA is 19
//SCL is 22
SH1106 display(0x3c, 19, 22); 

/* -------------------- Global Varibales -------------------- */
// WiFi
//const char* ssid = "Mechatronics";
//const char* password = "YayFunFun";
const char* ssid = "iPhoneHotspot";
const char* password = "sexpanther";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 120);
IPAddress ipTarget(192, 168, 1, 158);
const int UDP_PACKET_SIZE = 10; 
char udpBuffer[UDP_PACKET_SIZE];
byte packetBuffer[UDP_PACKET_SIZE+1];

/* -------------------- Code -------------------- */
void setup() {
  // Pins
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(WEAPON_UD,INPUT);
  pinMode(WEAPON_LR,INPUT);
  pinMode(STEERING_PIN,INPUT);
  pinMode(THROTTLE_PIN,INPUT);

  // Serial
  Serial.begin(115200);
  delay(10);
  // LCD
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Connecting");
  display.drawString(0, 16, "to Wifi!");
  display.display();
  delay(2000);
  // WiFi
  WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  udp.begin(LOCALPORT);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  packetBuffer[UDP_PACKET_SIZE] = 0;
  display.clear();
  // Wair for the GO! message
  // Comment or uncomment based on use case
  //waitForGo();
}

void loop() {
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Connected");
  display.drawString(0, 16, "to Wifi!");
  display.display();
  display.clear();
  // Byte to hold rotation direction of each motor
  byte dirByte;
  int max_steer;
  
  // Read the joystick values
  int mag_val = map(analogRead(THROTTLE_PIN) + THROTTLE_OFFSET, 0, 4095, -255, 255);
  if (abs(mag_val) < 10) {
    max_steer = 80;
  } else {
    max_steer = map(abs(mag_val), 0, 255, 60, 110);
  }
  int dir_val = map(analogRead(STEERING_PIN) + STEERING_OFFSET, 0, 4095, -max_steer, max_steer);

  // Calculate duty cycle values
  int dc_right = _min(255, abs(mag_val - dir_val));
  int dc_left = _min(255, abs(mag_val + dir_val));

  // Calculate individual motor directions
  int dir_right = (mag_val - dir_val) < 0;
  int dir_left = (mag_val + dir_val) < 0;

  // Weapon readings
  float weaponUD = analogRead(WEAPON_UD);
  float weaponLR = analogRead(WEAPON_LR);

  // Base and arm control
  x = (weaponUD - UD_center) / 100 + x;
  y = (weaponLR - LR_center) / 100 + y;

  // Map values from 0 to 180 for servo
  int basepos = map(x, 0, 4095, 0, 180);
  int armpos = map(y, 0, 4095, 0, 180);
  // Debug printing
  Serial.println(basepos);
  Serial.print("   ");
  Serial.print(armpos);
  Serial.print("   ");
  Serial.print(dc_right);
  Serial.print("   ");
  Serial.print(dc_left);
  Serial.print("   ");
  
  // Write these bits to dirByte so it can be sent as single number
  bitWrite(dirByte, 0, 1);
  bitWrite(dirByte, 1, dir_right);
  bitWrite(dirByte, 2, dir_left);
  //TODO: add team switch input
  //TODO: add in byte for team 
  // Communicate 
  digitalWrite(LED_BUILTIN,HIGH);
  sendPacket(dc_right, dc_left, dirByte, basepos, armpos);
  digitalWrite(LED_BUILTIN,LOW);
  delay(50);
  
}

/* -------------------- Functions -------------------- */

/*******************************************************
 * Function: sendPacket
 * Collate info into packet and send to car
 *******************************************************/
void sendPacket (int throttleIn, int steeringIn, byte dirIn, int baseIn, int armIn){
  
  // set all bytes in the buffer to 0
  memset(udpBuffer, 0, UDP_PACKET_SIZE); 

  // Load the buffer, maybe add one so theyre never zero?
  udpBuffer[0] = throttleIn;
  udpBuffer[1] = steeringIn;
  udpBuffer[2] = dirIn;
  udpBuffer[3] = baseIn;
  udpBuffer[4] = armIn;
  
  udp.beginPacket(ipTarget, REMOTEPORT);

  // Let it fly
  udp.print(udpBuffer);

  udp.endPacket();
}


/*******************************************************
 * Function: waitForGo
 * Sets go boolean once GO! message is received
 *******************************************************/
void waitForGo(){
  while(1) {
    int cb = udp.parsePacket();

    // If there's a packet to read, read into myData
    if(cb) {
      udp.read(packetBuffer, UDP_PACKET_SIZE);
      String myData= "";
      for(int i= 0; i < UDP_PACKET_SIZE; i++) {
        myData += (char)packetBuffer[i];
      }

      // If the packet said "GO!" let's gooooo
      if(myData.equals("GO!")){
        break;
      }
    }
  }
}
