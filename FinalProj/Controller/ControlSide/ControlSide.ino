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
#define STEERING_PIN  36
#define THROTTLE_PIN  33
#define TEAM_PIN      18

// WiFi
#define LOCALPORT     2390
#define REMOTEPORT    2800

// Motor Calibration
//TODO: UPDATE THESE VALUES
#define THROTTLE_OFFSET 189
#define STEERING_OFFSET 165
#define WEAP_UD_OFFSET 219
#define WEAP_LR_OFFSET 271
//DECREASE this to make it more sensitive to joystick
#define SENSITIVITY 10 
int maxMapVal = 1000;

//Initialize Joystick
float throt_center = 2047 - STEERING_OFFSET;
float UD_center = 2047 - WEAP_UD_OFFSET;
float LR_center = 2047 - WEAP_LR_OFFSET;
float x = maxMapVal/2;
float y = maxMapVal/2;

//Intialize Team Boolean, true is blue, false is red
int teamIsBlue; 


/* -------------------- Initialize LCD -------------------- */
//SDA is 19
//SCL is 21
SH1106 display(0x3c, 19, 21); 

/* -------------------- Global Varibales -------------------- */
// WiFi
const char* ssid = "Mechatronics";
const char* password = "YayFunFun";
//const char* ssid = "iPhoneHotspot";
//const char* password = "sexpanther";
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
  pinMode(TEAM_PIN, INPUT);

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
  //WiFi
  WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  udp.begin(LOCALPORT);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  packetBuffer[UDP_PACKET_SIZE] = 0;
  
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Connected");
  display.drawString(0, 16, "to Wifi!");
  display.display();
  delay(1000);
  display.clear();
  // Wair for the GO! message
  // Comment or uncomment based on use case
  //waitForGo();
}

void loop() {
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Driving");
  display.display();
  
  // Byte to hold rotation direction of each motor
  byte dirByte;
  int max_steer;
  
  // Read the joystick values
  int throttleVal = analogRead(THROTTLE_PIN);
  int mag_val = map(throttleVal + THROTTLE_OFFSET, 0, 4095, -255, 255);
  if (abs(mag_val) < 10) {
    max_steer = 80;
  } else {
    max_steer = map(abs(mag_val), 0, 255, 60, 110);
  }
  int steerVal = analogRead(STEERING_PIN);
  int dir_val = map(steerVal + STEERING_OFFSET, 0, 4095, -max_steer, max_steer);

  // Calculate duty cycle values
  int dc_right = _min(255, abs(mag_val - dir_val));
  int dc_left = _min(255, abs(mag_val + dir_val));

  // Calculate individual motor directions
  int dir_right = (mag_val - dir_val) < 0;
  int dir_left = (mag_val + dir_val) < 0;

  // Weapon readings
  float weaponUD = analogRead(WEAPON_UD);
  float weaponLR = analogRead(WEAPON_LR);

//  Serial.print(weaponLR);
//  Serial.print("   ");
//  Serial.print(weaponUD);
//  Serial.print("   ");
//  Serial.print(analogRead(THROTTLE_PIN));
//  Serial.print("   ");
//  Serial.print(analogRead(STEERING_PIN));
//  Serial.println("   ");
//  Serial.print("   ");
  
  // Base and arm control
  // Dead bands
  if(abs(weaponUD - UD_center)>70){
    x = (weaponUD - UD_center) / SENSITIVITY + x;
  }
  if(abs(weaponLR - LR_center)>70){
    y = (weaponLR - LR_center) / SENSITIVITY + y;
  }
  
  if(x>maxMapVal) x = maxMapVal;
  if(x<0) x = 0;
  if(y>maxMapVal) y = maxMapVal;
  if(y<0) y = 0;
  // Map values from 0 to 180 for servo
  int basepos = map(x, 0, maxMapVal, 0, 180);
  int armpos = map(y, 0, maxMapVal, 0, 180);
  if(basepos>180) basepos = 180;
  if(basepos<0) basepos = 0;
  if(armpos>180)armpos = 180;
  if(armpos<0) armpos = 0;
  // Debug printing
  Serial.print(weaponUD-UD_center);
  Serial.print("   ");
  Serial.print(weaponLR-LR_center);
  Serial.print("   ");
  Serial.print(basepos);
  Serial.print("   ");
  Serial.print(armpos);
  Serial.print("   ");
  Serial.print(dc_right);
  Serial.print("   ");
  Serial.print(dc_left);
  Serial.println("   ");
  Serial.print("   ");
  //check team pin
  teamIsBlue = digitalRead(TEAM_PIN);
  if(teamIsBlue){
    display.drawString(0, 12, "Blue Team!");
    
  }
  else{
    display.drawString(0, 12, "Red Team!");
    
  }
  display.display();
  // Show a little throttle meter
  display.drawRect(90,0,10,30);
  display.fillRect(90,0,10,map(armpos, 0, 180, 30,0));
  display.drawProgressBar(60,35,60,10,map(basepos,0,180,0,100));
  display.drawProgressBar(0, 50, 120, 10, abs(map(throttleVal-1600, 0, 3500, 0, 100)));
  display.display();
    
  
  // Write these bits to dirByte so it can be sent as single number
  bitWrite(dirByte, 0, 1);
  bitWrite(dirByte, 1, dir_right);
  bitWrite(dirByte, 2, dir_left);
  // did we blue ourselves?
  bitWrite(dirByte, 3, teamIsBlue);
  
  // Communicate 
  digitalWrite(LED_BUILTIN,HIGH);
  sendPacket(dc_right, dc_left, dirByte, basepos, armpos);
  digitalWrite(LED_BUILTIN,LOW);
  delay(50);
  display.clear();
  
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
  //SEND 1 FIRST?
  udpBuffer[0] = throttleIn+1;
  udpBuffer[1] = steeringIn+1;
  udpBuffer[2] = dirIn;
  udpBuffer[3] = baseIn+1;
  udpBuffer[4] = armIn+1;
  
  udp.beginPacket(ipTarget, REMOTEPORT);

  // Let it fly
  udp.print(udpBuffer);

  udp.endPacket();
}


/*******************************************************
 * Function: waitForGo
 * Sets go boolean once GO! message is received

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
 *******************************************************/
