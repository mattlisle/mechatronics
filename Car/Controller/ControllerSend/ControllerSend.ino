/* Car Controller
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
#define TRIM_PIN     33
#define STEERING_PIN   35
#define THROTTLE_PIN   32
#define LOCALPORT     2801
#define REMOTEPORT    2800


/* -------------------- Wifi Setup -------------------- */
const char* ssid = "Mechatronics";
const char* password = "YayFunFun";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 120);
IPAddress ipTarget(192, 168, 1, 158);
const int UDP_PACKET_SIZE = 10; 
char udpBuffer[UDP_PACKET_SIZE];
byte packetBuffer[UDP_PACKET_SIZE+1];

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(TRIM_PIN,INPUT);
  pinMode(STEERING_PIN,INPUT);
  pinMode(THROTTLE_PIN,INPUT);
  Serial.begin(115200);

  delay(10);
  //wifi setup
  WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  udp.begin(LOCALPORT);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  digitalWrite(LED_BUILTIN,HIGH);
  packetBuffer[UDP_PACKET_SIZE] = 0;

}

void loop() {
  
  int trimVal = analogRead(TRIM_PIN);
  
  //dir val will be either 0, 1, or 2, 3 (2 bits, one for each motor)
  // 0 is backwards, 1 is forwards 
  byte dirVal = 3;
//  Serial.print(trimVal);
//  Serial.print(" , ");
//  Serial.print(throtVal);
//  Serial.print(" , ");
//  Serial.println(steerVal);
  //Throttle goes from 0 (reverse) to 4095 (forward), center at 1872
  //Steering goes from 4095(full left) to 0(full right), center at 1892
  //Trim goes from 4095(full left rot) to 0(full right rot)
  
  //Quick maffs to take these values and make them into control values
  
  // TODO: TUNE THE MAPPING, RESTING POINT ISNT EXACTLY IN THE MIDDLE
  int mag_val = map(analogRead(THROTTLE_PIN), 0, 4095, -255, 255);
  int dir_val = map(analogRead(STEERING_PIN), 0, 4095, -100, 100);

  int dc_right = _min(255, abs(mag_val + dir_val));
  int dc_left = _min(255, abs(mag_val - dir_val));

  int dir_right = (mag_val + dir_val) > 0;
  int dir_left = (mag_val - dir_val) > 0;
  Serial.println("----------");
  Serial.print("mag_val: ");
  Serial.println(mag_val);
  Serial.print("dir_val: ");
  Serial.println(dir_val);
  Serial.print("dir_left: ");
  Serial.println(dir_left);
  Serial.print("dir_right: ");
  Serial.println(dir_right);
  
  //write these bits to dirVal so it can be sent as single number
  bitWrite(dirVal, 0, 1);
  bitWrite(dirVal, 1, dir_right);
  bitWrite(dirVal, 2, dir_left);

  //TODO: ADD TRIM POT VALUE INTO THE MIX

  // Communicate 
  digitalWrite(LED_BUILTIN,HIGH);
  sendPacket(dc_right, dc_left, dirVal);
  digitalWrite(LED_BUILTIN,LOW);
  delay(50);
  
  

}

void sendPacket (int throttleIn, int steeringIn, byte dirIn){
  
  // set all bytes in the buffer to 0
  memset(udpBuffer, 0, UDP_PACKET_SIZE); 

  //sprintf((char*)udpBuffer,"%u,%u",throttle, steering);

  // Load the buffer, maybe add one so theyre never zero?
  udpBuffer[0] = throttleIn;
  udpBuffer[1] = steeringIn;
  udpBuffer[2] = dirIn;
  udp.beginPacket(ipTarget, REMOTEPORT);

  // Print what we're going to send to serial
  //Serial.println("----------");
  //Serial.print("Sent: ");
  //Serial.println(udpBuffer[0], BIN);
  //Serial.print("and ");
  //Serial.println(udpBuffer[1], BIN);
  Serial.print("dirVal: ");
  Serial.print(bitRead(udpBuffer[2], 1), BIN);
  Serial.println(bitRead(udpBuffer[2], 0), BIN);

  // Let it fly
  udp.print(udpBuffer);
 
  udp.endPacket();
}
