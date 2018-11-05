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
#define STEERING_PIN  25
#define THROTTLE_PIN   32
#define LOCALPORT     2801
#define REMOTEPORT    2800


/* -------------------- Wifi Setup -------------------- */
const char* ssid = "iPhoneHotspot";
const char* password = "sexpanther";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 120);
IPAddress ipTarget(192, 168, 1, 158);
const int UDP_PACKET_SIZE = 10; 
char udpBuffer[UDP_PACKET_SIZE];
byte packetBuffer[UDP_PACKET_SIZE+1];

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
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
  int throtVal = analogRead(THROTTLE_PIN);
  int steerVal = analogRead(STEERING_PIN);
  Serial.print(trimVal);
  Serial.print(" , ");
  Serial.print(throtVal);
  Serial.print(" , ");
  Serial.println(steerVal);
  

  // Communicate 
  digitalWrite(LED_BUILTIN,HIGH);
  sendPacket(throtVal, steerVal);
  digitalWrite(LED_BUILTIN,LOW);
  delay(100);
  
  

}

void sendPacket (int throttle, int steering){
  
  // set all bytes in the buffer to 0
  memset(udpBuffer, 0, UDP_PACKET_SIZE); 

  //sprintf((char*)udpBuffer,"%u,%u",throttle, steering);

  // Load the buffer
  udpBuffer[0] = throttle+1;
  udpBuffer[1] = steering+1;
  udp.beginPacket(ipTarget, REMOTEPORT);

  // Print what we're going to send to serial
  //Serial.println("----------");
  //Serial.print("Sent: ");
  //Serial.println(udpBuffer[0], BIN);
  //Serial.print("and ");
  //Serial.println(udpBuffer[1], BIN);

  // Let it fly
  udp.print(udpBuffer);
 
  udp.endPacket();
}
