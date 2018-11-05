/* Car Reciever
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

#define LOCALPORT     2800
#define REMOTEPORT    2801


/* -------------------- Wifi Setup -------------------- */
const char* ssid = "iPhoneHotspot";
const char* password = "sexpanther";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 158);
IPAddress ipTarget(192, 168, 1, 120);
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
  packetBuffer[UDP_PACKET_SIZE] = 0;

}


void loop() {
  // put your main code here, to run repeatedly:
 readPacket();

}

void readPacket () {
  
  // Check if there's a packet to read
  
  int packetSize = udp.parsePacket();
  if (packetSize) {
    digitalWrite(LED_BUILTIN,HIGH);
    udp.read(packetBuffer, UDP_PACKET_SIZE);
    byte val1 = packetBuffer[0];
    byte val2 = packetBuffer[1];

    // Print what we got to serial
    Serial.println("----------");
    Serial.print("Received: ");
    Serial.print(packetBuffer[0]);
    Serial.print(" and ");
    Serial.print(packetBuffer[1]);
    delay(10);
    

  }
  // LED debugging
  else{
    digitalWrite(LED_BUILTIN,LOW);
  }

}
