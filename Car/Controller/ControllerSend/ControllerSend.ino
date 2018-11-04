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
#define TRIM_PIN     23
#define STEERING_PIN  26
#define THROTTLE_PIN   35
#define LOCALPORT     2801
#define REMOTEPORT    2800


/* -------------------- Wifi Setup -------------------- */
const char* ssid = "Greenzang1fl";
const char* password = "503greenzang";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 120);
IPAddress ipTarget(192, 168, 1, 158);
const int UDP_PACKET_SIZE = 100; 
char udpBuffer[UDP_PACKET_SIZE];
byte packetBuffer[UDP_PACKET_SIZE+1];

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
