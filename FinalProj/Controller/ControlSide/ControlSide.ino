/* Lab 3.2: ControlSide
 * Author: Shiv, Victoria, Matt
 * Copyright: I don't have enough money to sue you anyway
 * License: See above
 */

/* -------------------- Includes -------------------- */
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/* -------------------- Defines -------------------- */
// Pins
#define LED_BUILTIN   2
#define TRIM_PIN      33
#define STEERING_PIN  35
#define THROTTLE_PIN  32

// WiFi
#define LOCALPORT     2390
#define REMOTEPORT    2800

// Motor Calibration
#define THROTTLE_OFFSET 175
#define STEERING_OFFSET 155


/* -------------------- Global Varibales -------------------- */
// WiFi
const char* ssid = "Mechatronics";
const char* password = "YayFunFun";
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
  pinMode(TRIM_PIN,INPUT);
  pinMode(STEERING_PIN,INPUT);
  pinMode(THROTTLE_PIN,INPUT);

  // Serial
  Serial.begin(115200);
  delay(10);
  
  // WiFi
  WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  udp.begin(LOCALPORT);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  packetBuffer[UDP_PACKET_SIZE] = 0;

  // Wair for the GO! message
  waitForGo();
}

void loop() {

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
  
  // Write these bits to dirByte so it can be sent as single number
  bitWrite(dirByte, 0, 1);
  bitWrite(dirByte, 1, dir_right);
  bitWrite(dirByte, 2, dir_left);
  
  // Communicate 
  digitalWrite(LED_BUILTIN,HIGH);
  sendPacket(dc_right, dc_left, dirByte);
  digitalWrite(LED_BUILTIN,LOW);
  delay(50);
  
}

/* -------------------- Functions -------------------- */

/*******************************************************
 * Function: sendPacket
 * Collate info into packet and send to car
 *******************************************************/
void sendPacket (int throttleIn, int steeringIn, byte dirIn){
  
  // set all bytes in the buffer to 0
  memset(udpBuffer, 0, UDP_PACKET_SIZE); 

  // Load the buffer, maybe add one so theyre never zero?
  udpBuffer[0] = throttleIn;
  udpBuffer[1] = steeringIn;
  udpBuffer[2] = dirIn;
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
