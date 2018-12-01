/* Lab 3.2: DriveSide
 * Author: Shiv, Alejo, Matt
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
#define EN1   13
#define H1A   12
#define H2A   14
#define EN3   25
#define H3A   27  
#define H4A   26
#define BASE_SERVO 17 //TODO ADD PINS HERE
#define ARM_SERVO 16 //TODO ADD PINS HERE


// Wifi
#define LOCALPORT       2800
#define REMOTEPORT      2801
#define UDP_PACKET_SIZE 10

/* -------------------- Global Variables -------------------- */
// Motor command variables
byte dc_left;
byte dc_right;
byte dir_left;
byte dir_right;
byte base_pos;
byte arm_pos;

//Intialize Team Boolean, true is blue, false is red
int teamIsBlue; 

// For setting up ledcChannel
byte resolution = 8;
byte left_channel = 1;
byte right_channel = 2;
byte base_channel = 3;
byte arm_channel = 4;
int freq = 200;

// WiFi
const char* ssid = "Mechatronics";
const char* password = "YayFunFun";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 158);
IPAddress ipTarget(192, 168, 1, 120);
char udpBuffer[UDP_PACKET_SIZE];
byte packetBuffer[UDP_PACKET_SIZE + 1];


/* -------------------- ISRs -------------------- */

/* -------------------- Code -------------------- */

void setup() {
  Serial.begin(115200);
  delay(10);

  // Pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(H1A, OUTPUT);
  pinMode(H2A, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(H3A, OUTPUT);
  pinMode(H4A, OUTPUT);

  // PWM for motors
  ledcSetup(left_channel, freq, resolution);
  ledcSetup(right_channel, freq, resolution);
  ledcAttachPin(EN1, left_channel);
  ledcAttachPin(EN3, right_channel);

  // PWM for servos
  ledcSetup(base_channel, freq, resolution);
  ledcSetup(arm_channel, freq, resolution);
  ledcAttachPin(BASE_SERVO, base_channel);
  ledcAttachPin(ARM_SERVO, arm_channel);

  // WiFi
  WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  udp.begin(LOCALPORT);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  packetBuffer[UDP_PACKET_SIZE] = 0;
  
}

void loop() {
    // Read the packet sent by the controller
    readPacket();

    // Use values in packet to control motors
    controlMotors();
}


/* -------------------- Functions -------------------- */

/*******************************************************
 * Function: readPacket
 * Reads packet from Controller into dc and dir variables
 *******************************************************/
void readPacket () {
  
  // Check if there's a packet to read
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // We've received a packet, so let's celebrate
    digitalWrite(LED_BUILTIN,HIGH);

    // Read the packet into the buffer
    udp.read(packetBuffer, UDP_PACKET_SIZE);

    // Parse the first to bytes into duty cycles
    // motors
    dc_right = packetBuffer[0];
    dc_left = packetBuffer[1];
    // servos
    base_pos = packetBuffer[3];
    arm_pos = packetBuffer[4];
    
    // Parse the directions from the third byte
    byte dir = packetBuffer[2];
    dir_right = bitRead(dir, 1);
    dir_left = bitRead(dir,2);
    teamIsBlue = bitRead(dir,3);
    

  }
  // No packet received, sad.
  else{
    digitalWrite(LED_BUILTIN,LOW);
  }
}


/*******************************************************
 * Function: controlMotors
 * Controls duty cycle, direction of PWM to each motor and servo
 *******************************************************/
void controlMotors () {

  // Set and clear H-bridge pins to control direction
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

  // Generate PWM signal based on supplied duty cycle
  ledcWrite(left_channel, dc_left);
  ledcWrite(right_channel, dc_right);
  ledcWrite(base_channel, base_pos);
  ledcWrite(arm_channel, arm_pos);
}
