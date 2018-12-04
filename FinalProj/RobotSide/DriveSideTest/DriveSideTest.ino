/* Lab 3.2: DriveSide
 * Author: Shiv, Alejo, Matt
 * Copyright: I don't have enough money to sue you anyway
 * License: See above
 */

/* -------------------- Includes -------------------- */
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPServo.h>


/* -------------------- Defines -------------------- */
// Pins
#define LED_BUILTIN   2
#define EN1   13
#define EN3   12
// direction of zero is backwards in both cases
#define DIR1  14
#define DIR2  27


#define BASE_SERVO 25 //TODO ADD PINS HERE
#define ARM_SERVO 26 //TODO ADD PINS HERE

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
byte left_channel = 9;
byte right_channel = 10;
int freq = 200;

// WiFi
const char* ssid = "Mechatronics";
const char* password = "YayFunFun";
//const char* ssid = "iPhoneHotspot";
//const char* password = "sexpanther";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 158);
IPAddress ipTarget(192, 168, 1, 120);
char udpBuffer[UDP_PACKET_SIZE];
byte packetBuffer[UDP_PACKET_SIZE + 1];

// Servos
Servo baseServo;
Servo armServo;


/* -------------------- ISRs -------------------- */

/* -------------------- Code -------------------- */

void setup() {
  Serial.begin(115200);
  delay(10);

  // Pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(EN3, OUTPUT);
 
  // Attach servos
  baseServo.attach(BASE_SERVO);
  armServo.attach(ARM_SERVO);
  
  // PWM
  ledcSetup(left_channel, freq, resolution);
  ledcSetup(right_channel, freq, resolution);
  ledcAttachPin(EN1, left_channel);
  ledcAttachPin(EN3, right_channel);

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
    dc_right = packetBuffer[0];
    dc_left = packetBuffer[1];
    // servos
    base_pos = packetBuffer[3];
    arm_pos = packetBuffer[4];

    Serial.println(arm_pos);

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
 * Controls duty cycle, direction of PWM to each motor
 *******************************************************/
void controlMotors () {

  // Set and clear H-bridge pins to control direction
  if (dir_left) {
    digitalWrite(DIR1, LOW);
  } else {
    digitalWrite(DIR1, HIGH);
    
  }
  if (dir_right) {
    digitalWrite(DIR2, LOW);
  } else {
    digitalWrite(DIR2, HIGH);
  }

  // Generate PWM signal based on supplied duty cycle
  ledcWrite(left_channel, dc_left);
  ledcWrite(right_channel, dc_right);
  // Servos
  baseServo.write(base_pos);
  armServo.write(arm_pos);
}
