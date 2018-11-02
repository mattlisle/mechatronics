/* Problem: 3.1.4
 * Author: Matthew Lisle
 * Copyright: I don't have enough money to sue you anyway
 * License: None
 */

/* -------------------- Includes -------------------- */
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/* -------------------- Defines -------------------- */
#define LED_BUILTIN   2
#define SWITCH_PIN    23
#define LED_PIN       26
#define ADC_PIN       35
#define LOCALPORT     2800
#define REMOTEPORT    2801

/* -------------------- Global Variables -------------------- */
// General
int adc_val = 0;

// Timer
hw_timer_t * timer = NULL;
volatile byte led_state = LOW;
long counts = 1000;
int our_freq = counts / 500;
int their_freq = 3; // Zero is special case

// Wifi
const char* ssid = "Greenzang1fl";
const char* password = "503greenzang";
WiFiUDP udp;
IPAddress myIPaddress(192, 168, 1, 158);
IPAddress ipTarget(192, 168, 1, 120);
const int UDP_PACKET_SIZE = 100; 
char udpBuffer[UDP_PACKET_SIZE];
byte packetBuffer[UDP_PACKET_SIZE+1];

/***********************************
 * Function: IRAM_ATTR
 * Inputs: None
 * Outputs: None
 * ISR for LED state control
 ***********************************/
void IRAM_ATTR onTimer(){
  led_state = !led_state;
  digitalWrite(LED_PIN, led_state);
}

/* -------------------- Code -------------------- */

void setup() {
  Serial.begin(115200);
  delay(10);

  WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  udp.begin(LOCALPORT);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  packetBuffer[UDP_PACKET_SIZE] = 0;

  // Configure pins
  pinMode(SWITCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ADC_PIN, INPUT_PULLDOWN);

  // Timer setup
  timer = timerBegin(0, 8000, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, counts, true);
  timerAlarmEnable(timer);
}

void loop() {
  byte freq_match;

  // Controls interrupt alarm trigger and updates our_freq
  setFrequency();
  
  // If the switch is on, end the game
  if (digitalRead(SWITCH_PIN)) {
    freq_match = compareFreq();
    tryToWin(freq_match);
  }

  // If they've sent anthing, read it
  readPacket();

  // Communicate our frequency
  sendPacket();
  
  // So we don't overwhelm anything
  delay(1);
}


/* -------------------- Functions -------------------- */

/*******************************************************
 * Function: tryToWin
 * Inputs: freq_match - boolean true if freqs w/in 10%
 * Outputs: None
 * Figures out the winner and tells the other what to do
 *******************************************************/
void tryToWin (byte freq_match) {
  
  // Pick values that will never otherwise be communicated
  byte communique = 0xFF * freq_match;

  // Send value of winner to other board
  udpBuffer[0] = communique;
  udp.beginPacket(ipTarget, REMOTEPORT);
  Serial.println("----------");
  Serial.print("Sent: ");
  Serial.println(udpBuffer[0], BIN);
  udp.write((byte)udpBuffer[0]);
  udp.endPacket();

  // Run game over function
  gameOver(freq_match);
}


/*******************************************************
 * Function: gameOver
 * Inputs: winner - boolean true if we won
 * Outputs: None
 * Controls built-in LED based on winner and resets game
 *******************************************************/
void gameOver (byte winner) {
  // Signal if we won
  if (winner) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("WE WON");
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("You really let us down. I'm disappointed in you.");
  }

  // Give it a bit before we startup again
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
}


/*******************************************************
 * Function: setFrequency
 * Inputs: None
 * Outputs: None
 * Controls frequency of PWM blinking
 *******************************************************/
void setFrequency () {
  adc_val = analogRead(ADC_PIN);
  counts = map(adc_val, 0, 4095, 500, 50000);
  our_freq = counts / 500;
  timerAlarmWrite(timer, counts, true);
}


/*******************************************************
 * Function: compareFreq
 * Inputs: None
 * Outputs: freq_match
 * Returns 1 if frequencies are w/in 10% of e.o.
 *******************************************************/
byte compareFreq () {
  byte bigger;
  byte smaller;
  byte difference;
  
  // Update our frequency
  our_freq = counts / 500;
  Serial.print("Our Frequency: ");
  Serial.println(our_freq);
  Serial.print("Their Frequency: ");
  Serial.println(their_freq);

  // Compute if the frequencies match
  bigger = max((byte)our_freq, (byte)their_freq);
  smaller = min((byte)our_freq, (byte)their_freq);
  difference = bigger - (byte)(bigger * 0.9);
  
  return (bigger - smaller <= difference);
}


/*******************************************************
 * Function: readPacket
 * Inputs: None
 * Outputs: None
 * Updates their_freq if packet has been received
 *******************************************************/
void readPacket () {
  
  // Check if there's a packet to read
  int packetSize = udp.parsePacket();
  if (packetSize) {
    udp.read(packetBuffer, UDP_PACKET_SIZE);
    their_freq = packetBuffer[0];

    // Print what we got to serial
    Serial.println("----------");
    Serial.print("Received: ");
    Serial.println(their_freq, BIN);

    // Check to see if they tried to win
    if ((their_freq == 0) | (their_freq == 0xFF)) {
      byte winner = (their_freq == 0);
      gameOver(winner);
    }
  }

}


/*******************************************************
 * Function: sendPacket
 * Inputs: None
 * Outputs: None
 * Sends our_freq to their IP address
 *******************************************************/
void sendPacket () {

  // Load the buffer
  udpBuffer[0] = our_freq;
  udp.beginPacket(ipTarget, REMOTEPORT);

  // Print what we're going to send to serial
  Serial.println("----------");
  Serial.print("Sent: ");
  Serial.println(udpBuffer[0], BIN);

  // Let it fly
  udp.write((byte)udpBuffer[0]);
  udp.endPacket();
  
}
