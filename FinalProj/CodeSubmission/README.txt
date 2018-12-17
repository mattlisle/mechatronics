Authors: Shiv Dalla, Matt Lisle, Alejandro Villasmil 
Copyright: My uncle is a lawyer … ?
Date: 12/18/2018
Course: MEAM510 
Final Project README

DriveSidewCooldown.ino: 

This is the code that was uploaded to the robot. It is the integration of our core driving and healing code with the TestRobot.ino that was provided. It also added in some LED ring functionality including indicating weapon cool-down, healing, and, weapon hits. 

To run you must have the following libraries: SPI, WiFi, WiFiUdp, ESPServo (this is Servo.h but I renamed the library as to not conflict with the ‘vanilla’ Arduino Servo library, it is enclosed in our submission) as well as all the files enclosed in the DriveSidewCooldown folder. The folder ESP32-Arduino-Servo-Library-master must be placed in Arduino/libraries. 


ControlSide.ino: 

This code was uploaded to our controller. It is an adaption of our code from the Car Project but adds some extra functionality and values sent for the weapon position and team switching. 

To run you must have the following libraries: SPI, WiFi, WifiUdp, Wire, and SH1106 (enclosed in our submission). The SH1106 library is for our OLED Display on the controller that displays the Wifi status, our current team assignment, and other robot control metrics. The folder esp8266-oled-ssd1306-master folder must be placed in Arduino/libraries. 


The rest of the files/folders 
