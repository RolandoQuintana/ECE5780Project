# README - ECE5780 Final Project: Rescue Rover

This readme document serves as a guide for our rescue rover. It includes the following sections:

1. Purpose
2. Functionality
3. Wiring/Schematics
4. Setup

## 1. Purpose

The purpose of our rover is to manually navigate an obstacle course/maze where the drivers are not able to see the course themselves.

This proposal outlines the project plan for the development of our teams’ final project for Embedded Systems. Our rescue rover will use a camera and send images over the provided USART → Bluetooth → USART connection to the users. The images will be sent as monochrome 160x120-bit images. The users will then use the images to determine which way the robot should move, controlling our omnidirectional rover base. For controlling the rover, an X-box controller will be used to move the robot forward, backward, left, and right respectively. The side-to-side movement is made possible by the omnidirectional wheels. To turn clockwise and counterclockwise, the secondary joystick can be used. 

## 2. Functionality

We’ll be demonstrating UART for communication to the user interface, I2C and SPI for the camera connection, and PWM for motor control from the labs as our three+ techniques. Additional techniques we plan on using include timers, for sampling data from the camera and motor encoders, and interrupts, for handling commands sent to the rover.

FLOW CHART:

![Software flowchart.png](software_flowchart.png)

## 3. Wiring/Schematics

Wiring Diagram:

![Wiring Diagram](wiring_diagram.png)

## 4. Setup

Here, you will find step-by-step instructions on how to set up the rescue rover. 

1. Ensure all devices are Powered on and wires are connected wherever applicable
    1. Rover
    2. X-box controller
2. Ensure the correct C code is flashed onto the STM32
3. Connect devices to the computer via Bluetooth
    1. Rover 
    2. X-box controller
4. Run the Main Python program on the computer (be sure the correct comport is selected for the HC-05 module on the rover).
