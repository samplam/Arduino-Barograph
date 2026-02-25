# Arduino-Barograph
Program to build a barograph with a e-ink display, a VOC sensor, a CO2 sensor, a temperature sensor, a humidity sensor, 3 RBG LEDs and a fan controller. Runs on an Arduino Mega.

## Overview
This project is an Arduino-based that I made to display on an e-ink display the barometric pressure history of the last 72 hours. It also includes some calculations for the rate of pressure change, the number of pressure variations in the last 12 hours, the time elapsed since the last pressure variation and the maximum + minimum pressure in the last 72 hours. There is a fan controller for keeping noise down and provide case ventilation for heat and VOC + CO2 circulation. The RBG LEDs are used to display basic pressure information readable from a long distance (1 for actual pressure, 1 for the pressure variation and 1 for the prediction). Some things in the code are overkill, in particular how the pressure is recorded.

## Features
- Basic automation loops and logic
- Commented and structured code for readability

## Skills Highlighted
- Programming in C/C++ for microcontrollers
- Basic electronics and automation principles

## Usage
1. Open `Barograph_v2020-08-01.ino` in Arduino IDE
2. Upload to the Arduino board
3. Connect the input and output as per requirements
4. Test the program

## Notes
- Written in 2020 before my formal IT studies; shows structured code and practical electronics knowledge. Also note that no functions were used and that the core of the code is based around several nested IF structures.
