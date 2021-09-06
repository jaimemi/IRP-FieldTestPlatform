# IRP-FieldTestPlatform

This repository contains all the code the Control Units from the Field Test Platform contains. The control units are:
- Climate Unit: measures temperature and pressure at the test location.
- SC Unit: evaluates performance of SC.
- ECD Unit: evaluates performance of ECD.
- WiFi Unit: communicates with the Cloud.

If any quesiton, drop your query at the "Issues" section!

## CAN bus
- The WiFi unit uses the CAN library available at [this repository](https://github.com/sandeepmistry/arduino-CAN)
- The SC, ECD and Climate Units use the CAN library availbale at [this repository](https://github.com/Seeed-Studio/Seeed_Arduino_CAN)

## How to install the code
### Climate, SC and ECD units
Just import the `.ino` file into the Arduino IDE and download the required libraries.
### WiFi unit
1. Import the `.ino` file into the Arduino IDE
2. Drag the libraries `arduino_secrets.h` and `thingProperties.h` if they do not show in the IDE
3. Import the library `Arduino_ConnectionHandler.zip` available at the folder "Custom Libraries" into the Arduino IDE
4. Fill the SSID, USER and PASWORD fields at `arduino_sectrets.h` on the Arduino IDE.

Alternatively, you can use the Arduino IoT Cloud Editor to open the program, it will be easier to import a Custom Library.
