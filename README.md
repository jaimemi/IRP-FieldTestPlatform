# IRP-FieldTestPlatform

This repository contains all the code the Control Units from the Field Test Platform contains. The control units are:
- Climate Unit: measures temperature and pressure at the test location.
- SC Unit: evaluates performance of SC.
- ECD Unit: evaluates performance of ECD.
- WiFi Unit: communicates with the Cloud.

If any quesiton, drop your query at the "Issues" section!

## 1. Platform General Schematic
![GeneralSchematic](https://user-images.githubusercontent.com/42868962/132248472-2921787c-5292-4810-903e-16d5eed5d239.png)

## 2. Units Logic
### Climate, SC and ECD Unit
1. Once the Unit is plugged into the battery, 
![ClimateUnitLogic](https://user-images.githubusercontent.com/42868962/132248611-a8fe3be6-e839-43cb-9d25-37b08ed77b5e.png)


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

## Error IDs LookUp Table
Each pheripheral has associated an error ID. In case such pheripheral does not work as expected, the error will be shown in the LCD screen and the Arduino will fall into an infinite loop.

| Function Type | Function Name         | ID Name                 | ID Number |
|---------------|-----------------------|-------------------------|-----------|
| Init          | RV3028Init()          | ERROR_ID_RV3028_INIT    | 11        |
| Init          | MAXInit()             | ERROR_ID_MAX314855_INIT | 12        |
| Init          | Ina260Init(INA260_F)  | ERROR_ID_INA260_F_INIT  | 13        |
| Init          | Ina260Init(INA260_NF) | ERROR_ID_INA260_NF_INIT | 14        |
| Init          | SDCardInit()          | ERROR_ID_SD_INIT        | 15        |
| Init          | BMP280Init()          | ERROR_ID_BMP280_INIT    | 16        |
| Init          | AS7341Init()          | ERROR_ID_AS7341_INIT    | 17        |
| Init          | CANInit()             | ERROR_ID_CAN_INIT       | 18        |
| Init          | Ina260Init()          | ERROR_ID_INA260_INIT    | 19        |
| Write         | RV3028Set()           | ERROR_ID_RV3028_WRITE   | 21        |
| Write         | SDCardWrite()         | ERROR_ID_SD_WRITE       | 25        |


