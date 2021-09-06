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
### 2.1. Climate, SC and ECD Unit
1. Once the Unit is plugged into the battery, it goes into `SETUP` mode. In `SETUP`, all the sensors, pheripherals (except SD and CAN Modules), and IO Pins are initialised.
2. If the switch is toggled, the unit asks for toggling the switch back. Once the switch is toggled, the unit goes into `SLEEP` mode, which means the SD can be extracted safely.
3. Once the switch is toggled, the unit goes into `INITIALISATION` mode. Here, the SD and CAN Modules are initialised. In addition, an alarm is set every period of time stablished by the user (10s by default).
4. The unit asks the current time and date to the WiFi Unit by CAN. Once received, it goes into `READING` mode.
5. The unit can go at any time into `SLEEP` mode by toggling the switch. The SD cart can be then extracted safely.
6. Once the alarm is triggered every period of time (10s by default), the unit reads all the sensors, stores the information into the SD Card and send it over CAN.
7. If an error occurs while initialisation or writing into the SD card or RTC, the unit goes into `ERROR` mode and must be restarted.

![ClimateUnitLogic](https://user-images.githubusercontent.com/42868962/132248611-a8fe3be6-e839-43cb-9d25-37b08ed77b5e.png)

### 2.2. WiFi Unit
1. Once the Unit is plugged into the battery, it goes into `SETUP` mode. In `SETUP` and the pheripherals are initialised.
2. As soon as the Unit receives a CAN message, it filters the address to see which Unit it belongs to and either send the current date and time or update the variables the information is comming from.

![WiFiUnitLogic](https://user-images.githubusercontent.com/42868962/132249259-2faa20dd-25ab-4372-858b-5eaf8b50c734.png)

## 3. How to install the code
### 3.1. Climate, SC and ECD units
Just import the `.ino` file into the Arduino IDE and download the required libraries.

### 3.2. WiFi unit
1. Import the `.ino` file into the Arduino IDE
2. Drag the libraries `arduino_secrets.h` and `thingProperties.h` if they do not show in the IDE
3. Import the library `Arduino_ConnectionHandler.zip` available at the folder "Custom Libraries" into the Arduino IDE
4. Fill the SSID, USER and PASWORD fields at `arduino_sectrets.h` on the Arduino IDE.

## 3.3. CAN bus
- The WiFi unit uses the CAN library available at [this repository](https://github.com/sandeepmistry/arduino-CAN)
- The SC, ECD and Climate Units use the CAN library availbale at [this repository](https://github.com/Seeed-Studio/Seeed_Arduino_CAN)

Alternatively, you can use the Arduino IoT Cloud Editor to open the program, it will be easier to import a Custom Library.

## 4. Error IDs
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


