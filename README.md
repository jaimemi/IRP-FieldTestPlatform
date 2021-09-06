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

### 3.3. CAN bus
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

## 5. CAN IDs
Each measurement has associated a CAN address or ID. If the CAN message contains a measurement from a sensor, the Cloud variable associated to it changes its value. Then, the measurement can be seen from the Cloud.

| Unit    | Sensor   | Measurement                              | ID Number | Cloud variable     |
|---------|----------|------------------------------------------|-----------|--------------------|
| ALL     | REV3028  | Ask about time and date                  | 0x0000FF  | -                  |
| Climate | BMP280   | Temperature                              | 0x000100  | temp_ws            |
| Climate | BMP280   | Pressure                                 | 0x000101  | pres_ws            |
| Climate | BMP280   | Altitude                                 | 0x000102  | alt_ws             |
| Climate | AS7341   | Channel 415 nm                           | 0x000110  | counts_ws_415nm_F1 |
| Climate | AS7341   | Channel 445 nm                           | 0x000111  | counts_ws_445nm_F1 |
| Climate | AS7341   | Channel 480 nm                           | 0x000112  | counts_ws_480nm_F1 |
| Climate | AS7341   | Channel 515 nm                           | 0x000113  | counts_ws_515nm_F1 |
| Climate | AS7341   | Channel 555 nm                           | 0x000114  | counts_ws_555nm_F1 |
| Climate | AS7341   | Channel 590 nm                           | 0x000115  | counts_ws_590nm_F1 |
| Climate | AS7341   | Channel 630 nm                           | 0x000116  | counts_ws_630nm_F1 |
| Climate | AS7341   | Channel 680 nm                           | 0x000117  | counts_ws_680nm_F1 |
| Climate | AS7341   | Channel CLEAR                            | 0x000118  | counts_ws_CLEAR    |
| Climate | AS7341   | Channel NIR                              | 0x000119  | counts_ws_NIR      |
| SC      | MAX31855 | Amplifier Temp. (Right-Top-w\ SC)        | 0x000200  | temp_pv_int_rtf    |
| SC      | MAX31855 | Thermocouple Temp. (Right-Top-w\ SC)     | 0x000201  | temp_pv_k_rtf      |
| SC      | MAX31855 | Amplifier Temp. (Left-Top-w\ SC)         | 0x000210  | temp_pv_int_ltf    |
| SC      | MAX31855 | Thermocouple Temp. (Left-Top-w\ SC)      | 0x000211  | temp_pv_k_ltf      |
| SC      | MAX31855 | Amplifier Temp. (Right-Bottom-w\ SC)     | 0x000220  | temp_pv_int_rbf    |
| SC      | MAX31855 | Thermocouple Temp. (Right-Bottom-w\ SC)  | 0x000221  | temp_pv_k_rbf      |
| SC      | MAX31855 | Amplifier Temp. (Left-Bottom-w\ SC)      | 0x000230  | temp_pv_int_lbf    |
| SC      | MAX31855 | Thermocouple Temp. (Left-Bottom-w\ SC)   | 0x000231  | temp_pv_k_lbf      |
| SC      | MAX31855 | Amplifier Temp. (Right-Top-w\o SC)       | 0x000240  | temp_pv_int_rtnf   |
| SC      | MAX31855 | Thermocouple Temp. (Right-Top-w\o SC)    | 0x000241  | temp_pv_k_rtnf     |
| SC      | MAX31855 | Amplifier Temp. (Left-Top-w\o SC)        | 0x000250  | temp_pv_int_ltnf   |
| SC      | MAX31855 | Thermocouple Temp. (Left-Top-w\o SC)     | 0x000251  | temp_pv_k_ltnf     |
| SC      | MAX31855 | Amplifier Temp. (Right-Bottom-w\o SC)    | 0x000260  | temp_pv_int_rbnf   |
| SC      | MAX31855 | Thermocouple Temp. (Right-Bottom-w\o SC) | 0x000261  | temp_pv_k_rbnf     |
| SC      | MAX31855 | Amplifier Temp. (Left-Bottom-w\o SC)     | 0x000270  | temp_pv_int_lbnf   |
| SC      | MAX31855 | Thermocouple Temp. (Left-Bottom-w\o SC)  | 0x000271  | temp_pv_k_lbnf     |
| SC      | INA260   | Current (w\ SC)                          | 0x000280  | i_pv_f             |
| SC      | INA260   | Voltage (w\ SC)                          | 0x000281  | v_pv_f             |
| SC      | INA260   | Power (w\ SC)                            | 0x000282  | p_pv_f             |
| SC      | INA260   | Current (w\o SC)                         | 0x000290  | i_pv_nf            |
| SC      | INA260   | Voltage (w\o SC)                         | 0x000291  | v_pv_nf            |
| SC      | INA260   | Power (w\o SC)                           | 0x000292  | p_pv_nf            |
| ECD     | MAX31855 | Amplifier Temp. (1)                      | 0x000300  | temp_ec_int_1      |
| ECD     | MAX31855 | Thermocouple Temp. (1)                   | 0x000301  | temp_ec_k_1        |
| ECD     | MAX31855 | Amplifier Temp. (2)                      | 0x000310  | temp_ec_int_2      |
| ECD     | MAX31855 | Thermocouple Temp. (2)                   | 0x000311  | temp_ec_k_2        |
| ECD     | MAX31855 | Amplifier Temp. (3)                      | 0x000320  | temp_ec_int_3      |
| ECD     | MAX31855 | Thermocouple Temp. (3)                   | 0x000321  | temp_ec_k_3        |
| ECD     | INA260   | Current                                  | 0x000380  | i_ec               |
| ECD     | INA260   | Voltage                                  | 0x000381  | v_ec               |
| ECD     | INA260   | Power                                    | 0x000382  | p_ec               |
| ECD     | AS7341   | Channel 415 nm                           | 0x000390  | counts_ec_415nm_F1 |
| ECD     | AS7341   | Channel 445 nm                           | 0x000391  | counts_ec_445nm_F1 |
| ECD     | AS7341   | Channel 480 nm                           | 0x000392  | counts_ec_480nm_F1 |
| ECD     | AS7341   | Channel 515 nm                           | 0x000393  | counts_ec_515nm_F1 |
| ECD     | AS7341   | Channel 555 nm                           | 0x000394  | counts_ec_555nm_F1 |
| ECD     | AS7341   | Channel 590 nm                           | 0x000395  | counts_ec_590nm_F1 |
| ECD     | AS7341   | Channel 630 nm                           | 0x000396  | counts_ec_630nm_F1 |
| ECD     | AS7341   | Channel 680 nm                           | 0x000397  | counts_ec_680nm_F1 |
| ECD     | AS7341   | Channel CLEAR                            | 0x000398  | counts_ec_CLEAR    |
| ECD     | AS7341   | Channel NIR                              | 0x000399  | counts_ec_NIR      |
