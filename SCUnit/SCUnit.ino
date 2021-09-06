#include <RV-3028-C7.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_MAX31855.h"
#include <Adafruit_INA260.h>
#include <LiquidCrystal_I2C.h>
#include "mcp2515_can.h"

/***************************** Customizable variables ********************************************/
// Sampling time (s)
#define TIME_SAMP 10 // Seconds

// Time zone
#define GMT 1 // UK

// .CSV Name and Header
#define FILE_NAME "pvlog.csv"
#define FILE_SEPARATOR ","


/************************************ Layout ******************************************************/
// SD SPI interface
#define SD_CS 10

// CAN interface
#define CAN_CS 22
#define CAN_INT 19

// RTC interrupt
#define RTC_INT 3

// Card extract/introduce button interrupt
#define TOGGLE_INT 2

// CS Pins multiplexer
const int n_selectPins = 3;
const int selectPins[n_selectPins] = {4,5,6};

// MAX31855 SPI Interface
#define MAX_DO   8
#define MAX_CS   7
#define MAX_SCK  9

// Number of Thermocouples
#define MAX_NUM 8

// Thermocouples address (0-16) || R - right | L - left || T - Top | B - Bottom || F - With film | NF - With no film
#define THERMOCOUPLE_RTF 0
#define THERMOCOUPLE_LTF 1
#define THERMOCOUPLE_RBF 2
#define THERMOCOUPLE_LBF 3
#define THERMOCOUPLE_RTNF 4
#define THERMOCOUPLE_LTNF 5
#define THERMOCOUPLE_RBNF 6
#define THERMOCOUPLE_LBNF 7

// Ina260 address (0-7) || F - With Film | NF - With no film
#define INA260_F 0x40
#define INA260_NF 0x41


/************************************* Internal Variables *****************************************/
// Internal States
#define STATE_SETUP 0
#define STATE_INIT 1
#define STATE_READ_SENSOR 2
#define STATE_SLEEP 3
# define STATE_ERROR 4

volatile int next_state;
volatile int state;

volatile bool init_request = false;
volatile bool read_request = false;
volatile bool sleep_request = false;

//RCT set-up time
byte second, minute, hour, date, month, year;

// Sensor objects
RV3028 rtc;
Adafruit_MAX31855 thermocouple(MAX_SCK, MAX_CS, MAX_DO);
Adafruit_INA260 ina260_F = Adafruit_INA260();
Adafruit_INA260 ina260_NF = Adafruit_INA260();
mcp2515_can CAN(CAN_CS);

// LCD Object
LiquidCrystal_I2C lcd(0x27, 16, 2);

// SD write object
String SD_buffer = "";

// Global error flag
volatile bool error_flag = false;

// Error write buffer
String ERROR_buffer = "";

// Error write separator
#define ERROR_SEPARATOR " "

// CAN Read RTC Objects
volatile unsigned char flag_CAN_recv = 0;
unsigned char lenRecv = 0;
unsigned char bufRecv[8];

File myFile;

/*************************************      IDs          *****************************************/

// Error IDs
#define ERROR_ID_RV3028_INIT 11
#define ERROR_ID_MAX31855_INIT 12
#define ERROR_ID_INA260_F_INIT 13
#define ERROR_ID_INA260_NF_INIT 14
#define ERROR_ID_CAN_INIT 18
#define ERROR_ID_SD_INIT 15
#define ERROR_ID_RV3028_WRITE 21
#define ERROR_ID_SD_WRITE 25

// Platform ID
#define ID_WIFI_PLATFORM            0x001000
#define ID_WS_PLATFORM              0x000100
#define ID_PV_PLATFORM              0x000200
#define ID_EC_PLATFORM              0x000300

#define ID_MY_PLATFORM ID_PV_PLATFORM

// Sensor IDs
#define ID_THERMOCOUPLE_RTF 0x000
#define ID_THERMOCOUPLE_LTF 0x010
#define ID_THERMOCOUPLE_RBF 0x020
#define ID_THERMOCOUPLE_LBF 0x030
#define ID_THERMOCOUPLE_RTNF 0x040
#define ID_THERMOCOUPLE_LTNF 0x050
#define ID_THERMOCOUPLE_RBNF 0x060
#define ID_THERMOCOUPLE_LBNF 0x070
#define ID_INA260_F 0x080
#define ID_INA260_NF 0x090

// Measurement IDs
#define ID_THERMOCOUPLE_INT 0x000
#define ID_THERMOCOUPLE_K 0x001
#define ID_INA260_I 0x000
#define ID_INA260_V 0x001
#define ID_INA260_P 0x002

// RTC request ID
#define ID_RTC_REQUEST              0xFF

/******************************************* Code **************************************************/
void setup() {

  state = STATE_SETUP;
  
  Wire.begin();

//  SERIAL_PORT_MONITOR.begin(9600);

  // Initializing RTC
  error_flag |= RV3028Init();

  // Initializing Mux
  error_flag |= MuxInit();

  // Initializing MAX31855
  error_flag |= MAXInit();

  // Initializing Ina260
  error_flag |= Ina260Init(INA260_F);
  error_flag |= Ina260Init(INA260_NF);
  
  // Initializing RTC Interrupt
  error_flag |= RTCINTInit();

  // Initializing SD extract/insert interrupt
  error_flag |= SDINTInit();

  // LCD inizialization
  error_flag|= LCDInit();

  delay(2000);
  
  if (error_flag) {
    next_state = STATE_ERROR;
    LCDWriteError();
  } else {
    while (ToggleRead() == HIGH) {
      LCDWriteToggleSwitch(); 
      delay(500);
    }
    next_state = STATE_SLEEP;
    LCDWriteState();
  }
}

void loop() {
  
  switch (next_state) {

    case STATE_INIT:
      {
        state = STATE_INIT;  
    
        // Initializing SD Card
        error_flag |= SDCardInit();

        // CAN Initialization
        error_flag |= CANInit();

        if (!error_flag) {
          LCDWriteRTCRequest();
          do {
           CAN.sendMsgBuf(ID_RTC_REQUEST, 0, 1, 0);
           delay(1000);
          } while (!CANRead());
          error_flag |= RV3028Write();
          LCDWriteRTCAnswer();
          delay(10000);
        }

        // Enable RTC interrupts
        rtc.enableTimerInterrupt();

        // Clean request flags
        init_request = false;
        read_request = false;
        sleep_request = false;

        if (error_flag) {
          next_state = STATE_ERROR;
          LCDWriteError();
        } else {        
          next_state = STATE_READ_SENSOR;
          LCDWriteState();
        }
      }
      break;
    
    case STATE_READ_SENSOR:
      {
        state = STATE_READ_SENSOR;
        
        if (read_request) {
          read_request = false;
          
          // Read RTC
          RV3028Read();

          // Read MAX31855
          MAXRead(ID_THERMOCOUPLE_RTF);
          MAXRead(ID_THERMOCOUPLE_LTF);
          MAXRead(ID_THERMOCOUPLE_RBF);
          MAXRead(ID_THERMOCOUPLE_LBF);
          MAXRead(ID_THERMOCOUPLE_RTNF);
          MAXRead(ID_THERMOCOUPLE_LTNF);
          MAXRead(ID_THERMOCOUPLE_RBNF);
          MAXRead(ID_THERMOCOUPLE_LBNF);   

          // Read Ina260
          Ina260Read(ID_INA260_F);
          Ina260Read(ID_INA260_NF);
  
          // Write data
//          SD_bufferRead();
          error_flag |= SDCardWrite();      
        }      
        
        if (error_flag) {
          next_state = STATE_ERROR;
          LCDWriteError();
        } else {
          if (sleep_request) {
            next_state = STATE_SLEEP;
            LCDWriteState();
          }       
        }
      }
      break;

    case STATE_SLEEP:
      {
        state= STATE_SLEEP;
        if (init_request) {
          next_state = STATE_INIT;
          LCDWriteState();
        }
      }
      break;

    case STATE_ERROR:
      {
        ERROR_bufferRead();
        while (1);
      }
      break;

    default:
      break;
  }
}

/***************************************** Init Functions *******************************************/

bool RV3028Init() {
  bool error = false;
  if (!rtc.begin()) {
    ERROR_buffer += ERROR_ID_RV3028_INIT;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  }
  return error;
}

bool MuxInit() {
  bool error = false;
  for (uint8_t i=0; i<n_selectPins; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], LOW);
  }
  return error;
}

bool MAXInit() {
  bool error = false;
  if (!thermocouple.begin()) {
    ERROR_buffer += ERROR_ID_MAX31855_INIT;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  }
  return error;
}

bool Ina260Init(uint8_t ina_num) {
  bool error = false;
  switch (ina_num) {
    case INA260_F:
    {     
      if (!ina260_F.begin(ina_num)) {
        ERROR_buffer += ERROR_ID_INA260_F_INIT;
        ERROR_buffer += ERROR_SEPARATOR;
        error = true;
      }
    }
    break;
    case INA260_NF:
    {
      if (!ina260_NF.begin(ina_num)) {
        ERROR_buffer += ERROR_ID_INA260_NF_INIT;
        ERROR_buffer += ERROR_SEPARATOR;
        error = true;
      }
    }
    break;
    default:
    break;
  }
  return error;  
}

bool RTCINTInit() {
  bool error = false;
  rtc.setTimer(true, 1, TIME_SAMP, true, true);
  pinMode(RTC_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(RTC_INT), SensorRead, FALLING);
  return error;
}

bool SDINTInit() {
  bool error = false;
  pinMode(TOGGLE_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(TOGGLE_INT), SDInOut, CHANGE);
  return error;
}

bool SDCardInit() {
  bool error = false;
  if (!SD.begin(SD_CS)) {
    ERROR_buffer += ERROR_ID_SD_INIT;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  }
  return error;
}

bool LCDInit() {
  bool error = false;
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Hi!");
  return error;
}

bool CANInit() {
  bool error = false;
  attachInterrupt(digitalPinToInterrupt(CAN_INT), MCP2515_ISR, FALLING); // start interrupt
  if (CAN_OK != CAN.begin(CAN_125KBPS, MCP_16MHz)) {
    ERROR_buffer += ERROR_ID_CAN_INIT;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  }
  return error;
}

/***************************************** Write Functions *******************************************/
bool RV3028Write() {
  bool error = false;
  if(!rtc.setTime(second, minute, hour, 0, date, month, year)){
    ERROR_buffer += ERROR_ID_RV3028_WRITE;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  }
  return error;
}

bool SDCardWrite() {
  bool error = false;
  File dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile) {
    dataFile.println(SD_buffer);
    dataFile.close();
    SD_buffer = "";
  } else {    
    ERROR_buffer += ERROR_ID_SD_WRITE;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  }
  return error;
}

void LCDWriteToggleSwitch () {
  lcd.setCursor(0,1);
  lcd.print("Switch toggle...");
}

void LCDWriteState() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("State:");
  lcd.setCursor(0,1);
  switch (next_state) {
    case STATE_INIT:
    {
      lcd.print("Initiating");
    }
    break;
    case STATE_READ_SENSOR:
    {
      lcd.print("Reading");
    }
    break;
    case STATE_SLEEP:
    {
      lcd.print("Sleeping");
    }
    break;
    default:
    {
      lcd.print("Error!");
    }
    break;
  }
}

void LCDWriteError() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Error:");
  lcd.setCursor(0,1);
  lcd.print(ERROR_buffer);
}

void LCDWriteRTCRequest() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Asking for RT...");
}

void LCDWriteRTCAnswer() {
  rtc.updateTime();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(rtc.stringDate());
  lcd.setCursor(0,1);
  lcd.print(rtc.stringTime());
}

void CANWrite(unsigned char msg_id, String msg_data) {
  unsigned char len = msg_data.length();
  unsigned char buf[len];
  msg_data.toCharArray(buf, len);
  CAN.sendMsgBuf(ID_MY_PLATFORM|msg_id, 1, len, buf);
//  SERIAL_PORT_MONITOR.println("Message sent!");
  delay(100);
}

/***************************************** Read Functions *******************************************/
bool ToggleRead() {
  return digitalRead(TOGGLE_INT);
}

void RV3028Read() {
  rtc.updateTime();
  SD_buffer += rtc.stringDate();
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += rtc.stringTime();
}

void MAXRead(uint16_t therm_num) {
  String temp_int, temp_k;
  selectMuxPin(therm_num>>4);

  temp_int = thermocouple.readInternal();
  temp_k = thermocouple.readCelsius();
  
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += temp_int;
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += temp_k;
  
  CANWrite(therm_num | ID_THERMOCOUPLE_INT, temp_int);
  CANWrite(therm_num | ID_THERMOCOUPLE_K, temp_k);
}

void Ina260Read(uint16_t ina_num) {
  String current_s, voltage_s, power_s;
  switch (ina_num) {
    case ID_INA260_F:
    {
      current_s = String(int(ina260_F.readCurrent()));
      voltage_s = String(int(ina260_F.readBusVoltage()));
      power_s = String(int(ina260_F.readPower())); 
    }
    break;

    case ID_INA260_NF:
    { 
      current_s = String(int(ina260_NF.readCurrent()));
      voltage_s = String(int(ina260_NF.readBusVoltage()));
      power_s = String(int(ina260_NF.readPower()));
    }
    break;

    default:
    break;
  }
  
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += current_s;
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += voltage_s;
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += power_s;
  
  CANWrite(ina_num | ID_INA260_I, current_s);
  CANWrite(ina_num | ID_INA260_V, voltage_s); 
  CANWrite(ina_num | ID_INA260_P, power_s); 
}

bool CANRead() {
  bool done_reading = false;

  if (flag_CAN_recv) {
        // check if get data

        flag_CAN_recv = 0;                   // clear flag
        unsigned long id = 0;

        myFile = SD.open("can.csv", FILE_WRITE);

        // iterate over all pending messages
        // If either the bus is saturated or the MCU is busy,
        // both RX buffers may be in use and reading a single
        // message does not clear the IRQ conditon.
        while (CAN_MSGAVAIL == CAN.checkReceive()) {
            // read data,  len: data length, buf: data buf
            CAN.readMsgBufID(&id, &lenRecv, bufRecv);

//            SERIAL_PORT_MONITOR.print(id);
//            SERIAL_PORT_MONITOR.print(",");
            myFile.print(id);
            myFile.print(",");

            for (int i = 0; i < lenRecv; i++) {
//                SERIAL_PORT_MONITOR.print(bufRecv[i]);
//                SERIAL_PORT_MONITOR.print(",");

                myFile.print(bufRecv[i]);
                myFile.print(",");
            }
//            SERIAL_PORT_MONITOR.println();
            myFile.println();
            if (id == ID_RTC_REQUEST) {
              second = bufRecv[0];
              minute = bufRecv[1];
              hour = bufRecv[2];
              date = bufRecv[3];
              month = bufRecv[4];
              year = 2000 + bufRecv[5];
              done_reading = true;
            }
        }

        myFile.close();
    }
  return done_reading;
}

/************************************** Interrupt Functions *******************************************/
void SensorRead() {
  if (state == STATE_READ_SENSOR) {
    read_request = true;
  } 
}

void SDInOut() {
  if ((state == STATE_SLEEP) && (ToggleRead() == HIGH)) {
    init_request = true;
  }  
  if ((state == STATE_READ_SENSOR) && (ToggleRead() == LOW)) {
    sleep_request = true;
  }
}

void MCP2515_ISR() {
//    SERIAL_PORT_MONITOR.println("Interrupt!");
    flag_CAN_recv = 1;
}

/************************************** Custom Functions ******************************************/
bool SDCardEmpty() {
  if (!SD.exists(FILE_NAME)) {
    return true;
  } else {
    return false;
  }
}

void selectMuxPin(byte pin)
{
  if (pin > MAX_NUM - 1) return; // Exit if pin is out of scope
  for (int i=0; i<n_selectPins; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}

/**************************************** Test Functions *****************************************/
void SD_bufferRead() {
  SERIAL_PORT_MONITOR.println(SD_buffer);
}

void ERROR_bufferRead() {
  SERIAL_PORT_MONITOR.println(SD_buffer);
}

void next_stateRead() {
  SERIAL_PORT_MONITOR.println(next_state);
}
