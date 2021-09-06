#include <RV-3028-C7.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AS7341.h>
#include <LiquidCrystal_I2C.h>
#include <CAN.h>

/***************************** Customizable variables ********************************************/
// Sampling time (s)
#define TIME_SAMP 10 // Seconds

//RCT set-up time
#define SECOND 0
#define MINUTE 15
#define HOUR 13
#define DAY 3 /* M=0|T=1|W=2|T=3|F=4|S=5|S=6 */
#define DATE 27
#define MONTH 5
#define YEAR 2021

// .CSV Name and Header
#define FILE_NAME "wslog.csv"
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

// Sensor objects
RV3028 rtc;
Adafruit_BMP280 bmp;
Adafruit_AS7341 as7341;

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

/*************************************      IDs          *****************************************/

// Error IDs
#define ERROR_ID_RV3028_INIT 11
#define ERROR_ID_SD_INIT 15
#define ERROR_ID_BMP280_INIT 16
#define ERROR_ID_AS7341_INIT 17
#define ERROR_ID_CAN_INIT 18
#define ERROR_ID_RV3028_WRITE 21
#define ERROR_ID_SD_WRITE 25

// Platform ID
#define ID_WIFI_PLATFORM          0x000100
#define ID_WS_PLATFORM            0x000200
#define ID_PV_PLATFORM            0x000300
#define ID_EC_PLATFORM            0x000400

#define MASK_PLATFORM             0x111100

// Sensor IDs
#define ID_THERMOCOUPLE_RTF       0x000000
#define ID_THERMOCOUPLE_LTF       0x000010
#define ID_THERMOCOUPLE_RBF       0x000020
#define ID_THERMOCOUPLE_LBF       0x000030
#define ID_THERMOCOUPLE_RTNF      0x000040
#define ID_THERMOCOUPLE_LTNF      0x000050
#define ID_THERMOCOUPLE_RBNF      0x000060
#define ID_THERMOCOUPLE_LBNF      0x000070
#define ID_INA260_F               0x000080
#define ID_INA260_NF              0x000090

// Measurement IDs
#define ID_THERMOCOUPLE_INT       0x000000
#define ID_THERMOCOUPLE_K         0x000001
#define ID_INA260_I               0x000000
#define ID_INA260_V               0x000001
#define ID_INA260_P               0x000002



/******************************************* Code **************************************************/
void setup() {

  state = STATE_SETUP;
  
  Wire.begin();

  // Initializing Serial
  Serial.begin(115200); // TEST
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Initializing RTC
  error_flag |= RV3028Init();
  
  // Initializing RTC Interrupt
  error_flag |= RTCINTInit();

  // Initializing BMP280
  error_flag |= BMP280Init();

  // Initializing AS7341;
  error_flag |= AS7341Init();

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
        
        if (SDCardEmpty()) {
          error_flag |= RV3028Write();
        }

        // CAN Initialization
        error_flag |= CANInit();
        
        if (!error_flag) {
          LCDWriteCANRead();
          while(!CANRead()) {}
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
          
          // Read BMP280
          BMP280Read();
  
          // Read AS7341
          AS7341Read();
  
          // Write data
          SD_bufferRead();
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

bool BMP280Init () {
  bool error = false;
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    ERROR_buffer += ERROR_ID_BMP280_INIT;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  } else {
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
  return error;
}

bool AS7341Init() {
  bool error = false;
  if (!as7341.begin()){
    ERROR_buffer += ERROR_ID_AS7341_INIT;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  } else {
    // TINT: 280ms | GAIN: 0.5 | MAX: 65,536 counts
    as7341.setATIME(100);
    as7341.setASTEP(999);
    as7341.setGain(AS7341_GAIN_0_5X);
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
  CAN.setPins(CAN_CS);
  if (!CAN.begin(500E3)) {
    ERROR_buffer += ERROR_ID_CAN_INIT;
    ERROR_buffer += ERROR_SEPARATOR;
    error = true;
  }
  return error;
}

/***************************************** Write Functions *******************************************/
bool RV3028Write() {
  bool error = false;
  if(!rtc.setTime(SECOND, MINUTE, HOUR, DAY, DATE, MONTH, YEAR)){
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

void LCDWriteCANRead() {
  lcd.setCursor(0,1);
  lcd.print("Asking for RT...");
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

bool BMP280Read() {
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(bmp.readTemperature());
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(bmp.readPressure());
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(bmp.readAltitude(1013.25));
}

bool AS7341Read() {
  as7341.readAllChannels();
  
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_415nm_F1));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_445nm_F2));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_480nm_F3));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_515nm_F4));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_555nm_F5));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_590nm_F6));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_630nm_F7));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_680nm_F8));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_CLEAR));
  SD_buffer += FILE_SEPARATOR;
  SD_buffer += String(as7341.getChannel(AS7341_CHANNEL_NIR));
}

bool CANRead() {
  bool done_reading = false;
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // received a packet
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        Serial.print((char)CAN.read());
      }
      Serial.println();
    }

    Serial.println();
    done_reading = true;
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

/************************************** Custom Functions ******************************************/
bool SDCardEmpty() {
  if (!SD.exists(FILE_NAME)) {
    return true;
  } else {
    return false;
  }
}


/**************************************** Test Functions *****************************************/
void SD_bufferRead() {
  Serial.println(SD_buffer);
}

void ERROR_bufferRead() {
  Serial.println(SD_buffer);
}

void next_stateRead() {
  Serial.println(next_state);
}
