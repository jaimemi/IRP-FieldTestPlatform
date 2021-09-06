#include "arduino_secrets.h"
// WiFiNINA - Version: Latest
#include <WiFiNINA.h>

/*
  Sketch generated by the Arduino IoT Cloud Thing "OnlineLab"
  https://create.arduino.cc/cloud/things/c9d4b724-e406-4d7d-9964-070eae54f9ac

  Arduino IoT Cloud Variables description

  The following variables are automatically generated and updated when changes are made to the Thing

  float temp_pv_k_rtf;
  float temp_pv_int_rtf;
  float temp_pv_int_ltf;
  float temp_pv_k_ltf;
  float temp_pv_int_rbf;
  float temp_pv_k_rbf;
  float temp_pv_int_lbf;
  float temp_pv_int_rtnf;
  float temp_pv_k_rtnf;
  float temp_pv_int_ltnf;
  float temp_pv_k_ltnf;
  float temp_pv_int_rbnf;
  float temp_pv_k_rbnf;
  float temp_pv_int_lbnf;
  float temp_pv_k_lbnf;
  int i_pv_f;
  int i_pv_nf;
  int p_pv_nf;
  int v_pv_nf;
  int p_pv_f;
  int v_pv_f;
  int alt_ws;
  float temp_ws;
  int pres_ws;
  int counts_ws_415nm_F1;
  int counts_ws_445nm_F2;
  int counts_ws_480nm_F3;
  int counts_ws_515nm_F4;
  int counts_ws_555nm_F5;
  int counts_ws_590nm_F6;
  int counts_ws_630nm_F7;
  int counts_ws_680nm_F8;
  int counts_ws_CLEAR;
  int counts_ws_NIR;
  float temp_pv_k_lbf;
  int counts_ec_415nm_F1;
  int counts_ec_445nm_F2;
  int counts_ec_480nm_F3;
  int counts_ec_515nm_F4;
  int counts_ec_555nm_F5;
  int counts_ec_590nm_F6;
  int counts_ec_630nm_F7;
  int counts_ec_680nm_F8;
  int counts_ec_CLEAR;
  int counts_ec_NIR;
  int p_ec;
  int v_ec;
  float temp_ec_int_1;
  float temp_ec_int_2;
  float temp_ec_int_3;
  float temp_ec_k_1;
  float temp_ec_k_2;
  float temp_ec_k_3;
  int i_ec;

  Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.
*/

#include "thingProperties.h"
#include <CAN.h>

/*********************************** Variables definition *****************************************/
// RTC Request Objets
RTCZero rtc_request;
bool rtc_request_flag = false;

// Alarm LED
#define LED_CS LED_BUILTIN

/*************************************** IDs ******************************************************/
// Platform ID
#define ID_WIFI_PLATFORM            0x001000
#define ID_WS_PLATFORM              0x000100
#define ID_PV_PLATFORM              0x000200
#define ID_EC_PLATFORM              0x000300

#define ID_MY_PLATFORM ID_WIFI_PLATFORM

// WS Sensor IDs
#define ID_BMP280_WS                0x000000
#define ID_AS7341_WS                0x000010

// WS MEasurement IDs
#define ID_BMP280_T                 0x000000
#define ID_BMP280_P                 0x000001
#define ID_BMP280_ALT               0x000002

#define ID_AS7341_CHANNEL_415nm_F1  0x000000
#define ID_AS7341_CHANNEL_445nm_F2  0x000001
#define ID_AS7341_CHANNEL_480nm_F3  0x000002
#define ID_AS7341_CHANNEL_515nm_F4  0x000003
#define ID_AS7341_CHANNEL_555nm_F5  0x000004
#define ID_AS7341_CHANNEL_590nm_F6  0x000005
#define ID_AS7341_CHANNEL_630nm_F7  0x000006
#define ID_AS7341_CHANNEL_680nm_F8  0x000007
#define ID_AS7341_CHANNEL_CLEAR     0x000008
#define ID_AS7341_CHANNEL_NIR       0x000009

// PV Sensor IDs
#define ID_THERMOCOUPLE_PV_RTF      0x000000
#define ID_THERMOCOUPLE_PV_LTF      0x000010
#define ID_THERMOCOUPLE_PV_RBF      0x000020
#define ID_THERMOCOUPLE_PV_LBF      0x000030
#define ID_THERMOCOUPLE_PV_RTNF     0x000040
#define ID_THERMOCOUPLE_PV_LTNF     0x000050
#define ID_THERMOCOUPLE_PV_RBNF     0x000060
#define ID_THERMOCOUPLE_PV_LBNF     0x000070
#define ID_INA260_PV_F              0x000080
#define ID_INA260_PV_NF             0x000090

// PV Measurement IDs
#define ID_THERMOCOUPLE_INT         0x000000
#define ID_THERMOCOUPLE_K           0x000001
#define ID_INA260_I                 0x000000
#define ID_INA260_V                 0x000001
#define ID_INA260_P                 0x000002

// EC Sensor IDs
#define ID_THERMOCOUPLE_EC_1        0x000000
#define ID_THERMOCOUPLE_EC_2        0x000010
#define ID_THERMOCOUPLE_EC_3        0x000020
#define ID_INA260_EC                0x000080
#define ID_AS7341_EC                0x000090

// RTC request ID
#define ID_RTC_REQUEST              0xFF




/******************************************* Code **************************************************/
void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
  */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  CANInit();
  
  rtc_request.begin();
}

void loop() {
  ArduinoCloud.update();
  // Your code here
  
  if (rtc_request_flag) {
    rtc_request_flag = false;
    
    // Serial.println("Sending rtc...");
    CAN.beginPacket(ID_RTC_REQUEST);
    
    CAN.write(rtc_request.getSeconds());
    CAN.write(rtc_request.getMinutes());
    CAN.write(rtc_request.getHours());
    CAN.write(rtc_request.getDay());
    CAN.write(rtc_request.getMonth());
    CAN.write(rtc_request.getYear());
    
    CAN.endPacket();
  }
}

/***************************************** Init Functions *******************************************/
bool CANInit() {
  CAN.setPins(7,6);
  // start the CAN bus at 500 kbps
  if (!CAN.begin(125E3)) {
    Serial.println("Starting CAN failed!");
    while (1) (toggleLED());
  }
  
  CAN.onReceive(onReceiveCAN);
}

/**************************************** Custom Functions *****************************************/
void IoTCANUpdate(long id, String data) {
  uint16_t platform = id & 0xF00;
  uint16_t sensor = id & 0x0F0;
  uint16_t measurement = id & 0x00F;

  switch (platform) {
    case ID_PV_PLATFORM:
      {
        switch (sensor) {
          case ID_THERMOCOUPLE_PV_RTF:
            {
              switch (measurement) {
                case ID_THERMOCOUPLE_INT:
                  {
                    temp_pv_int_rtf = data.toFloat();
                  }
                  break;
                case ID_THERMOCOUPLE_K:
                  {
                    temp_pv_k_rtf = data.toFloat();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_THERMOCOUPLE_PV_LTF:
            {
              switch (measurement) {
                case ID_THERMOCOUPLE_INT:
                  {
                    temp_pv_int_ltf = data.toFloat();
                  }
                  break;
                case ID_THERMOCOUPLE_K:
                  {
                    temp_pv_k_ltf = data.toFloat();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_THERMOCOUPLE_PV_RBF:
            {
              switch (measurement) {
                case ID_THERMOCOUPLE_INT:
                  {
                    temp_pv_int_rbf = data.toFloat();
                  }
                  break;
                case ID_THERMOCOUPLE_K:
                  {
                    temp_pv_k_rbf = data.toFloat();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_THERMOCOUPLE_PV_LBF:
            {
              switch (measurement) {
                case ID_THERMOCOUPLE_INT:
                  {
                    temp_pv_int_lbf = data.toFloat();
                  }
                  break;
                case ID_THERMOCOUPLE_K:
                  {
                    temp_pv_k_lbf = data.toFloat();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_THERMOCOUPLE_PV_RTNF:
            {
              switch (measurement) {
                case ID_THERMOCOUPLE_INT:
                  {
                    temp_pv_int_rtnf = data.toFloat();
                  }
                  break;
                case ID_THERMOCOUPLE_K:
                  {
                    temp_pv_k_rtnf = data.toFloat();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_THERMOCOUPLE_PV_LTNF:
            {
              switch (measurement) {
                case ID_THERMOCOUPLE_INT:
                  {
                    temp_pv_int_ltnf = data.toFloat();
                  }
                  break;
                case ID_THERMOCOUPLE_K:
                  {
                    temp_pv_k_ltnf = data.toFloat();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_THERMOCOUPLE_PV_RBNF:
            {
              switch (measurement) {
                case ID_THERMOCOUPLE_INT:
                  {
                    temp_pv_int_rbnf = data.toFloat();
                  }
                  break;
                case ID_THERMOCOUPLE_K:
                  {
                    temp_pv_k_rbnf = data.toFloat();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_THERMOCOUPLE_PV_LBNF:
            {
              switch (measurement) {
                case ID_THERMOCOUPLE_INT:
                  {
                    temp_pv_int_lbnf = data.toFloat();
                  }
                  break;
                case ID_THERMOCOUPLE_K:
                  {
                    temp_pv_k_lbnf = data.toFloat();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_INA260_PV_F:
            {
              switch (measurement) {
                case ID_INA260_I:
                  {
                    i_pv_f = data.toInt();
                  }
                  break;
                case ID_INA260_V:
                  {
                    v_pv_f = data.toInt();
                  }
                  break;
                case ID_INA260_P:
                  {
                    p_pv_f = data.toInt();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          case ID_INA260_PV_NF:
            {
              switch (measurement) {
                case ID_INA260_I:
                  {
                    i_pv_nf = data.toInt();
                  }
                  break;
                case ID_INA260_V:
                  {
                    v_pv_nf = data.toInt();
                  }
                  break;
                case ID_INA260_P:
                  {
                    p_pv_nf = data.toInt();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          default:
            break;
        }
      }
      break;
      
    case ID_WS_PLATFORM:
    {
      switch (sensor) {
        case ID_BMP280_WS:
        {
          switch (measurement) {
            case ID_BMP280_T:
            {
              temp_ws = data.toFloat();
            }
            break;
            
            case ID_BMP280_P:
            {
              pres_ws = data.toInt();
            }
            break;
            
            case ID_BMP280_ALT:
            {
              alt_ws = data.toInt();
            }
            break;
            
            default:
            break;
          }
        }
        break;
      
        case ID_AS7341_WS:
        {
          switch (measurement) {
            case ID_AS7341_CHANNEL_415nm_F1:
            {
              counts_ws_415nm_F1 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_445nm_F2:
            {
              counts_ws_445nm_F2 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_480nm_F3:
            {
              counts_ws_480nm_F3 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_515nm_F4:
            {
              counts_ws_515nm_F4 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_555nm_F5:
            {
              counts_ws_555nm_F5 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_590nm_F6:
            {
              counts_ws_590nm_F6 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_630nm_F7:
            {
              counts_ws_630nm_F7 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_680nm_F8:
            {
              counts_ws_680nm_F8 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_CLEAR:
            {
              counts_ws_CLEAR = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_NIR:
            {
              counts_ws_NIR = data.toInt();
            }
            break;
            
            default:
            break;
          }
        }
        break;
          
        default:
        break;
      }
      
    }
    break;
    
    case ID_EC_PLATFORM:
    {
      switch (sensor) {
        
        case ID_THERMOCOUPLE_EC_1:
        {
          switch (measurement) {
            case ID_THERMOCOUPLE_INT:
            {
              temp_ec_int_1 = data.toFloat();
            }
            break;
            case ID_THERMOCOUPLE_K:
            {
              temp_ec_k_1 = data.toFloat();
            }
            break;
            default:
              break;
          }
        }
        break;
        case ID_THERMOCOUPLE_EC_2:
        {
          switch (measurement) {
            case ID_THERMOCOUPLE_INT:
            {
              temp_ec_int_2 = data.toFloat();
            }
            break;
            case ID_THERMOCOUPLE_K:
            {
              temp_ec_k_2 = data.toFloat();
            }
            break;
            default:
            break;
          }
        }
        break;
        case ID_THERMOCOUPLE_EC_3:
        {
          switch (measurement) {
            case ID_THERMOCOUPLE_INT:
            {
              temp_ec_int_3 = data.toFloat();
            }
            break;
            case ID_THERMOCOUPLE_K:
            {
              temp_ec_k_3 = data.toFloat();
            }
            break;
            default:
            break;
          }
        }
        break;
    
        case ID_AS7341_EC:
        {
          switch (measurement) {
            case ID_AS7341_CHANNEL_415nm_F1:
            {
              counts_ec_415nm_F1 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_445nm_F2:
            {
              counts_ec_445nm_F2 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_480nm_F3:
            {
              counts_ec_480nm_F3 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_515nm_F4:
            {
              counts_ec_515nm_F4 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_555nm_F5:
            {
              counts_ec_555nm_F5 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_590nm_F6:
            {
              counts_ec_590nm_F6 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_630nm_F7:
            {
              counts_ec_630nm_F7 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_680nm_F8:
            {
              counts_ec_680nm_F8 = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_CLEAR:
            {
              counts_ec_CLEAR = data.toInt();
            }
            break;
            
            case ID_AS7341_CHANNEL_NIR:
            {
              counts_ec_NIR = data.toInt();
            }
            break;
            
            default:
            break;
          }
        }
        break;
        
        case ID_INA260_EC:
            {
              switch (measurement) {
                case ID_INA260_I:
                  {
                    i_ec = data.toInt();
                  }
                  break;
                case ID_INA260_V:
                  {
                    v_ec = data.toInt();
                  }
                  break;
                case ID_INA260_P:
                  {
                    p_ec = data.toInt();
                  }
                  break;
                default:
                  break;
              }
            }
            break;
          
        default:
        break;
      }
      
    }
    break;
      
    default:
      break;
  }
}

void toggleLED() {
  digitalWrite(LED_CS, HIGH);
  delay(500);
  digitalWrite(LED_CS, LOW);
  delay(500);
}

/***************************************** Callback Functions *******************************************/
void onReceiveCAN(int packetSize) {
  String data = "";
  while (CAN.available()) {
    data.concat((char)CAN.read());
    // Serial.print((char)CAN.read());
  }
  
  Serial.print(CAN.packetId(), HEX);
  Serial.print(" : ");
  Serial.println(data);
  
  if (CAN.packetId() == ID_RTC_REQUEST) {
    rtc_request_flag = true;
  } else {
    IoTCANUpdate(CAN.packetId(), data);
  }
}





void onMKR1010Change() {
  // Do something
}


void onVocFChange() {
  // Do something
}


void onVocNfChange() {
  // Do something
}


void onIPvFChange() {
  // Do something
}


void onVPvFChange() {
  // Do something
}


void onVPvNfChange() {
  // Do something
}


void onIPvNfChange() {
  // Do something
}


void onPPvFChange() {
  // Do something
}


void onPPvNfChange() {
  // Do something
}



void onTempWsChange() {
  // Do something
}


void onPresWsChange() {
  // Do something
}


void onAltWsChange() {
  // Do something
}


void onCountsWs415nmF1Change() {
  // Do something
}


void onCountsWs445nmF2Change() {
  // Do something
}


void onCountsWs480nmF3Change() {
  // Do something
}


void onCountsWs515nmF4Change() {
  // Do something
}


void onCountsWs555nmF5Change() {
  // Do something
}


void onCountsWs590nmF6Change() {
  // Do something
}


void onCountsWs630nmF7Change() {
  // Do something
}


void onCountsWs680nmF8Change() {
  // Do something
}


void onCountsWsCLEARChange() {
  // Do something
}


void onCountsWsNIRChange() {
  // Do something
}








































void onCountsEc415nmF1Change() {
  // Do something
}


void onCountsEc445nmF2Change() {
  // Do something
}


void onCountsEc480nmF3Change() {
  // Do something
}


void onCountsEc515nmF4Change() {
  // Do something
}



void onCountsEc555nmF5Change() {
  // Do something
}


void onCountsEc590nmF6Change() {
  // Do something
}


void onCountsEc630nmF7Change() {
  // Do something
}


void onCountsEc680nmF8Change() {
  // Do something
}


void onCountsEcCLEARChange() {
  // Do something
}


void onCountsEcNIRChange() {
  // Do something
}


void onIEcChange() {
  // Do something
}



void onVEcChange() {
  // Do something
}


void onPEcChange() {
  // Do something
}


void onTempEcInt1Change() {
  // Do something
}


void onTempEcInt2Change() {
  // Do something
}


void onTempEcInt3Change() {
  // Do something
}


void onTempEcK1Change() {
  // Do something
}


void onTempEcK2Change() {
  // Do something
}


void onTempEcK3Change() {
  // Do something
}









