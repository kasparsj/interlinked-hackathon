#include <Sodaq_N2X.h>
#include <Sodaq_wdt.h>
#include <Sodaq_LSM303AGR.h>
#include <Sodaq_UBlox_GPS.h>
#include <Wire.h>
#include "TM1637.h"

#if defined(ARDUINO_SODAQ_AUTONOMO)
/* SODAQ AUTONOMO + SODAQ NB-IoT R41XM Bee */
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial1
#define powerPin BEE_VCC
#define enablePin BEEDTR

#elif defined(ARDUINO_SODAQ_SARA)
/* SODAQ SARA AFF*/
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial1
#define powerPin SARA_ENABLE
#define enablePin SARA_TX_ENABLE
#define MODEM_ON_OFF_PIN SARA_ENABLE

#elif defined(ARDUINO_SODAQ_SFF)
/* SODAQ SARA SFF*/
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial
#define powerPin SARA_ENABLE
#define enablePin SARA_TX_ENABLE

#else
#error "Please use one of the listed boards or add your board."
#endif

#define DEBUG_STREAM SerialUSB
#define DEBUG_STREAM_BAUD 115200
#define STARTUP_DELAY 3000

String deviceId = "6ZDQgGhvGCglVObEOWKXgZOa";
String token = "maker:4UpOc1MiQcLxW1VeVzlWw9J6szQFAALCaCrbSeH";

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
// 4 digit 7 SEGMENT DISPLAY I2C

#define DISPLAY_CLK A0  //pins definitions for TM1637 and can be changed to other ports
#define DISPLAY_DIO A1
#define GAS_PIN A4

TM1637 tm1637(DISPLAY_CLK, DISPLAY_DIO);
Sodaq_N2X n2x;
Sodaq_LSM303AGR AccMeter;
Sodaq_SARA_N211_OnOff saraR4xxOnOff;
int numSendFailed = 0;

void sendReadings(int socketID, bool sendTemp = true, bool sendGps = true, bool sendHR = true, bool sendAcc = true, bool sendMag = true, bool sendGas = true);

void setup()
{
    sodaq_wdt_safe_delay(STARTUP_DELAY);

    #ifdef powerPin
    // Turn the nb-iot module on
    pinMode(powerPin, OUTPUT); 
    digitalWrite(powerPin, HIGH);
    //DEBUG_STREAM.println("powerPin!");
    #endif
  
    #ifdef enablePin
    // Set state to active
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);
    //DEBUG_STREAM.println("enablePin!");
    #endif // enablePin

  
    Wire.begin();
    AccMeter.rebootAccelerometer();
    delay(1000);

    AccMeter.enableAccelerometer();
    AccMeter.enableMagnetometer(Sodaq_LSM303AGR::MagLowPowerMode, Sodaq_LSM303AGR::Hz10, Sodaq_LSM303AGR::Continuous);

    tm1637.init();
    // BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
    tm1637.set(BRIGHT_TYPICAL);

    DEBUG_STREAM.begin(DEBUG_STREAM_BAUD);
    MODEM_STREAM.begin(n2x.getDefaultBaudrate());

    DEBUG_STREAM.println("Initializing and connecting... ");

    sodaq_gps.init(GPS_ENABLE);
    //sodaq_gps.setDiag(DEBUG_STREAM);

    n2x.setDiag(DEBUG_STREAM);
    n2x.init(&saraR4xxOnOff, MODEM_STREAM);
    if (n2x.on()) {
       DEBUG_STREAM.println("turning modem on");
    }
    
    tryConnect();
}

void tryConnect() {
  const char* apn = "nb-iot.lmt.lv";
  const char* forceOperator = "24701"; // optional - depends on SIM / network (network Local) Country 
  uint8_t urat = 20;
  if (n2x.isConnected()) {
    n2x.disconnect();
  }
  if (!n2x.connect(apn, "0", forceOperator, urat)) {
     DEBUG_STREAM.println("FAILED TO CONNECT TO MODEM");
  }
}

int createSocket() {
  int localPort = 16666;
  int socketID = n2x.socketCreate(localPort);
  if (socketID >= 7 || socketID < 0) {
      DEBUG_STREAM.println("Failed to create socket");
  }
  else {
    DEBUG_STREAM.println("Created socket!");
  }
  return socketID;
}

void closeSocket(int socketID) {
  n2x.socketClose(socketID);
}

void loop()
{
  if (!n2x.isConnected() || numSendFailed > 3) {
    tryConnect();
    numSendFailed = 0;
  }
  int socketID = createSocket();
  sendReadings(socketID, true, false);  
  closeSocket(socketID);
  
  //sodaq_wdt_safe_delay(1000);
  delay(1000);
} 

void sendReadings(int socketID, bool sendTemp, bool sendGps, bool sendHR, bool sendAcc, bool sendMag, bool sendGas)
{   
  DEBUG_STREAM.println("Sending readings...");

  String value = "{";
  if (sendTemp) {
    int8_t temp = AccMeter.getTemperature();
    value += "\"t\":{\"value\":" + String(temp) +"}";
    DEBUG_STREAM.print("Temperature is:" );
    DEBUG_STREAM.println(temp);
  }

  if (sendGps) {
    if (sodaq_gps.scan())
    {
      if (value.length() > 1) { 
        value += ", ";
      }
      value += "\"gps\":{\"lat\":" + String(sodaq_gps.getLat(), 7) +", \"long\": " + String(sodaq_gps.getLon(), 7) + "}";  
      DEBUG_STREAM.print("Lat is:" );
      DEBUG_STREAM.println(sodaq_gps.getLat());
      DEBUG_STREAM.print("Lon is:" );
      DEBUG_STREAM.println(sodaq_gps.getLon());
    }
    else {
      DEBUG_STREAM.println("Failed to scan GPS");
    }
  }

  if (sendHR) {
    Wire.requestFrom(0xA0 >> 1, 1);
    if (Wire.available()) {
      unsigned char hr = Wire.read();
      if (value.length() > 1) { 
        value += ", ";
      }
      value += "\"hr\":{\"value\":" + String(hr) +"}";
      DEBUG_STREAM.print("HR is:");
      DEBUG_STREAM.println(hr);
    }
    else {
      DEBUG_STREAM.println("Failed to read HR");
    }
  }

  if (sendAcc) {
    if (value.length() > 1) {
      value += ", ";
    }
    value += "\"acc\": {\"value\":{\"x\":" + String(AccMeter.getX()) +", \"y\": " + String(AccMeter.getY()) + ", \"z\": " + String(AccMeter.getZ()) + "}}";
    DEBUG_STREAM.print("Accelerometer is:" );
    DEBUG_STREAM.println(String(AccMeter.getX()) + ", " + String(AccMeter.getY()) + ", " + String(AccMeter.getZ()));
  }

  if (sendMag) {
    if (value.length() > 1) { 
      value += ", ";
    }
    value += "\"mag\":{\"value\":{\"x\":" + String(AccMeter.getMagX()) +", \"y\": " + String(AccMeter.getMagY()) + ", \"z\": " + String(AccMeter.getMagZ()) + "}}";
    DEBUG_STREAM.print("Magnetometer is:" );
    DEBUG_STREAM.println(String(AccMeter.getMagX()) + ", " + String(AccMeter.getMagY()) + ", " + String(AccMeter.getMagZ()));
  }

  if (sendGas) {
    if (value.length() > 1) { 
      value += ", ";
    }
    int gasValue = analogRead(GAS_PIN);
    value += "\"gas\":{\"value\":" + String(gasValue) +"}";
    DEBUG_STREAM.print("Gas is:" );
    DEBUG_STREAM.println(String(gasValue));
  }
  
  value += "}";

  String reading = deviceId + '\n' + token + '\n' + value;

  uint8_t size = reading.length();
  int lengthSent = n2x.socketSend(socketID, "40.68.172.187", 8891, (uint8_t*)reading.c_str(), size);

  if (size <= lengthSent) {
    numSendFailed = 0;
    DEBUG_STREAM.println("socketSend success!");
  }
  else {
    numSendFailed++;
    DEBUG_STREAM.print("Length buffer vs sent:");
    DEBUG_STREAM.print(size);
    DEBUG_STREAM.print(",");
    DEBUG_STREAM.println(lengthSent);
    DEBUG_STREAM.println();
  }
  
  DEBUG_STREAM.println();
}

void display()
{
  //0~9,A,b,C,d,E,F
  int8_t NumTab[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  int8_t ListDisp[4];
  unsigned char i = 0;
  unsigned char count = 0;
  delay(150);
//  while(1)
//  {
//    i = count;
//    count ++;
//    if(count == sizeof(NumTab)) count = 0;
//    for(unsigned char BitSelect = 0;BitSelect < 4;BitSelect ++)
//    {
//      ListDisp[BitSelect] = NumTab[i];
//      i ++;
//      if(i == sizeof(NumTab)) i = 0;
//    }
//    tm1637.display(0,ListDisp[0]);
//    tm1637.display(1,ListDisp[1]);
//    tm1637.display(2,ListDisp[2]);
//    tm1637.display(3,ListDisp[3]);
//    delay(300);
//  }
}
