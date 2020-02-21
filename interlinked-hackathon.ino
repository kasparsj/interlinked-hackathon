#include <Sodaq_N2X.h>
#include <Sodaq_wdt.h>
#include <Sodaq_LSM303AGR.h>
//#include <Sodaq_UBlox_GPS.h>
#include <Wire.h>
//#include "TM1637.h"

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
#define STARTUP_DELAY 5000

String deviceId = "6ZDQgGhvGCglVObEOWKXgZOa";
String token = "maker:4UpOc1MiQcLxW1VeVzlWw9J6szQFAALCaCrbSeH";

#define DISPLAY_CLK A0  //pins definitions for TM1637 and can be changed to other ports
#define DISPLAY_DIO A1
#define GAS_PIN A7

//TM1637 tm1637(DISPLAY_CLK, DISPLAY_DIO);
Sodaq_N2X n2x;
Sodaq_LSM303AGR AccMeter;
Sodaq_SARA_N211_OnOff saraR4xxOnOff;
int numSendFailed = 0;

bool isConnected = false;
int8_t temperature = 0;
unsigned char hr = 0;
int gasValue = 0;
double gpsValue[2] = {0, 0};
double accValue[3] = {0, 0, 0};
double magValue[3] = {0, 0, 0};
double accCompensation[3] = {0, 0, 0};
double magCompensation[3] = {0, 0, 0};
double velocity[3] = {0, 0, 0};
double position[3] = {0, 0, 0};
double old_R[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
double current_R[3][3] = {{0}};
unsigned long int updateCounter = 0;
void sendReadings(int socketID);

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
    setupAccMeter();

    //tm1637.init();
    // BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
    //tm1637.set(BRIGHT_TYPICAL);

    DEBUG_STREAM.begin(DEBUG_STREAM_BAUD);
    MODEM_STREAM.begin(n2x.getDefaultBaudrate());

    DEBUG_STREAM.println("Initializing and connecting... ");

    //sodaq_gps.init(GPS_ENABLE);
    //sodaq_gps.setDiag(DEBUG_STREAM);

    n2x.setDiag(DEBUG_STREAM);
    n2x.init(&saraR4xxOnOff, MODEM_STREAM);
    if (n2x.on()) {
       DEBUG_STREAM.println("turning modem on");
    }
    
    tryConnect();
}

void setupAccMeter()
{
  DEBUG_STREAM.println("setup accmeter");
  
  AccMeter.rebootAccelerometer();
  delay(1000);

  AccMeter.enableAccelerometer(Sodaq_LSM303AGR::LowPowerMode, Sodaq_LSM303AGR::HrNormalLowPower200Hz);
  AccMeter.enableMagnetometer(Sodaq_LSM303AGR::MagLowPowerMode, Sodaq_LSM303AGR::Hz10, Sodaq_LSM303AGR::Continuous);

  long accSums[3] = {0, 0, 0};
  long magSums[3] = {0, 0, 0};
  int numSamples = 1000;
  for(int i=0; i<numSamples; i++)
  {
    accSums[0] += AccMeter.getX();
    accSums[1] += AccMeter.getY();
    accSums[2] += AccMeter.getZ();
    if (i % 20 == 0) {
      magSums[0] += AccMeter.getMagX();
      magSums[1] += AccMeter.getMagY();
      magSums[2] += AccMeter.getMagZ();
    }
    delay(5);
  }
  accCompensation[0] = accSums[0] / numSamples;
  accCompensation[1] = accSums[1] / numSamples;
  accCompensation[2] = accSums[2] / numSamples;
  magCompensation[0] = magSums[0] / (numSamples / 20);
  magCompensation[1] = magSums[1] / (numSamples / 20);
  magCompensation[2] = magSums[2] / (numSamples / 20);
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
  else {
    DEBUG_STREAM.println("Connected!!! Yay!");
    isConnected = true;
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
  updateReadings(false);
  if (updateCounter % 50 == 0) {
    if (!isConnected || !n2x.isConnected() || numSendFailed > 3) {
      tryConnect();
      numSendFailed = 0;
    }
    int socketID = createSocket();
    sendReadings(socketID);
    closeSocket(socketID);
  }
  updateCounter++;

  //sodaq_wdt_safe_delay(5000);
  //sodaq_wdt_safe_delay(100);
  delay(100);
}

void updateReadings(bool updateGps)
{
  DEBUG_STREAM.println("Updating readings...");
  temperature = AccMeter.getTemperature();
  Wire.requestFrom(0xA0 >> 1, 1);
  if (Wire.available()) {
    hr = Wire.read();
  }
//  if (updateGps && sodaq_gps.scan()) {
//    gpsValue[0] = sodaq_gps.getLat();
//    gpsValue[1] = sodaq_gps.getLon();
//  }
  double newAccValue[3] = {
    AccMeter.getX() - accCompensation[0],
    AccMeter.getY() - accCompensation[1],
    AccMeter.getZ() - accCompensation[2]
  };
  float accWin = 0.25f;
  float accWinZ = 1.1f;
  if (newAccValue[0] < accWin) { newAccValue[0] = 0; }
  if (newAccValue[1] < accWin) { newAccValue[1] = 0; }
  if (newAccValue[2] < accWinZ) { newAccValue[2] = 0; }
  double newVelocity[3] = {
    velocity[0] + accValue[0] + ((newAccValue[0] - accValue[0]) / 2),
    velocity[1] + accValue[1] + ((newAccValue[1] - accValue[1]) / 2),
    velocity[2] + accValue[2] + ((newAccValue[2] - accValue[2]) / 2)
  };
  double newMagValue[3] = {
    AccMeter.getMagX() - magCompensation[0],
    AccMeter.getMagY() - magCompensation[1],
    AccMeter.getMagZ() - magCompensation[2]
  };
  float magWin = 10.f;
  if (newMagValue[0] < magWin) { newMagValue[0] = 0; }
  if (newMagValue[1] < magWin) { newMagValue[1] = 0; }
  if (newMagValue[2] < magWin) { newMagValue[2] = 0; }
  //if (newMagValue[0] != 0 || newMagValue[1] != 0 || newMagValue[2] != 0) {
  //  updateRotationMatrix(newMagValue);
  //}
  position[0] = position[0] + velocity[0] + ((newVelocity[0] - velocity[0]) / 2);
  position[1] = position[1] + velocity[1] + ((newVelocity[1] - velocity[1]) / 2);
  position[2] = position[2] + velocity[2] + ((newVelocity[2] - velocity[2]) / 2);
  velocity[0] = newVelocity[0];
  velocity[1] = newVelocity[1];
  velocity[2] = newVelocity[2];
  accValue[0] = newAccValue[0];
  accValue[1] = newAccValue[1];
  accValue[2] = newAccValue[2];
  magValue[0] = newMagValue[0];
  magValue[1] = newMagValue[1];
  magValue[2] = newMagValue[2];
  gasValue = analogRead(GAS_PIN);
}

void sendReadings(int socketID)
{   
  DEBUG_STREAM.println("Sending readings...");

  String value = "{";
  value += "\"t\":{\"value\":" + String(temperature) +"}";
  //value += ", \"gps\":{\"lat\":" + String(gpsValue[0]) +", \"long\": " + String(gpsValue[1]) + "}";
  value += ", \"hr\":{\"value\":" + String(hr) + "}";
  value += ", \"acc\":{\"value\":{\"x\":" + String(accValue[0]) +", \"y\": " + String(accValue[1]) + ", \"z\": " + String(accValue[2]) + "}}";
  //value += ", \"vel\":{\"value\":{\"x\":" + String(velocity[0]) +", \"y\": " + String(velocity[1]) + ", \"z\": " + String(velocity[2]) + "}}";
  //value += ", \"pos\":{\"value\":{\"x\":" + String(position[0]) +", \"y\": " + String(position[1]) + ", \"z\": " + String(position[2]) + "}}";
  value += ", \"mag\":{\"value\":{\"x\":" + String(magValue[0]) +", \"y\": " + String(magValue[1]) + ", \"z\": " + String(magValue[2]) + "}}";
  value += ", \"gas\":{\"value\":" + String(gasValue) +"}";
  value += "}";
  DEBUG_STREAM.println(value);

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

void UpdateRotationMatrix(double angle[3]){ // Update R as each new sample becomes available
    double sum; //summation variable used in matrix multiplication
    int i, j, k; //counters

    //attitude update matrix is an elementary rotation matrix about z
    double attitude_update[3][3] = {{cos(angle[0]),-sin(angle[0]),0},
                                    {sin(angle[0]),cos(angle[0]),0},
                                    {0,0,1}};

    //Compute: Current Rotation Matrix = Old Rotation Matrix * Attitude Update Matrix
    //(multiplication of two 3x3 matrices)
    for (i=0; i<3; i++) {
        for (j=0; j<3; j++) {
            sum=0;
            for (k=0; k<3; k++) {
                sum = sum + old_R[i][k]*attitude_update[k][j];
                current_R[i][j] = sum;
            } 
        }  
    }
    
    //Send Current Rotation Matrix to Previous Rotation Matrix: old_R = current_R
    for (i=0; i<3; i++) {
        for (j=0; j<3; j++) {
            old_R[i][j] = current_R[i][j]; 
        }  
    }
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
