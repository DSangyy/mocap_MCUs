#include <Wire.h>
#include <CRC.h>
#include <EEPROM.h>
#include <SPI.h>
#include "KZ_filter.h"
#include "SparkFun_BNO080_Arduino_Library.h"

byte msg[16] = {0};

const uint8_t cmdlen = 10;
char* response;

const uint8_t targetFps = 50;  // fixed at 50 sample/s

bool isReading = false;
bool isReady = false;
bool isQuery = false;  // true will respond last command, false will respond imu data

const byte imuCSPin = 10;
const byte imuWAKPin = 4;
const byte imuINTPin = 2;
const byte imuRSTPin = 3;

uint8_t samplecount = 0;
unsigned long startTime = 0;

KZ_buf KZ_buf;

BNO080 myIMU;

void reverseMemcpy(byte* des, byte* source, uint8_t len) {
  for (uint8_t i=0; i<len; i++) {
    *(des+i) = *(source+len-i-1);
  }
}

void reading2msg(byte* msg, float a, float b, float c, float d) {
  reverseMemcpy(msg, (byte*)&a, 4);
  reverseMemcpy(msg+4, (byte*)&b, 4);
  reverseMemcpy(msg+8, (byte*)&c, 4);
  reverseMemcpy(msg+12, (byte*)&d, 4);
}

void IMU_Setup() {
  while (!myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin))
  {
    Serial.println(F("BNO080 over SPI not detected. Waiting..."));
    delay(10);
  }
  myIMU.enableRotationVector(1000000/targetFps);  // us
  Serial.println(F("Rotation vector enabled"));
  myIMU.modeSleep();  // put sensor to sleep
  isReady = true;
}

void printAccuracyLevel(byte accuracyNumber)
{
  if (accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if (accuracyNumber == 1) Serial.print(F("Low"));
  else if (accuracyNumber == 2) Serial.print(F("Medium"));
  else if (accuracyNumber == 3) Serial.print(F("High"));
}

void IMU_Reset() {
  myIMU.softReset();
}

void IMU_Calibrate() {
  while (!myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin))
  {
    Serial.println(F("BNO080 over SPI not detected. Waiting..."));
    delay(10);
  }
  //Turn on cal for Accel, Gyro, and Mag
  myIMU.calibrateAll(); 
  //Send data update every 100ms
  myIMU.enableRotationVector(100000);
  myIMU.enableMagnetometer(100000);
  Serial.println(F("Calibrating. Press 's' to save to flash"));
  Serial.println(F("Output in form x, y, z, in uTesla"));
  while (1) {
    delay(10);
    if(Serial.available())
    {
      byte incoming = Serial.read();
      if(incoming == 's')
      {
        myIMU.saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
        myIMU.requestCalibrationStatus(); //Sends command to get the latest calibration status
        //Wait for calibration response, timeout if no response
        int counter = 100;
        while(1 && counter > 0)
        {
          if(myIMU.dataAvailable() == true)
          {
            //The IMU can report many different things. We must wait
            //for the ME Calibration Response Status byte to go to zero
            if(myIMU.calibrationComplete() == true)
            {
              Serial.println(F("Calibration data successfully stored"));
              delay(1000);
              IMU_Reset();
              return;
            }
          }
          delay(1);
          counter--;
          if(counter == 0) {
            Serial.println(F("Calibration data failed to store. Please try again."));
            delay(1000);
            IMU_Reset();
            return;
          }
        }
      }
    }

    //Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {
      float x = myIMU.getMagX();
      float y = myIMU.getMagY();
      float z = myIMU.getMagZ();
      byte accuracy = myIMU.getMagAccuracy();

      float quatI = myIMU.getQuatI();
      float quatJ = myIMU.getQuatJ();
      float quatK = myIMU.getQuatK();
      float quatReal = myIMU.getQuatReal();
      byte sensorAccuracy = myIMU.getQuatAccuracy();

      Serial.print(x, 2);
      Serial.print(F(","));
      Serial.print(y, 2);
      Serial.print(F(","));
      Serial.print(z, 2);
      Serial.print(F(","));
      printAccuracyLevel(accuracy);
      Serial.print(F(","));

      Serial.print("\t");

      Serial.print(quatI, 2);
      Serial.print(F(","));
      Serial.print(quatJ, 2);
      Serial.print(F(","));
      Serial.print(quatK, 2);
      Serial.print(F(","));
      Serial.print(quatReal, 2);
      Serial.print(F(","));
      printAccuracyLevel(sensorAccuracy);
      Serial.print(F(","));

      Serial.println();
    }
  }
}

void startReading() {
  Serial.println(F("Start reading sensor!"));
  myIMU.modeOn();
  startTime = millis();
  isReading = true;
}

void stopReading() {
  Serial.println(F("Stop reading sensor!"));
  isReading = false;
  myIMU.modeSleep();
}

void I2C_OnRequest() {
  if (isQuery) {
    isQuery = false;
    Wire.write(response, strlen(response)+1);
  }
  else {
    Wire.write(msg, 16);
    Wire.write(calcCRC8(msg, 16));
    Wire.flush();
  }

}

void I2C_OnReceive(int arg) {
  Serial.print(F("Receiving command"));
  char cmd[cmdlen+1];
  uint8_t i = 0;
  while (Wire.available() && i < cmdlen) {
    cmd[i] = Wire.read();
    i++;
    Serial.print(F("........"));
  }
  cmd[i] = '\0';
  Serial.println(cmd);

  if (strcmp(cmd, "IsReady") == 0) {
    isQuery = true;
    response = isReady ? "Ready" : "NotReady";
  }

  if (strcmp(cmd, "Start") == 0) {
    if (!isReading) startReading();
  }

  if (strcmp(cmd, "Stop") == 0) {
    if (isReading) stopReading();
  }
}

void I2C_Setup() {
  uint8_t addr = EEPROM.read(0);
  if (addr < 1 || addr > 127) {
    Serial.println(F("I2C address not set!"));
    Serial.println(F("Enter address (1-127): "));
    while (1) {
      // Wait for input
      while (!Serial.available()) {
        delay(10);
      }
      int new_addr = atoi(Serial.readStringUntil('\n').c_str());
      if (new_addr > 127 || new_addr == 0) {
        Serial.println(F("Wrong address, enter again:"));
      } else {
        addr = new_addr;
        break;
      }
    }
    EEPROM.write(0, addr);
    Serial.print(F("I2C address set as "));
    Serial.println(addr);
  }
  Wire.begin(addr);
  Wire.onRequest(&I2C_OnRequest);
  Wire.onReceive(&I2C_OnReceive);
  Serial.print(F("Started I2C with address "));
  Serial.println(addr);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Enter any key in 1s to start calibrate!"));
  unsigned long startDelay = millis();
  while (1) {
    if (millis() - startDelay > 1000) break;
    if (Serial.available()) {
      Serial.read();
      IMU_Calibrate();
      break;
    }
  }
  I2C_Setup();
  IMU_Setup();
}

void loop() {
  if (isReading) {
    if (myIMU.dataAvailable() == true)
    {
      Quat q = {myIMU.getQuatReal(), myIMU.getQuatI(), myIMU.getQuatJ(), myIMU.getQuatK()};
      Quat r = Quat();
      if (cal_3_5(q, KZ_buf, r)) {
        reading2msg(msg, r.w, r.x, r.y, r.z);
        
      }
      else {
        reading2msg(msg, q.w, q.x, q.y, q.z);
      }
      samplecount++;
      delay(10);
    }
    if (millis() - startTime > 1000) {
      Serial.print("Sample per 1s: ");
      Serial.println(samplecount);
      samplecount = 0;
      startTime = millis();
    }
  }
}
