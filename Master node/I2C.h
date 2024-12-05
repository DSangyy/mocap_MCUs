#include <stdint.h>
#include "Wire.h"
#include "Common.h"


uint8_t I2C_send(uint8_t id, char* msg) {
  Wire.beginTransmission(id);
  Wire.print(msg);
  return Wire.endTransmission();
}

void I2C_Announce(char* msg) {
  for (uint8_t i=0; i < totalDevices; i++) {
    I2C_send(devices[i], msg);
  }
}

void I2C_SlaveReset() {
  // Reset slaves
  pinMode(slaveReset, OUTPUT);
  digitalWrite(slaveReset, HIGH);
  delay(1);
  digitalWrite(slaveReset, LOW);
  delay(8000);
}

void I2C_Scan() {
  char msg[commandSize+1];
  for (uint8_t i=1; i<20; i++) {
    //Serial.printf("Trying address %i\n", i);
    uint8_t error = I2C_send(i, "IsReady");
    delay(100);
    //Serial.println(error);
    if (error == 0) {
      uint8_t bytesReceived = Wire.requestFrom(i, commandSize);
      if(bytesReceived) { //If received more than zero bytes  
        Wire.readBytes(msg, bytesReceived);
        msg[bytesReceived] = '\0';
        Serial.println(msg);
        if (strcmp(msg, "Ready") == 0) {
          Serial.printf("I2C device found at address %i, ready!\n", i);
          devices[totalDevices] = i;
          totalDevices++;
        } else {
          Serial.printf("I2C device found at address %i, not ready! Retrying...\n", i);
          delay(1000);
          i--;
        }
      }    
    }
    delay(10);
  }
  Serial.printf("I2C devices: %i\n", totalDevices);
}

void I2C_ForceRescan() {
  totalDevices = 0;
  memset(devices, 0, sizeof devices);
  I2C_SlaveReset();
  I2C_Scan();
}

void I2C_Setup() {
  I2C_SlaveReset();
  //Start I2C
  Wire.begin(21, 22, I2Cclock);
  Wire.setTimeout(I2Ctimeout);
}

bool I2C_GetData(uint8_t addr, uint8_t* data) {
  const uint8_t max_retry = 3;
  uint8_t retry = 0;
  while(retry <= max_retry) {
    uint8_t bytesReceived = Wire.requestFrom(addr, packetSize);
    //Serial.println(bytesReceived);
    if(bytesReceived) { //If received more than zero bytes  
      Wire.readBytes(data, bytesReceived);
      if (calcCRC8(data, packetSize-1) == data[packetSize-1]) {
        return true;
      }
    }
    retry++;
  }
  return false;
}