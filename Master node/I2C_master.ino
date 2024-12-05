#include "WiFi.h"
#include "CRC.h"
#include "Common.h"
#include "I2C.h"
#include "TCP.h"


void IRAM_ATTR onPromptTimer(){
  xSemaphoreGiveFromISR(promptSemaphore, NULL);
}

void IRAM_ATTR onreadingTimer(){
  xSemaphoreGiveFromISR(readingSemaphore, NULL);
}

void Timers_Setup() {
  // For prompt timer
  promptSemaphore = xSemaphoreCreateBinary();
  promptTimer = timerBegin(0, 80, true);  // 1 MHz
  timerAttachInterrupt(promptTimer, &onPromptTimer, true);
  timerAlarmWrite(promptTimer, promptInterval, true);
  timerAlarmEnable(promptTimer);

  // For reading timer
  readingSemaphore = xSemaphoreCreateBinary();
  readingTimer = timerBegin(1, 80, true);  // 1 MHz
  timerAttachInterrupt(readingTimer, &onreadingTimer, true);
  timerStop(readingTimer);
}

void startReading() {
  Serial.println("Start reading!");
  I2C_Announce("Start");
  isReading = true;
  frameTime = 1000000 / targetFps;
  timerAlarmWrite(readingTimer, frameTime, true);
  timerAlarmEnable(readingTimer);
  timerStart(readingTimer);
}

//***********************************************//

void stopReading() {
  Serial.println("Stop reading!");
  I2C_Announce("Stop");
  isReading = false;
  timerStop(readingTimer);
}

void forceStop(char* msg) {
  stopReading();
  sendToClient(theClient, msg, strlen(msg));
}

void softReset() {
  ESP.restart();
}

void sendNodeData(uint8_t nodeData[][packetSize]) {
  uint8_t data[totalDevices*packetSize];
  for (int i=0; i<totalDevices; i++) {
    for (int j=0; j<packetSize; j++) {
      data[i*packetSize+j] = nodeData[i][j];
    }
  }
  sendToClient(theClient, (char*)data, totalDevices*packetSize);
}

void sendInfo() {
  char msg[100];
  sprintf(msg, "Devices: %i [", totalDevices);
  for (uint8_t i=0; i<totalDevices; i++) {
    char _str[5];
    sprintf(_str, " %i", devices[i]);
    strcat(msg, _str);
  }
  strcat(msg, " ]");
  sendToClient(theClient, msg, strlen(msg));
}

//***********************************************//

void WiFi_Setup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  Timers_Setup();
  WiFi_Setup();
  I2C_Setup();
  I2C_Scan();
  TCPServer_Setup(); // Start TCP server
}

void loop() {
  if (xSemaphoreTake(readingSemaphore, 0) == pdTRUE) {  // reading timer fired
    uint8_t nodeData[totalDevices][packetSize];
    for (uint8_t i=0; i < totalDevices; i++) {
      if (I2C_GetData(devices[i], nodeData[i])) {
        nodeData[i][packetSize-1] = devices[i];  //  store node i2c address in last byte
      }
      else {
        char msg[50];
        sprintf(msg, "Node %i lost connection! Abort!", devices[i]);
        Serial.println(msg);
        forceStop(msg);
        break;
      }
    }
    sendNodeData(nodeData);
    okPacket++;
  }
  if (xSemaphoreTake(promptSemaphore, 0) == pdTRUE) {  // prompt timer fired
    if (isReading) {
      Serial.printf("Packets in %ds: %d\n", promptInterval/1000000, okPacket);
      okPacket = 0;
    }
    else {
      Serial.println("Waiting for command...");
    }
  }
  if (hasNewTcpCommand) {
    hasNewTcpCommand = false;
    switch (TCPcommand) {
      case start:
        startReading();
        break;
      case stop:
        stopReading();
        break;
      case getinfo:
        sendInfo();
        break;
      case reset:
        softReset();
        break;
      case rescan:
        I2C_ForceRescan();
        break;
      default:
        break;
    }
  }
}