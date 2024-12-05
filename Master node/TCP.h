#include "AsyncTCP.h"
#include "Common.h"

AsyncServer* server;
AsyncClient *theClient;
const int TCPPort = 1234;

void parseCommand(char* msg, char cmd[][commandSize], uint8_t numCommand) {
  char* s = strtok(msg, commandDelim);
  Serial.println(s);
  strcpy(cmd[0], s);
  uint8_t i = 1;
  while (i < numCommand) {
    s = strtok(nullptr, commandDelim);
    if (s==NULL) break;
    strcpy(cmd[i], s);
    //Serial.println(s);
    i++;
  }
}

bool sendToClient(AsyncClient* client, char* data, uint32_t len, int timeout=1000) {
  const uint8_t bufferLen = 64;
  unsigned long startTime = 0;
  size_t sendBytes = 0;
  // Send the size of message in first 4 bytes
  startTime = micros();
  while (!(client->space() > 4 && client->canSend())) {
    // wait for buffer to be available, timeout by micros
    if (micros() - startTime > timeout) return false;
  }
  const char* s = reinterpret_cast<const char*>(&len);
  client->add(s, 4);
  if (!client->send()) return false;
  // Send the whole message in chunks
  while (len > sendBytes) {
    startTime = micros();
    while (!(client->space() > bufferLen && client->canSend())) {
      // wait for buffer to be available, timeout by micros
      if (micros() - startTime > timeout) return false;
    }
    sendBytes += client->add(data + sendBytes, len % bufferLen > 0 ? len % bufferLen : bufferLen);
		if (!client->send()) return false;
  }
  return true;
}

void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
	Serial.printf("\n data received from client %s \n", client->remoteIP().toString().c_str());
  char msg[len+1];
  memcpy(msg, (char*)data, len);
  msg[len] = 0;
  
  char cmd[2][commandSize];
  parseCommand(msg, cmd, 2);
  
  if (strcmp(cmd[0], "GetInfo") == 0) {
    TCPcommand = getinfo;
    hasNewTcpCommand = true;
  }

  if (strcmp(cmd[0], "StartRead") == 0) {
    if (strlen(cmd[1]) > 0) {
      targetFps = atoi(cmd[1]);
      TCPcommand = start;
      hasNewTcpCommand = true;
    } else {
      Serial.println("Target FPS not found in command!");
    }
  }

  if (strcmp(cmd[0], "StopRead") == 0) {
    TCPcommand = stop;
    hasNewTcpCommand = true;
  }

  if (strcmp(cmd[0], "Reset") == 0) {
    TCPcommand = reset;
    hasNewTcpCommand = true;
  }

  if (strcmp(cmd[0], "ReScan") == 0) {
    TCPcommand = rescan;
    hasNewTcpCommand = true;
  }
}

void handleError(void* arg, AsyncClient* client, int8_t error) {
	Serial.printf("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
  TCPcommand = stop;
  hasNewTcpCommand = true;
}

void handleDisconnect(void* arg, AsyncClient* client) {
	Serial.printf("\n client %s disconnected \n", client->remoteIP().toString().c_str());
  TCPcommand = stop;
  hasNewTcpCommand = true;
}

void handleTimeOut(void* arg, AsyncClient* client, uint32_t time) {
	Serial.printf("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
  TCPcommand = stop;
  hasNewTcpCommand = true;
}

void handleNewClient(void* arg, AsyncClient* client) {
	Serial.printf("\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());
	theClient = client;
	// register events
	client->onData(&handleData, NULL);
	client->onError(&handleError, NULL);
	client->onDisconnect(&handleDisconnect, NULL);
	client->onTimeout(&handleTimeOut, NULL);
}

void TCPServer_Setup() {
  server = new AsyncServer(TCPPort); // start listening on tcp port 1234
	server->onClient(&handleNewClient, server);
	server->begin();
  Serial.println("TCP Server is on!");
}