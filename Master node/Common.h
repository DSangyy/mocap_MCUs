#pragma once

const char* ssid = "SANGPCWIFI";
const char* pass = "H415$63c";

const byte slaveReset = 17;
const int I2Ctimeout = 1;  // ms
const uint32_t I2Cclock = 400000;

uint8_t totalDevices = 0;
const uint8_t packetSize = 17;  // 4*32bit quaternion + 8bit CRC8
uint8_t devices[20] = {0};

uint8_t targetFps = 0;
unsigned long frameTime = 0;  // us
unsigned long frameStart = 0;
unsigned long failedPacket = 0;
unsigned long okPacket = 0;

bool isReading = false;
const uint8_t commandSize = 10;  // maximum 10 characters for each command/argument
const char* commandDelim = "|";

const uint32_t promptInterval = 1000000;  // us

hw_timer_t* promptTimer = NULL;
hw_timer_t* readingTimer = NULL;
volatile SemaphoreHandle_t promptSemaphore;
volatile SemaphoreHandle_t readingSemaphore;

enum TCPcommandlist {
  start, stop, getinfo, reset, rescan
};

TCPcommandlist TCPcommand;
bool hasNewTcpCommand = false;