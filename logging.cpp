#include "logging.h"
#include <Arduino.h>

void Logger::addLog(const char *msg) {
  if (logCount < LOG_BUFFER_SIZE) {
    logBuffer[logWriteIndex].timestamp = millis();
    strncpy(logBuffer[logWriteIndex].message, msg, 63);
    logBuffer[logWriteIndex].message[63] = '\0';
    logWriteIndex = (logWriteIndex + 1) % LOG_BUFFER_SIZE;
    logCount++;
  }
}

void Logger::flushLogs() {
  while (logCount > 0 && Serial.availableForWrite() > 64) {
    Serial.print("[");
    Serial.print(logBuffer[logReadIndex].timestamp);
    Serial.print("] ");
    Serial.println(logBuffer[logReadIndex].message);

    logReadIndex = (logReadIndex + 1) % LOG_BUFFER_SIZE;
    logCount--;
  }
}