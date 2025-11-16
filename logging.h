#pragma once

class Logger {
public:
  struct LogEntry {
    unsigned long timestamp;
    char message[64];
  };

private:
  static const int LOG_BUFFER_SIZE = 10;
  LogEntry logBuffer[LOG_BUFFER_SIZE];
  int logWriteIndex = 0;
  int logReadIndex = 0;
  int logCount = 0;

public:
  void addLog(const char *msg);
  void flushLogs();
};