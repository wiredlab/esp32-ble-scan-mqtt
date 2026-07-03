
#ifndef LOGGER_H
#define LOGGER_H

const uint8_t LOG_QUEUE_SIZE = 16;
const size_t LOG_MESSAGE_SIZE = 768;
const size_t LOG_LINE_SIZE = 10 + 1 + (LOG_MESSAGE_SIZE - 1) + 1 + 1;

struct LogMessage {
  size_t len;
  char line[LOG_LINE_SIZE];
};

LogMessage serialLogQueue[LOG_QUEUE_SIZE];
volatile uint8_t serialLogHead = 0;
volatile uint8_t serialLogTail = 0;
volatile unsigned long serialLogDropped = 0;

#if ENABLE_SDCARD
LogMessage sdLogQueue[LOG_QUEUE_SIZE];
volatile uint8_t sdLogHead = 0;
volatile uint8_t sdLogTail = 0;
#endif
volatile unsigned long sdLogDropped = 0;

bool queueLogMessage(LogMessage *queue,
                     volatile uint8_t *head,
                     volatile uint8_t *tail,
                     volatile unsigned long *dropped,
                     const char *line,
                     size_t len)
{
  uint8_t nextHead = (*head + 1) % LOG_QUEUE_SIZE;

  if (nextHead == *tail) {
    (*dropped)++;
    return false;
  }

  LogMessage *msg = &queue[*head];
  msg->len = min(len, sizeof(msg->line) - 1);
  memcpy(msg->line, line, msg->len);
  msg->line[msg->len] = '\0';
  *head = nextHead;
  return true;
}

bool queueSerialLog(const char *line, size_t len)
{
  return queueLogMessage(serialLogQueue,
                         &serialLogHead,
                         &serialLogTail,
                         &serialLogDropped,
                         line,
                         len);
}

bool queueSdLog(const char *line, size_t len)
{
#if ENABLE_SDCARD
  return queueLogMessage(sdLogQueue,
                         &sdLogHead,
                         &sdLogTail,
                         &sdLogDropped,
                         line,
                         len);
#else
  (void)line;
  (void)len;
  return false;
#endif
}

void drainSerialLogQueue(uint8_t maxMessages)
{
  uint8_t printed = 0;

  while (serialLogTail != serialLogHead && printed < maxMessages) {
    LogMessage *msg = &serialLogQueue[serialLogTail];
    Serial.write((const uint8_t *)msg->line, msg->len);
    serialLogTail = (serialLogTail + 1) % LOG_QUEUE_SIZE;
    printed++;
  }
}

void drainSdLogQueue(uint8_t maxMessages, bool sdcard_available)
{
#if ENABLE_SDCARD
  if (!sdcard_available || sdLogTail == sdLogHead) {
    return;
  }

  FILE *file = fopen("/sdcard/ble-sniffer.log", "a");
  if (!file) {
    uint8_t dropped = 0;
    while (sdLogTail != sdLogHead && dropped < maxMessages) {
      sdLogTail = (sdLogTail + 1) % LOG_QUEUE_SIZE;
      sdLogDropped++;
      dropped++;
    }
    return;
  }

  uint8_t written = 0;
  while (sdLogTail != sdLogHead && written < maxMessages) {
    LogMessage *msg = &sdLogQueue[sdLogTail];
    if (fwrite(msg->line, 1, msg->len, file) != msg->len) {
      sdLogDropped++;
    }
    sdLogTail = (sdLogTail + 1) % LOG_QUEUE_SIZE;
    written++;
  }
  fclose(file);
#else
  (void)maxMessages;
  (void)sdcard_available;
#endif
}

void drainLogQueues(uint8_t maxSerialMessages,
                    uint8_t maxSdMessages,
                    bool sdcard_available)
{
  drainSerialLogQueue(maxSerialMessages);
#if ENABLE_SDCARD
  drainSdLogQueue(maxSdMessages, sdcard_available);
#else
  (void)maxSdMessages;
  (void)sdcard_available;
#endif
}

void logger(const char *buf, bool sdcard_available)
{
  char str[LOG_LINE_SIZE];
  int len = snprintf(str, sizeof(str), "%lu %s\n", millis(), buf);

  if (len < 0) {
    return;
  }

  size_t boundedLen = min((size_t)len, sizeof(str) - 1);
  str[boundedLen] = '\0';

  queueSerialLog(str, boundedLen);
#if ENABLE_SDCARD
  if (sdcard_available) {
    queueSdLog(str, boundedLen);
  }
#else
  (void)sdcard_available;
#endif
}

#endif
