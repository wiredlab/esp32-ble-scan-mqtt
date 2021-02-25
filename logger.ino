// Save line to file on sd card
// open new file if:
//    no existing file
//    current file is too large
//    UTC date has changed
//



void sdcard_logger(const char *buf)
{
  char str[256];
  int len = 0;
  
  len = sprintf(str, "%lu %s\n", millis(), buf);
  len = len;  // make gcc happy
  
  Serial.print(str);
  
  if (sdcard_available) {
    FILE *file = fopen("/sdcard/ble-sniffer.log", "a");
    fprintf(file, "%s", str);
    fclose(file);
  }
}


/*
 * task to read from the message queue and publish via MQTT
 */
void vSdCardWriterTask(void *pvParameters)
{
  for ( ;; ) {
    char str[256];
    if (xQueueReceive(q_sdcard, &str, portMAX_DELAY) == pdTRUE) {
      sdcard_logger(str);
    }
    taskYIELD();
  }
}
