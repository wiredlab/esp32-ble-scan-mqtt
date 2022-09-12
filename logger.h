

void logger(const char *buf, bool sdcard_available)
{
  char str[256 + 10 + 1];  // buf is [256], uint32_t is 10 digits, null terminator
  
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
