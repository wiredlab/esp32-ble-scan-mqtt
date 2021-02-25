/*
 * Check the MQTT connection state and attempt to reconnect.
 * If we do reconnect, then subscribe to MQTT_CONTROL_TOPIC and
 * make an announcement to MQTT_ANNOUNCE_TOPIC with the WiFi SSID and
 * local IP address.
 */
bool mqtt_check()
{
  if (mqtt.connected()) { return true; }

  // no point to continue if no network
  if (WiFi.status() != WL_CONNECTED) { return false; }

  // reconnect
  Serial.print("MQTT reconnect...");
  // Attempt to connect
  int connect_status = mqtt.connect(my_mac.c_str(), MQTT_USER, MQTT_PASS,
                   (MQTT_PREFIX_TOPIC + my_mac + MQTT_ANNOUNCE_TOPIC).c_str(),
                   2,  // willQoS
                   1,  // willRetain
                   "{\"state\":\"disconnected\"}");
  if (connect_status) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    // JSON formatted payload
    String msg = "{\"state\":\"connected\",\"ssid\":\"" + WiFi.SSID()
                  + "\",\"ip\":\"" + WiFi.localIP().toString()
                  + "\"}";
    sdcard_logger(msg.c_str());
    mqtt.publish((MQTT_PREFIX_TOPIC + my_mac + MQTT_ANNOUNCE_TOPIC).c_str(),
                 msg.c_str(),
                 true);

    // ... and resubscribe
    mqtt.subscribe((MQTT_PREFIX_TOPIC + my_mac + MQTT_CONTROL_TOPIC).c_str());
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    //delay(5000);
  }
  return mqtt.connected();
}




/*
 * Called whenever a payload is received from a subscribed MQTT topic
 */
void mqtt_receive_callback(char* topic, byte* payload, unsigned int length) {
  char buffer[256];
  buffer[0] = 0;

  int len = sprintf(buffer, "MQTT-receive %s ", topic);

  for (int i = 0; i < length; i++) {
    len += sprintf(buffer, "%02x", (char)payload[i]);
  }
  len += sprintf(buffer, "\n");

  sdcard_logger(buffer);

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    led_state = 1;
  } else {
    led_state = 0;
  }

  // this will effectively be a half-blink, forcing the LED to the
  // requested state
  nBlinks += 1;
}


/*
 * Publish a JsonDocument via MQTT
 */
bool mqtt_publish_json(char* buffer)
{
  bool result = false;
  size_t len = strlen(buffer);
  
  if (mqtt.connected()) {
    result = mqtt.publish(topic.c_str(), buffer, len);
  }
  return result;
}


/*
 * task to read from the message queue and publish via MQTT
 */
void vMqttPublisherTask(void *pvParameters)
{
  for ( ;; ) {
    char str[256];
    if (xQueueReceive(q_mqtt, &str, portMAX_DELAY) == pdTRUE) {
      mqtt_publish_json(str);
    }
    taskYIELD();
    }
}
