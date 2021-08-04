/*
 * Hardware target:
 *  ESP32 DEVKIT V1
 *  generic boards from eBay/Amazon with ESP32-WROOM-32 module
 *
 * IDE setup:
 *  Board:  ESP32 Dev Module
 *  CPU frequency: 240MHz (WiFi/BT)
 *  Flash frequency: 80MHz
 *  Flash size: 4MB (32Mb)
 *  Partition scheme: Huge App (3MB No OTA/1MB SPIFFS)
 *
 *
 * TODO:
 * 
 * Some devices (e.g. Mooshimeter) broadcast multiple types of advertisements
 * with different payloads.  The BLE library can be set to only return new
 * addresses during a scan interval, which then misses these multiple packets.
 * pBLEScan->setAdvertisedDeviceCallbacks(..., bool keepDuplicates)
 * 
 * It would be potentially useful to record *unique advertisements*, meaning
 * only ignore duplicates of *both* address+payload.  The BLE library doesn't
 * do this (address only).
 * 
 * Make a hash of the payload, store the [address, pHash, time] in an array and
 * check for membership before deciding to pass on via MQTT.
 * 
 * Put a time limit on when to repeat heard devices, so a
 * conntinuously-broadcasting device would only be recorded every INTERVAL
 * seconds instead of its (shorter) beacon interval.
 */



#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiMulti.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include "time.h"


/*
 * available in the Library Manger
 * https://github.com/nkolban/ESP32_BLE_Arduino
 */
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>


/*
 * configuration includes passwords/etc
 * include separately to not leak private information
 */
#include "config.h"


/*
 * globals
 */
String topic;
struct tm timeinfo;

uint8_t mac[6];
String my_mac;

// blink-per-message housekeeping
int nBlinks = 0;
bool led_state = 0;
bool in_blink = false;
unsigned long last_blink = 0;


BLEScan* pBLEScan;
bool is_scanning = false;
bool cold_boot = true;  // treat power-on differently than re-starting a scan


WiFiClient wifi;
WiFiMulti wifiMulti;  // use multiple wifi options
WiFiUDP udp;
NTPClient ntpClient(udp, NTP_SERVER, 0, NTP_UPDATE_INTERVAL);
PubSubClient mqtt(wifi);


/*
 * Given a byte array of length (n), return the ASCII hex representation
 * and properly zero pad values less than 0x10.
 * String(0x08, HEX) will yield '8' instead of the expected '08' string
 */
String hexToStr(uint8_t* arr, int n)
{
  String result;
  for (int i = 0; i < n; ++i) {
    if (arr[i] < 0x10) {result += '0';}
    result += String(arr[i], HEX);
  }
  return result;
}



/*
 * Callback that gets called on every received BLE advertisement.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      // Construct a JSON-formatted string with device information
      String msg = "{";
      msg.reserve(300);
      
      msg.concat("\"time\":\"");
      msg.concat(getIsoTime());
      msg.concat("\",");
      
      msg.concat("\"mac\":\"");
      msg.concat(hexToStr(*advertisedDevice.getAddress().getNative(), 6));
      msg.concat("\",");

      msg.concat("\"payload\":\"");
      msg.concat(hexToStr(advertisedDevice.getPayload(), advertisedDevice.getPayloadLength()));
      msg.concat("\",");

      if (advertisedDevice.haveRSSI()) {
        msg.concat("\"rssi\":");
        msg.concat(advertisedDevice.getRSSI());
        msg.concat(",");
      }

      if (advertisedDevice.haveTXPower()) {
        msg.concat("\"tx\":");
        msg.concat(advertisedDevice.getTXPower());
        msg.concat(",");
      }

      if (advertisedDevice.haveName()) {
        msg.concat("\"name\":\"");
        msg.concat(advertisedDevice.getName().c_str());
        msg.concat("\",");
      }


      // trim the final comma to ensure valid JSON
      if (msg.endsWith(",")) {
        msg.remove(msg.lastIndexOf(","));
      }
      msg.concat("}");

      // Publish the string via MQTT
      mqtt.beginPublish(topic.c_str(), msg.length(), false);
      mqtt.print(msg.c_str());
      // mqtt.endPublish();  // does nothing

      // Blink for every received advertisement
      nBlinks += 1;

      Serial.println(msg);
      //Serial.printf("BLE: %s \n", advertisedDevice.toString().c_str());
    }
};



/*
 * Callback invoked when scanning has completed.
 */
static void scanCompleteCB(BLEScanResults scanResults) {
  is_scanning = false;
} // scanCompleteCB



/*
 * Start a scan for BLE advertisements
 * if reinit is true, then re-setup the scan configuration parameters
 */
void startBLEScan(bool reinit=false)
{
  Serial.println("Starting BLE scan.");
  if (reinit) {
    BLEDevice::deinit(true);
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), KEEP_DUPLICATES);  // keepDuplicates?
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);  // less or equal to setInterval value
  }

  // forget about the devices seen in the last BLE_SCAN_TIME interval
  pBLEScan->start(BLE_SCAN_TIME, scanCompleteCB, false);
  is_scanning = true;
}


/*
 * Called whenever a payload is received from a subscribed MQTT topic
 */
void mqtt_receive_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT-receive [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

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
 * Check WiFi connection, attempt to reconnect.
 * This blocks until a connection is (re)established.
 */
void check_wifi()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi connection lost, reconnecting");
    while (wifiMulti.run() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }
}


/*
 * Check the MQTT connection state and attempt to reconnect.
 * If we do reconnect, then subscribe to MQTT_CONTROL_TOPIC and
 * make an announcement to MQTT_ANNOUNCE_TOPIC with the WiFi SSID and
 * local IP address.
 */
void check_mqtt()
{
  if (mqtt.connected()) { return; }

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
    Serial.println(msg);
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
    delay(5000);
  }
}


/*
 * Return a string in RFC3339 format of the current time.
 * Will return a placeholder if there is no network connection to an
 * NTP server.
 */
String getIsoTime()
{
  char timeStr[21] = {0};  // NOTE: change if strftime format changes

  time_t time_now = ntpClient.getEpochTime();
  localtime_r(&time_now, &timeinfo);

  if (timeinfo.tm_year <= (2016 - 1900)) {
    Serial.println("Failed to obtain time.");
    timeStr[0] = 0;
    return String("YYYY-MM-DDTHH:MM:SSZ");
  }

  strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(timeStr);
}



void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led_state);

  /*
   * setup WiFi
   */
  //WiFi.mode(WIFI_STA);
  for (int i=0; i<NUM_WLANS; i++) {
    wifiMulti.addAP(WLAN_SSID[i], WLAN_PASS[i]);
  }

  WiFi.macAddress(mac);
  my_mac = hexToStr(mac, 6);
  
  Serial.print("MAC: ");
  Serial.println(my_mac);

  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  /*
   * setup MQTT
   */
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqtt_receive_callback);
  topic = MQTT_PREFIX_TOPIC + my_mac + MQTT_BLE_TOPIC;

  delay(1000);

  /*
   * setup NTP time sync
   */
  ntpClient.begin();
}




void loop() {
  check_wifi();
  check_mqtt();

  mqtt.loop();
  ntpClient.update();

  // Handle blinking without using delay()
  unsigned long now = millis();
  if (nBlinks > 0) {
    if (now - last_blink >= BLINK_MS) {
      last_blink = now;
      if (in_blink) { // then finish
        digitalWrite(LED_PIN, led_state);
        nBlinks--;
        in_blink = false;
      } else {
        digitalWrite(LED_PIN, !led_state);
        in_blink = true;
      }
    }
  }

  // Setup BLE scanning and restart a scan
  // only fire off a new scan if we are not already scanning!
  if (not is_scanning) {
    if (cold_boot) {
      startBLEScan(true);
      cold_boot = false;
    } else {
      startBLEScan();
    }
  }

  delay(1);
}
