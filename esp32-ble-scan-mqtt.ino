/*
 * Hardware target:
 *  generic boards from eBay/Amazon with ESP32-WROOM-32 module
 *    --> ESP32 DEVKIT V1
 *
 * IDE setup:
 *  Board:  ESP32 Dev Module
 *    add package via:  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *  CPU frequency: 240MHz (WiFi/BT)
 *  Flash frequency: 80MHz
 *  Flash size: 4MB (32Mb)
 *  Partition scheme: Minimal SPIFFS (1.9MB APP with OTA/128KB SPIFFS)
 *
 *
 * TODO:
 * 
 * Some devices (e.g. Mooshimeter) broadcast multiple types of advertisements
 * with different payloads.  The BLE library can be set to only return new
 * addresses during a scan interval, which then misses these multiple packets.
 * pBLEScan->setScanCallbacks(..., bool wantDuplicates)
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


// all 3 WiFi libraries from
// https://github.com/espressif/arduino-esp32
// already installed with ESP32 package
struct AdvertisementMessage;
struct DecodedAdvertisement;

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiMulti.h>

#include "time.h"

/*
 * configuration includes passwords/etc
 * include separately to not leak private information
 */
#include "config.h"

#ifndef ENABLE_SDCARD
#define ENABLE_SDCARD 1
#endif

#if ENABLE_SDCARD
// MicroSD
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#endif

// espressif to get MAC before startup
#include "esp_mac.h"


/*
 * Extra libraries installed from the Library Manager
 */
// NimBLE-Arduino
#include <NimBLEDevice.h>

// https://github.com/arduino-libraries/NTPClient
#include <NTPClient.h>

// https://pubsubclient.knolleary.net/
// PubSubClient by Nick O'Leary
#include <PubSubClient.h>

// https://arduinojson.org/
#include <ArduinoJson.h>




#include "logger.h"

/* https://github.com/fabianoriccardi/git-describe-arduino
 * add a GIT_VERSION string that is updated every compile event
 */
#include "git-version.h"



/*
 * globals
 */
String topic;
struct tm timeinfo;

uint8_t mac[6];
String my_mac;

// blink-per-message housekeeping
unsigned int nBlinks = 0;
bool led_state = 0;
bool in_blink = false;
typeof(millis()) last_blink = 0;

// status update housekeeping
unsigned long last_status = 30000;  // nonzero to defer our first status until triggered
unsigned long nPackets = 0;


NimBLEScan* pBLEScan;
bool is_scanning = false;
bool cold_boot = true;  // treat power-on differently than re-starting a scan


WiFiClient wifi;
WiFiMulti wifiMulti;  // use multiple wifi options
WiFiUDP udp;
NTPClient ntpClient(udp, NTP_SERVER, 0, NTP_UPDATE_INTERVAL);
PubSubClient mqtt(wifi);
unsigned long last_wifi_check = 0;
unsigned long last_mqtt_check = 0;

const uint8_t MQTT_PUBLISH_QUEUE_SIZE = 16;
const uint8_t ADV_QUEUE_SIZE = 16;
const uint16_t ADV_MAX_PAYLOAD_SIZE = 255;

struct MqttPublishMessage {
  size_t len;
  char payload[LOG_MESSAGE_SIZE];
};

struct AdvertisementMessage {
  uint32_t epoch;
  uint32_t millisSeen;
  uint8_t mac[6];
  int8_t rssi;
  bool hasTxPower;
  int8_t txPower;
  uint16_t payloadLen;
  uint8_t payload[ADV_MAX_PAYLOAD_SIZE];
};

MqttPublishMessage mqttPublishQueue[MQTT_PUBLISH_QUEUE_SIZE];
volatile uint8_t mqttPublishHead = 0;
volatile uint8_t mqttPublishTail = 0;
volatile unsigned long mqttPublishDropped = 0;

AdvertisementMessage advQueue[ADV_QUEUE_SIZE];
volatile uint8_t advHead = 0;
volatile uint8_t advTail = 0;
volatile unsigned long advDropped = 0;


volatile bool sdcard_available = false;



bool queueMqttPublish(const char *payload, size_t len)
{
  uint8_t nextHead = (mqttPublishHead + 1) % MQTT_PUBLISH_QUEUE_SIZE;

  if (nextHead == mqttPublishTail) {
    mqttPublishDropped++;
    return false;
  }

  MqttPublishMessage *msg = &mqttPublishQueue[mqttPublishHead];
  msg->len = min(len, sizeof(msg->payload) - 1);
  memcpy(msg->payload, payload, msg->len);
  msg->payload[msg->len] = '\0';
  mqttPublishHead = nextHead;
  return true;
}


void drainMqttPublishQueue(uint8_t maxMessages)
{
  uint8_t published = 0;

  while (mqtt.connected() &&
         mqttPublishTail != mqttPublishHead &&
         published < maxMessages) {
    MqttPublishMessage *msg = &mqttPublishQueue[mqttPublishTail];

    mqtt.beginPublish(topic.c_str(), msg->len, false);
    mqtt.write((const uint8_t *)msg->payload, msg->len);
    mqtt.endPublish();  // does nothing?

    mqttPublishTail = (mqttPublishTail + 1) % MQTT_PUBLISH_QUEUE_SIZE;
    published++;
  }
}


static void formatIsoTimeFromEpoch(uint32_t epoch, char *out, size_t outSize)
{
  if (outSize == 0) {
    return;
  }

  time_t time_now = (time_t)epoch;
  struct tm timestamp;
  localtime_r(&time_now, &timestamp);

  if (timestamp.tm_year <= (2016 - 1900)) {
    snprintf(out, outSize, "YYYY-MM-DDTHH:MM:SSZ");
  } else {
    strftime(out, outSize, "%Y-%m-%dT%H:%M:%SZ", &timestamp);
  }
}


static void hexToBuffer(const uint8_t *src, size_t srcLen, char *out, size_t outSize)
{
  static const char hex[] = "0123456789ABCDEF";
  size_t pos = 0;

  if (outSize == 0) {
    return;
  }

  for (size_t i = 0; i < srcLen && pos + 2 < outSize; i++) {
    out[pos++] = hex[src[i] >> 4];
    out[pos++] = hex[src[i] & 0x0F];
  }
  out[pos] = '\0';
}


struct DecodedAdvertisement {
  bool haveShortName;
  const uint8_t *shortName;
  size_t shortNameLen;
  bool haveCompleteName;
  const uint8_t *completeName;
  size_t completeNameLen;
  bool haveTxPower;
  int8_t txPower;
};


static void decodeAdvertisementPayload(const uint8_t *payload,
                                size_t payloadLen,
                                DecodedAdvertisement *decoded)
{
  memset(decoded, 0, sizeof(*decoded));

  size_t pos = 0;
  while (pos < payloadLen) {
    uint8_t fieldLen = payload[pos];
    if (fieldLen == 0) {
      break;
    }

    if (fieldLen < 1 || pos + 1 + fieldLen > payloadLen) {
      break;
    }

    uint8_t adType = payload[pos + 1];
    const uint8_t *data = payload + pos + 2;
    size_t dataLen = fieldLen - 1;

    switch (adType) {
      case 0x08:
        if (dataLen > 0) {
          decoded->haveShortName = true;
          decoded->shortName = data;
          decoded->shortNameLen = dataLen;
        }
        break;
      case 0x09:
        if (dataLen > 0) {
          decoded->haveCompleteName = true;
          decoded->completeName = data;
          decoded->completeNameLen = dataLen;
        }
        break;
      case 0x0A:
        if (dataLen >= 1) {
          decoded->haveTxPower = true;
          decoded->txPower = (int8_t)data[0];
        }
        break;
      default:
        break;
    }

    pos += 1 + fieldLen;
  }
}


static size_t formatAdvertisementJson(const AdvertisementMessage *msg, char *out, size_t outSize)
{
  char timeStr[21];
  char macStr[13];
  char payloadStr[ADV_MAX_PAYLOAD_SIZE * 2 + 1];
  DecodedAdvertisement decoded;

  if (outSize == 0) {
    return 0;
  }

  formatIsoTimeFromEpoch(msg->epoch, timeStr, sizeof(timeStr));
  hexToBuffer(msg->mac, sizeof(msg->mac), macStr, sizeof(macStr));
  hexToBuffer(msg->payload, msg->payloadLen, payloadStr, sizeof(payloadStr));
  decodeAdvertisementPayload(msg->payload, msg->payloadLen, &decoded);

  JsonDocument json;
  json["time"] = timeStr;
  json["mac"] = macStr;
  json["payload"] = payloadStr;
  json["rssi"] = msg->rssi;

  if (msg->hasTxPower) {
    json["tx"] = msg->txPower;
  } else if (decoded.haveTxPower) {
    json["tx"] = decoded.txPower;
  }

  if (decoded.haveShortName) {
    json["name_short"] = JsonString((const char *)decoded.shortName, decoded.shortNameLen);
  }

  if (decoded.haveCompleteName) {
    json["name_complete"] = JsonString((const char *)decoded.completeName, decoded.completeNameLen);
  }

  if (measureJson(json) >= outSize) {
    snprintf(out, outSize, "{\"error\":\"advertisement_json_overflow\"}");
    return strlen(out);
  }

  return serializeJson(json, out, outSize);
}


static bool queueAdvertisement(const NimBLEAdvertisedDevice *advertisedDevice)
{
  uint8_t nextHead = (advHead + 1) % ADV_QUEUE_SIZE;

  if (nextHead == advTail) {
    advDropped++;
    return false;
  }

  AdvertisementMessage *msg = &advQueue[advHead];
  const NimBLEAddress& address = advertisedDevice->getAddress();
  const uint8_t *nativeAddress = address.getVal();
  const std::vector<uint8_t>& payload = advertisedDevice->getPayload();
  size_t payloadLen = payload.size();

  msg->epoch = ntpClient.getEpochTime();
  msg->millisSeen = millis();
  for (uint8_t i = 0; i < sizeof(msg->mac); i++) {
    msg->mac[i] = nativeAddress[sizeof(msg->mac) - 1 - i];
  }
  msg->rssi = (int8_t)advertisedDevice->getRSSI();
  msg->hasTxPower = false;
  msg->txPower = 0;

  if (advertisedDevice->haveTXPower()) {
    msg->hasTxPower = true;
    msg->txPower = advertisedDevice->getTXPower();
  }

  if (payloadLen > sizeof(msg->payload)) {
    payloadLen = sizeof(msg->payload);
  }

  msg->payloadLen = payloadLen;
  if (payloadLen > 0) {
    memcpy(msg->payload, payload.data(), payloadLen);
  }

  advHead = nextHead;
  return true;
}


static void drainAdvertisementQueue(uint8_t maxMessages)
{
  uint8_t drained = 0;
  char buffer[LOG_MESSAGE_SIZE];

  while (advTail != advHead && drained < maxMessages) {
    AdvertisementMessage *msg = &advQueue[advTail];
    size_t len = formatAdvertisementJson(msg, buffer, sizeof(buffer));

    logger(buffer, sdcard_available);
    queueMqttPublish(buffer, len);

    advTail = (advTail + 1) % ADV_QUEUE_SIZE;
    drained++;
  }
}



/*
 * Callback that gets called on every received BLE advertisement.
 */
class MyAdvertisedDeviceCallbacks: public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
    queueAdvertisement(advertisedDevice);

    // Blink for every received advertisement
    nBlinks += 1;

    // Count packets heard
    nPackets += 1;
  }

  void onScanEnd(const NimBLEScanResults& scanResults, int reason) override {
    is_scanning = false;
  }
};



static MyAdvertisedDeviceCallbacks advertisedDeviceCallbacks;



/*
 * Start a scan for BLE advertisements
 * if reinit is true, then re-setup the scan configuration parameters
 */
void startBLEScan(bool reinit=false)
{
  Serial.println("Starting BLE scan.");
  if (reinit) {
    NimBLEDevice::deinit(true);
    NimBLEDevice::init("");
    pBLEScan = NimBLEDevice::getScan(); //create new scan
    pBLEScan->setScanCallbacks(&advertisedDeviceCallbacks, KEEP_DUPLICATES);  // keepDuplicates?
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);  // less or equal to setInterval value
  }

  // forget about the devices seen in the last BLE_SCAN_TIME interval
  pBLEScan->start((uint32_t)BLE_SCAN_TIME * 1000, false, true);
  is_scanning = true;
}


/*
 * Called whenever a payload is received from a subscribed MQTT topic
 */
void mqtt_receive_callback(char* topic, byte* payload, unsigned int length) {
  char buffer[256];
  buffer[0] = 0;

  int len = sprintf(buffer, "MQTT-receive %s ", topic);

  for (int i = 0; i < length; i++) {
    len += sprintf(&buffer[len], "%02x", (unsigned char)payload[i]);
  }
  len += sprintf(&buffer[len], "\n");

  logger(buffer, sdcard_available);

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
bool check_wifi()
{
  if (cold_boot || WiFi.status() != WL_CONNECTED) {
    if ((millis() - last_wifi_check) >= WIFI_RETRY_DELAY) {
      logger("WiFi connecting", sdcard_available);
      Serial.print("*");
      int retries = 5;
      //Serial.print("*");
      while (retries > 0 && wifiMulti.run() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        retries--;
      }

      if (retries == 0) {
        String msg = "WiFi: failed, waiting for " + String(WIFI_RETRY_DELAY/1000) + " seconds before trying again";
        logger(msg.c_str(), sdcard_available);
      } else {
        logger(WiFi.localIP().toString().c_str(), sdcard_available);
      }
      last_wifi_check = millis();
    } // retry delay
  } // !connected
  return (WiFi.status() == WL_CONNECTED);
}


/*
 * Check the MQTT connection state and attempt to reconnect.
 * If we do reconnect, then subscribe to MQTT_CONTROL_TOPIC and
 * make an announcement to MQTT_ANNOUNCE_TOPIC with the WiFi SSID and
 * local IP address.
 */
bool check_mqtt()
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
    // let everyone know we are alive
    pub_status_mqtt("connected");

    // ... and resubscribe to downlink topic
    mqtt.subscribe((MQTT_PREFIX_TOPIC + my_mac + MQTT_CONTROL_TOPIC).c_str());
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
  }
  return mqtt.connected();
}


/*
 * Publish a status message with system telemetry.
 */
bool pub_status_mqtt(const char *state)
{
  // JSON formatted payload
  JsonDocument status_json;
  status_json["state"] = state;
  status_json["time"] = getIsoTime();
  status_json["uptime_ms"] = millis();
  status_json["packets"] = nPackets;
  status_json["dropped_mqtt"] = mqttPublishDropped;
  status_json["dropped_adv"] = advDropped;
  status_json["dropped_serial"] = serialLogDropped;
  status_json["dropped_sd"] = sdLogDropped;
  status_json["ssid"] = WiFi.SSID();
  status_json["rssi"] = WiFi.RSSI();
  status_json["ip"] = WiFi.localIP().toString();
  status_json["hostname"] = WiFi.getHostname();
  status_json["version"] = GIT_VERSION;

  char buf[LOG_MESSAGE_SIZE];
  size_t len = serializeJson(status_json, buf, sizeof(buf));

  logger(buf, sdcard_available);

  if (mqtt.connected()) {
    String announceTopic = MQTT_PREFIX_TOPIC + my_mac + MQTT_ANNOUNCE_TOPIC;
    if (!mqtt.beginPublish(announceTopic.c_str(), len, true)) {
      return false;
    }
    mqtt.write((const uint8_t *)buf, len);
    return mqtt.endPublish();
  } else {
    return false;
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
    return String("YYYY-MM-DDTHH:MM:SSZ");
  } else {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(timeStr);
  }
}


/*
 * init_sdcard()
 * 
 * Setup the SD card for 
 */
#if ENABLE_SDCARD
esp_err_t init_sdcard()
{
  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 1,
    .allocation_unit_size = 512,
  };
  sdmmc_card_t *card;

  Serial.println("Mounting SD card...");
  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    sdcard_available = true;
    Serial.println("SD card mount successfully!");
  }  else  {
    sdcard_available = false;
    Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
    Serial.println();
  }
  return ret;
}
#endif



void setup() {
  Serial.begin(115200);
  Serial.println();


#if ENABLE_SDCARD
  /*
   * TODO: LED is normall on pin 2, unless this is the esp32-cam board.
   * There, pin 2 is used for the uSD card interface (HS2_DATA0).
   * Attempting to initialize the uSD card changes the pin modes, so we must change
   * the LED pin back if there is in fact no uSD card connected.
   */
  init_sdcard();
  Serial.print("LED pin: ");
  if (!sdcard_available) {
    Serial.println(LED_PIN);
    pinMode(LED_PIN, OUTPUT);
  } else {
    /*
     * assume at this point that this is the esp32-cam board,
     * which has an LED on GPIO33
     */
    Serial.print("LED pin: ");
    Serial.println(33);
    pinMode(33, OUTPUT);
  }
#else
  Serial.print("LED pin: ");
  Serial.println(LED_PIN);
  pinMode(LED_PIN, OUTPUT);
#endif
  digitalWrite(LED_PIN, led_state);


  /*
   * setup WiFi
   */
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char macStr[13];
  hexToBuffer(mac, sizeof(mac), macStr, sizeof(macStr));
  my_mac = macStr;

  String msg = "\nMAC: " + my_mac;
  logger(msg.c_str(), sdcard_available);

  msg = "ValpoSensorNet-" + my_mac;
  WiFi.setHostname(msg.c_str());

  WiFi.mode(WIFI_STA);

  for (int i=0; i<NUM_WLANS; i++) {
    wifiMulti.addAP(WLAN_SSID[i], WLAN_PASS[i]);
  }

  int retries = 5;
  while (retries > 0 && wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retries--;
  }
  logger(wifi.localIP().toString().c_str(), sdcard_available);


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
  ntpClient.update();

}




void loop() {
  bool wifi_good;
  bool mqtt_good;
  
  wifi_good = check_wifi();
  mqtt_good = check_mqtt();

  if (wifi_good) {
    ntpClient.update();
    mqtt.loop();
  }

  drainAdvertisementQueue(8);
  drainLogQueues(4, 4, sdcard_available);

  if (mqtt_good) {
    drainMqttPublishQueue(4);
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


  /*
   * Handle periodic events
   */
  unsigned long now = millis();

  // Handle blinking without using delay()
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

  static int no_packet_intervals = NO_PACKETS_INTERVALS_ZOMBIE_RESTART;
  if (now - last_status >= STATUS_INTERVAL) {
    pub_status_mqtt("status");

    if (nPackets > 0) {
      no_packet_intervals = NO_PACKETS_INTERVALS_ZOMBIE_RESTART;
    } else {
      Serial.println("no packets :(");
      no_packet_intervals--;

      if (no_packet_intervals == 0) {
        // no heard packets is a potential problem
        pub_status_mqtt("restarting");
        ESP.restart();
      }
    }

    nPackets = 0;
    last_status = millis();
  }
  /*
   * end periodic events
   */


  delay(1);
}
