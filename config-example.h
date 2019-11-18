/*
 * configuration
 *
 * copy to "config.h" and fill in the values
 */

// on-board LED blinks once per packet
// quiescent state (on or off) is set by "/control" topic reception
#define LED_PIN 2
#define BLINK_MS 50


// WiFi config
const char *WLAN_SSID[] = {"valpo-net"};
const char *WLAN_PASS[] = {"brownandgold"};
const int NUM_WLANS = 1;


// SNTP time config
const char *NTP_SERVER = "pool.ntp.org";
const int NTP_UPDATE_INTERVAL = 600000;  // ms between NTP queries


// MQTT settings
const char *MQTT_ANNOUNCE_TOPIC = "/status";
const char *MQTT_CONTROL_TOPIC = "/control";
const char *MQTT_BLE_TOPIC = "/ble";
const char *MQTT_SERVER = "mqtt.example.net";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "username";
const char *MQTT_PASS = "pass";


// Bluetooth LE settings
const int BLE_SCAN_TIME = 10; // seconds

// true: output every packet heard
// false: output first packet for each address heard in BLE_SCAN_TIME interval
const bool KEEP_DUPLICATES = false;
