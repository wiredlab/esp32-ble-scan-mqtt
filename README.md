# esp32-ble-scan-mqtt

ESP32 firmware for scanning Bluetooth Low Energy advertisements and publishing them to MQTT.

The scanner records BLE advertisement metadata and raw payloads, formats them as JSON, logs them through serial output, and publishes them to a per-device MQTT topic. It is intended for always-on BLE observation using inexpensive ESP32-WROOM-32 development boards.

## Features

- BLE advertisement scanning with NimBLE-Arduino.
- JSON output containing timestamp, advertiser MAC, payload, RSSI, optional TX power, and decoded name fields.
- MQTT publishing with per-device topics based on the ESP32 WiFi MAC address.
- Periodic MQTT status messages with packet and drop counters.
- Optional SD card logging.
- Non-blocking queue handoff between BLE callbacks, logging, and MQTT publishing.

## Hardware

The sketch targets generic ESP32 development boards using ESP32-WROOM-32 modules. The original development target was the Arduino IDE board profile `ESP32 Dev Module`.

Typical settings:

- CPU frequency: `240MHz (WiFi/BT)`
- Flash frequency: `80MHz`
- Flash size: `4MB`
- Partition scheme: `Minimal SPIFFS (1.9MB APP with OTA/128KB SPIFFS)`

## Dependencies

Install these Arduino libraries:

- NimBLE-Arduino
- NTPClient
- PubSubClient
- ArduinoJson

The ESP32 Arduino core provides the WiFi, UDP, SD/MMC, and ESP system headers used by the sketch.

## Configuration

Copy `config-example.h` to `config.h` and edit it for your environment:

```bash
cp config-example.h config.h
```

`config.h` is ignored by git because it contains WiFi and MQTT credentials.

Important settings:

- `WLAN_SSID`, `WLAN_PASS`, and `NUM_WLANS`: WiFi networks.
- `MQTT_SERVER`, `MQTT_PORT`, `MQTT_USER`, and `MQTT_PASS`: MQTT connection.
- `MQTT_PREFIX_TOPIC`: base topic for scanner messages.
- `BLE_SCAN_TIME`: scan duration before restart.
- `KEEP_DUPLICATES`: whether NimBLE reports repeated advertisements.
- `ENABLE_SDCARD`: compile-time SD logging switch.

## MQTT Topics

The device ID is the ESP32 WiFi station MAC address encoded as uppercase hex.

Default topic construction:

- Advertisement payloads: `<MQTT_PREFIX_TOPIC><device-mac>/ble`
- Status payloads: `<MQTT_PREFIX_TOPIC><device-mac>/status`
- Control payloads: `<MQTT_PREFIX_TOPIC><device-mac>/control`

The configured prefix currently includes a trailing slash in the example, so the actual default topic shape is `valpo/esp32-sniffer/<device-mac>/ble`.

## Advertisement Payload

Example advertisement JSON:

```json
{"time":"2026-07-09T22:36:48Z","mac":"A4C138D804D2","payload":"12161A18D204D838C1A4020A3A19DA0A408F050B094154435F443830344432","rssi":-61,"name_complete":"ATC_D804D2"}
```

Fields:

- `time`: capture timestamp from NTP epoch time.
- `mac`: advertiser address as uppercase hex.
- `payload`: raw BLE advertisement payload as uppercase hex.
- `rssi`: received signal strength.
- `tx`: TX power when available.
- `name_short`: decoded short local name when present.
- `name_complete`: decoded complete local name when present.

## Status Payload

Status messages include:

- Connection state.
- Time and uptime.
- Packet count for the last status interval.
- Queue drop counters.
- WiFi SSID, RSSI, IP, and hostname.
- Firmware git version.

These counters are useful for distinguishing normal BLE reception variance from software queue pressure.

## BLE Scanning Expectations

Two ESP32 scanners placed close together should not be expected to hear exactly the same advertisements. BLE advertising uses three channels, scanner channel timing is unsynchronized, WiFi shares the ESP32 radio, and duplicate filtering can suppress repeated packets before the application sees them.

Packet-level equality is not a realistic acceptance criterion. Compare aggregate packet rates, queue drops, queue pressure, WiFi health, and MQTT publish reliability over multi-minute windows.

## Documentation

Additional engineering notes:

- [Firmware design and assumptions](docs/design.md)
- [BLE scanning improvements](docs/scanning-improvements.md)
- [MQTT and queueing improvements](docs/mqtt-queueing-improvements.md)
- [Observability and tuning](docs/observability-and-tuning.md)

## Development Notes

The BLE callback should stay short. It currently copies advertisement data into a queue, and the main loop later handles JSON formatting, logging, and MQTT publishing. Preserve that separation when changing the scanner.

Run-time credentials and site-specific settings belong in `config.h`; shared defaults and documentation belong in `config-example.h` and this README.
