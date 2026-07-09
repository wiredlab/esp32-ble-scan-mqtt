# Firmware Design and Assumptions

This document describes the ESP32 BLE scanner firmware as it exists today. It focuses on the assumptions that are easy to miss when reading the sketch directly.

## Purpose

The firmware listens for BLE advertisements, records selected advertisement fields, logs each record locally through serial output, and publishes each record to MQTT. It also publishes periodic status messages containing telemetry and drop counters.

The firmware is best understood as a bridge:

```text
BLE controller -> NimBLE callback -> advertisement queue -> JSON/logging -> MQTT queue -> MQTT broker
```

The BLE side is bursty and timing-sensitive. The MQTT side can block or slow down because it depends on WiFi, TCP, and broker availability. The queues between those sides are the main protection against packet loss caused by slow publishing.

## Runtime Environment

The sketch targets generic ESP32-WROOM-32 development boards through the Arduino ESP32 environment. It uses:

- NimBLE-Arduino for BLE scanning.
- WiFi and WiFiMulti for network connection.
- NTPClient for timestamps.
- PubSubClient for MQTT.
- ArduinoJson for JSON serialization.
- Optional SD card logging behind `ENABLE_SDCARD`.

The active runtime configuration lives in `config.h`, which is intentionally ignored by git because it contains local network and MQTT credentials. `config-example.h` documents the expected settings.

## Startup Flow

`setup()` performs these steps:

1. Initializes serial output.
2. Optionally initializes the SD card.
3. Configures the LED pin.
4. Reads the WiFi station MAC address and uses it to build the device ID and MQTT topic.
5. Configures WiFi station mode and attempts an initial connection.
6. Configures the MQTT server and callback.
7. Starts NTP.

BLE scanning is not started in `setup()`. It starts from `loop()` when `is_scanning` is false. The first scan performs NimBLE initialization and scan parameter setup.

## Main Loop

Each `loop()` iteration does this work:

1. Check WiFi and reconnect when needed.
2. Check MQTT and reconnect when needed.
3. Update NTP and run `mqtt.loop()` when WiFi is connected.
4. Drain a bounded number of advertisement queue entries.
5. Drain a bounded number of serial and SD log entries.
6. Drain a bounded number of MQTT publish queue entries when MQTT is connected.
7. Start or restart BLE scanning when `is_scanning` is false.
8. Handle LED blinking.
9. Publish periodic status and apply the no-packet restart watchdog.
10. Delay for 1 ms.

The design assumes that no single step should monopolize the loop for long. BLE callbacks can still occur outside this loop while scanning is active, so queue depth matters.

## BLE Scan Configuration

`startBLEScan()` configures NimBLE only on cold boot or explicit reinitialization:

- Active scanning is enabled.
- Scan interval is set to 100 ms.
- Scan window is set to 99 ms.
- Duplicate reporting is controlled by `KEEP_DUPLICATES`.
- Scan duration is `BLE_SCAN_TIME` seconds.

In the installed NimBLE-Arduino version, scan interval and window are specified in milliseconds. Internally they are converted to BLE 0.625 ms units.

With active scan enabled, NimBLE may wait for scan responses before invoking `onResult()` for scannable advertisements. This can enrich data with scan-response fields, but it is not ideal for maximum raw advertisement capture.

With `KEEP_DUPLICATES=false`, the scanner behaves more like a device discovery tool than a packet sniffer. It generally reports one result per address per scan cycle, not every advertisement frame.

## Advertisement Capture Path

The NimBLE scan callback calls `queueAdvertisement()` and increments packet counters. The callback intentionally does not serialize JSON, write to serial, write to SD, or publish MQTT.

`queueAdvertisement()` copies these fields into `advQueue`:

- Timestamp from NTP epoch.
- Local `millis()` value.
- Advertiser address.
- RSSI.
- TX power when available.
- Raw advertisement payload.

This copy is important because the NimBLE advertised-device object is owned by the BLE library and may be reused or deleted later.

## Formatting and Logging Path

`drainAdvertisementQueue()` converts queued advertisements to JSON. It then:

1. Calls `logger()` with the JSON line.
2. Queues the same JSON payload for MQTT.
3. Advances the advertisement queue tail.

`logger()` adds the current `millis()` prefix and queues the line for serial output and, when enabled, SD output.

The timestamp in the JSON body is the advertisement capture time. The numeric prefix in the log line is the later formatting/logging time.

## MQTT Publish Path

MQTT messages are first copied into `mqttPublishQueue`. `drainMqttPublishQueue()` publishes a bounded number of messages each loop.

The current implementation assumes that if MQTT is connected at the start of a publish attempt, the publish succeeds. That assumption is too optimistic. The code should check `beginPublish()` and the byte count returned by `write()` before advancing the queue tail.

PubSubClient does not provide broker acknowledgement for QoS 0 publishes. A successful local write means the payload was handed to the TCP client, not that the broker stored it.

## Queue Assumptions

All queues are fixed-size ring buffers:

- One slot is reserved to distinguish full from empty.
- Queue counters are `uint8_t`, so queue sizes should stay below 256 unless the index types are changed.
- Queue operations are lock-free and rely on single-producer/single-consumer behavior.

The advertisement queue is written from the NimBLE callback context and drained from the Arduino loop. The queue variables are marked `volatile`, but multi-byte counters such as drop counters are not protected against concurrent reads and writes. For status telemetry this is acceptable enough, but it should not be treated as exact accounting under interrupt-like concurrency.

## Packet Loss Model

Packet loss can happen at several layers:

- RF loss: the packet was not decoded by the ESP32 radio.
- Scan scheduling loss: the scanner was listening on a different advertising channel.
- WiFi/BLE coexistence loss: WiFi activity prevented BLE reception.
- Controller duplicate filtering: a decoded packet was suppressed before the application saw it.
- NimBLE result timing: active-scan response handling delayed or changed callback behavior.
- Advertisement queue overflow: `advQueue` was full.
- MQTT queue overflow: formatted messages could not be queued for publish.
- MQTT publish failure: TCP write failed after the message left the software queue.
- Serial or SD log overflow: diagnostics were dropped.

Only some of these are currently visible in status messages.

## Status and Watchdog Assumptions

Periodic status reports are intended to show health and loss counters. The no-packet watchdog restarts the device after a configured number of status intervals with no packets.

The current `last_status` initialization causes an early status check shortly after startup because unsigned `millis()` arithmetic wraps when `millis()` is less than `last_status`. That early status can report `packets:0` before BLE scanning has had a fair interval.

## Operational Expectations

Two devices running this firmware near each other should have similar aggregate packet rates over time, but they should not be expected to agree packet by packet. Exact equality is not a realistic acceptance criterion for BLE advertisement scanning.

Useful acceptance criteria are:

- Queue drop counters remain zero or near zero.
- Queue high-water marks stay below configured thresholds.
- MQTT publish failures are visible and rare.
- Packet rates remain similar across devices over multi-minute windows.
- WiFi RSSI and reconnect counts are healthy.
- Scanner restart counts remain low.

## Design Direction

The current architecture is sound at a high level: capture quickly, queue, then publish outside the BLE callback. The next iteration should preserve that structure while making these changes:

- Prefer passive scan for raw packet capture.
- Enable duplicate callbacks when packet-level capture is desired.
- Add application-level `(address, payload hash, time)` suppression.
- Add queue depth and high-water status metrics.
- Make MQTT publish failure visible and retryable.
- Make scan mode and scan timing visible in status messages.

