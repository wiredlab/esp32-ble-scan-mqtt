# BLE Scanning Improvements

This project is an ESP32 BLE advertisement scanner. Its useful output is bounded by what the BLE controller hears, what NimBLE reports to the sketch, and what the application can enqueue before publishing.

Two colocated scanners should not be expected to produce identical packet streams. BLE advertisers transmit on three advertising channels. A scanner listens on one advertising channel at a time and changes channels according to its own unsynchronized scan interval. Even with identical firmware and antennas only inches apart, the units will have different channel timing, WiFi coexistence timing, RSSI margins, and controller state.

## Current Behavior

The sketch configures NimBLE in `startBLEScan()`:

- Active scan is enabled with `pBLEScan->setActiveScan(true)`.
- The scan interval is `100 ms`.
- The scan window is `99 ms`.
- `KEEP_DUPLICATES` controls the duplicate filter through `setScanCallbacks(..., KEEP_DUPLICATES)`.
- The scan is restarted after `BLE_SCAN_TIME` seconds.

With `KEEP_DUPLICATES=false`, NimBLE asks the controller to report each address once per scan. That is useful for device discovery, but it is not packet capture. A device that emits multiple changing advertisements from the same address can be under-reported.

## Expected Packet Differences Between Nearby Scanners

Different packet sets are normal when both devices show low or zero software drops. Important causes:

- Advertising packets are repeated probabilistically across channels, not delivered reliably.
- Scanner channel hopping is not synchronized between ESP32 boards.
- WiFi and BLE share the ESP32 radio, so WiFi activity can steal receive opportunities.
- Active scanning sends scan requests and waits for scan responses, changing timing and airtime use.
- Marginal packets near the RSSI/noise threshold may decode on one unit and not the other.
- Duplicate filtering suppresses repeated packets by address, so small timing differences can change which payload is reported.

## Recommended Changes

### Use passive scanning for raw advertisement capture

Prefer:

```cpp
pBLEScan->setActiveScan(false);
```

Passive scan reports advertisements without sending scan requests. This generally maximizes raw observation time and avoids delaying scannable devices while waiting for scan responses. If scan-response data or device names are important, make active scanning a separate mode or run short active scan windows occasionally.

### Enable duplicate callbacks when packet volume matters

Prefer:

```cpp
const bool KEEP_DUPLICATES = true;
```

Then do application-level filtering by `(address, payload hash, time bucket)` rather than relying on the controller's address-level duplicate filter. This preserves changing payloads from the same device while still allowing the application to reduce publish volume.

### Consider continuous or longer scans

Short scans create restart boundaries. Those boundaries clear duplicate state, but they can also introduce gaps and scan-end processing. If the priority is packet capture, use a continuous scan or much longer scan duration, then restart only for recovery.

Options:

- `pBLEScan->start(0, false, true)` for continuous scanning.
- Increase `BLE_SCAN_TIME` substantially if continuous scan is not reliable on the deployed library version.
- Keep a watchdog that restarts scanning only when packet counters or scanner state indicate a likely stuck controller.

### Tune interval and window empirically

The installed NimBLE-Arduino version interprets `setInterval()` and `setWindow()` arguments as milliseconds. `100/99` gives high duty cycle, but each channel dwell is relatively long. Test these alternatives with the new metrics described in `observability-and-tuning.md`:

- `50/50`: high duty cycle, faster channel rotation.
- `30/30`: higher channel rotation rate, potentially more overhead.
- `100/100`: true continuous window if accepted by the controller.

The best setting depends on local advertiser intervals, WiFi traffic, ESP32 board layout, and MQTT/logging load.

## Proposed Filtering Model

When `KEEP_DUPLICATES=true`, add a small recent-seen table:

- Key: BLE address plus a fast hash of the payload.
- Value: last-seen timestamp and repeat count.
- Behavior: publish the first instance immediately, suppress exact repeats for a configurable interval, and periodically publish summary counts if desired.

This makes the scanner a packet-oriented observer instead of an address-oriented device discovery tool.

