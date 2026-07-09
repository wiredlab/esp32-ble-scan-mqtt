# Observability and Tuning

Scanner tuning should be evidence-driven. Packet count differences between devices are expected, so the useful question is whether differences come from normal BLE reception variance or from avoidable software bottlenecks.

## Current Status Payload

The status payload already reports:

- State.
- Time and uptime.
- Packet count in the last status interval.
- Dropped MQTT queue messages.
- Dropped advertisement queue messages.
- Dropped serial and SD log messages.
- WiFi SSID, RSSI, IP, and hostname.
- Firmware version.

This is enough to identify severe queue overflow, but it does not show queue pressure before drops happen.

## Add Queue Depth Metrics

For each ring buffer, report current depth and high-water mark. A queue can be close to overflowing for long periods before drops occur. High-water marks are the best signal for tuning.

Recommended fields:

- `adv_queue_depth`
- `adv_queue_high_water`
- `mqtt_queue_depth`
- `mqtt_queue_high_water`
- `serial_queue_depth`
- `serial_queue_high_water`
- `sd_queue_depth`
- `sd_queue_high_water`

## Add Scan Metrics

Recommended fields:

- `scan_starts`
- `scan_start_failures`
- `scan_completions`
- `scan_last_end_reason`
- `scan_active`
- `scan_active_mode`
- `scan_interval_ms`
- `scan_window_ms`
- `scan_keep_duplicates`

These make the deployed scan mode visible from MQTT and from serial logs.

## Add Publish Metrics

Recommended fields:

- `mqtt_publish_attempts`
- `mqtt_publish_success`
- `mqtt_publish_failed`
- `mqtt_write_short`
- `mqtt_reconnects`
- `mqtt_last_state`

These distinguish "not hearing BLE" from "heard BLE but could not publish it."

## Tuning Procedure

Use one variable at a time:

1. Establish a baseline with the current firmware and record status payloads from both scanners.
2. Disable serial logging or increase baud rate if serial queue pressure appears.
3. Switch from active scan to passive scan and compare packet count, queue pressure, and MQTT reliability.
4. Enable duplicate callbacks and add application-level suppression.
5. Test scan interval/window pairs such as `50/50`, `30/30`, and `100/100`.
6. Increase queue sizes only after measuring whether the bottleneck is burst absorption or sustained publish throughput.

## Comparing Two Scanners

For colocated devices, compare these separately:

- Total packets heard per interval.
- Unique `(address, payload)` pairs per interval.
- RSSI distribution by address.
- Queue high-water marks and drop counters.
- MQTT publish failure counters.
- WiFi RSSI and reconnect counts.

Do not treat exact packet-by-packet equality as the target. The practical target is that both devices report similar aggregate rates, low queue pressure, no sustained drops, and no silent publish failures.

