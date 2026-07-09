# MQTT and Queueing Improvements

The scanner separates BLE receive callbacks from the slower JSON, logging, and MQTT paths. That is the right overall shape: the scan callback should stay short and avoid blocking operations.

The current implementation still has a few reliability gaps. The most important one is that MQTT publish errors can be invisible because the queue tail advances even when the underlying TCP write fails.

## Current Queues

The sketch has these in-memory queues:

- `advQueue`: raw advertisement snapshots from the NimBLE callback.
- `mqttPublishQueue`: formatted JSON payloads waiting for MQTT publish.
- `serialLogQueue`: lines waiting for serial output.
- `sdLogQueue`: lines waiting for SD card output when SD logging is enabled.

Each queue is a fixed-size ring buffer. The effective capacity is one less than the configured size because one slot is reserved to distinguish full from empty.

## Current MQTT Publish Risk

`drainMqttPublishQueue()` currently:

- Calls `mqtt.beginPublish()`.
- Writes the payload.
- Calls `mqtt.endPublish()`.
- Advances the queue tail unconditionally.

PubSubClient's `endPublish()` returns success in the installed implementation, so it is not a useful delivery check. The sketch should instead check:

- `beginPublish()` returned `true`.
- `mqtt.write()` returned exactly the payload length.
- `mqtt.connected()` remains true after the write.

If any of those fail, do not advance the queue tail. Disconnect or reconnect MQTT and retry later. Track a `mqttPublishFailed` counter separately from `mqttPublishDropped`.

## Recommended Backpressure Policy

Use high-water and low-water marks rather than only fixed per-loop counts.

Suggested behavior:

- Always keep `mqtt.loop()` frequent enough to maintain keepalive and receive control messages.
- Drain MQTT until the queue reaches a low-water mark or a small time budget expires.
- When the advertisement queue crosses a high-water mark, reduce optional work first: serial logging, SD logging, status publishes, and active scan response capture.
- When queues remain high, apply packet-level suppression before dropping arbitrary messages.

Example policy:

- MQTT low-water mark: 25% full.
- MQTT high-water mark: 75% full.
- Advertisement high-water mark: 75% full.
- Per-loop MQTT drain budget: 5 ms to 20 ms, measured on the target network.

## Preserve Capture Before Presentation

The highest-value data is the raw advertisement. JSON formatting, serial logs, SD logs, and MQTT are delivery and diagnostic paths. If the device is overloaded, prefer this order:

1. Keep the NimBLE callback short enough to capture raw advertisements.
2. Keep enough advertisement queue capacity to absorb bursts.
3. Publish MQTT reliably when connected.
4. Defer or sample serial/SD logs under pressure.

## Suggested Counters

Add these to the status payload:

- `adv_queue_depth`
- `adv_queue_high_water`
- `mqtt_queue_depth`
- `mqtt_queue_high_water`
- `serial_queue_depth`
- `sd_queue_depth`
- `mqtt_publish_failed`
- `mqtt_publish_retried`
- `mqtt_reconnects`

These counters make it possible to tune scan aggressiveness against publish reliability instead of guessing from packet counts alone.

