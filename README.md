# Laundry Monitor

A Particle Photon-based system that monitors washer and dryer status using current sensors and integrates with Home Assistant via MQTT.

## What It Does

Monitors electrical current on your washer and dryer to detect when they're running and when they finish. Sends real-time status updates to Home Assistant so you can see at a glance whether your laundry is done.

## Features

- **Real-time monitoring** - Checks current every second
- **Smart detection** - Filters out brief voltage spikes (5-second confirmation before marking as "running")
- **Automatic notifications** - Sends alerts when cycles complete
- **Home Assistant integration** - Publishes status via MQTT for dashboard display
- **Particle Cloud integration** - Also publishes events to Particle Cloud

## Hardware Requirements

- Particle Photon
- 2x ACS712 current sensors (one for washer, one for dryer)
- Connections:
  - Washer sensor → Pin A0
  - Dryer sensor → Pin A1

## Software Requirements

- Particle MQTT library (install via Particle Web IDE)
- MQTT broker (e.g., Mosquitto running on Home Assistant)

## Configuration

### Device Settings

Edit these constants in the code if needed:

```cpp
const int washerVoltageThreshold = 5;  // Voltage threshold for washer detection
const int dryerVoltageThreshold = 5;   // Voltage threshold for dryer detection
const int washerAlertThreshold = 300;  // Seconds to wait before "done" alert (5 min)
const int dryerAlertThreshold = 10;    // Seconds to wait before "done" alert
```

### MQTT Broker

Update the IP address if your MQTT broker is different:

```cpp
MQTT client("192.168.2.162", 1883, callback);
```

### Home Assistant Configuration

Add to your `configuration.yaml`:

```yaml
mqtt:
  sensor:
    - name: "Washer"
      state_topic: "homeassistant/sensor/washer/state"
      icon: mdi:washing-machine
      
    - name: "Dryer"
      state_topic: "homeassistant/sensor/dryer/state"
      icon: mdi:tumble-dryer
```

Restart Home Assistant after adding the configuration.

## MQTT Topics

- `homeassistant/sensor/washer/state` - Publishes "running" or "idle"
- `homeassistant/sensor/dryer/state` - Publishes "running" or "idle"

## How It Works

1. **Startup** - Calibrates current sensors to determine baseline voltage
2. **Monitoring Loop** (every 1 second):
   - Reads current from both sensors
   - If current exceeds threshold for 5 seconds → marks as "running" and publishes to MQTT
   - If current drops below threshold while running → starts countdown timer
   - When countdown reaches zero → marks as "idle", sends notification, and publishes to MQTT

3. **State Changes**:
   - Washer: 5 minutes of no current before marking as done
   - Dryer: 10 seconds of no current before marking as done

## Installation

1. Open the Particle Web IDE
2. Add the MQTT library to your project
3. Flash LaundryMonitor.ino to your Particle Photon
4. Configure Home Assistant as described above
5. Add the sensors to your Home Assistant dashboard

## Troubleshooting

- **No MQTT messages**: Check that your MQTT broker is running and accessible at the configured IP
- **False triggers**: Adjust voltage thresholds or wait times in the configuration
- **Sensors not calibrating**: Check wiring to A0 and A1 pins

## License

Open source - modify as needed for your setup.
