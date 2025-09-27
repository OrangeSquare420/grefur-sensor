# Grefur Sensor – Open Hardware IoT Sensor Platform

This is the main repository for **Grefur Sensor** project – an open-source, modular sensor platform designed for flexibility, extensibility, and easy integration into MQTT-based IoT ecosystems. Whether you're building environmental sensors, automation modules, or simple input/output devices – **Grefur's backbone** is the universal starting point.

---

## Backplate – The Heart of the System

The standardized **Grefur Backplate is designed** for universal mounting on walls, cases, HVAC systems, outdoor installations, and more. This versatile platform enables:
   
- **Modular Sensor Expansion**: Any sensor module can be designed for easy integration with the backplate and into your enviroment
- **Technology Agnostic**: Compatible with multiple microcontroller platforms including ESP8266, ESP32, and more
- **IoT-First Mindset**: Simplified access for exposing measured values into IoT environments


> Developed and maintained by **Grefur AS**, the Backplate Specification is open and public. Anyone can design modules.

---

## Sensor Modules – Build Your Own

Anyone can design a **Grefur-Backplate compatible module** which will be:

- **Self-contained**: Includes firmware, sensor logic, and configuration.  
- **Network-ready**: Connects via Wi-Fi or Ethernet and communicates via MQTT.  
- **Configurable**: Receive settings remotely and publish sensor readings to any MQTT broker.  
- **Pluggable**: Easily swappable while maintaining the same infrastructure.  

**Key sensor data for the IoT ecosystem**
- Temperature & Humidity sensors  
- Leak detectors  
- Soil moisture sensors
- Light sensors  
- Digital buttons and switches
---

<img width="1793" height="942" alt="image" src="https://github.com/user-attachments/assets/32b37591-f0d9-49dc-9605-c12f301d6dc5" />



## MQTT Broker Integration

> MQTT is a lightweight, publish–subscribe, machine-to-machine network protocol for message queue/message queuing service. It is designed for connections with remote locations that have devices with resource constraints or limited network bandwidth. MQTT topics and payloads are customizable to fit any integration scenario.

All Grefur sensors are designed to publish and receive messages through an MQTT broker. This allows seamless integration with:

- Home automation systems (Home Assistant, OpenHAB, etc.)
- Industrial control systems
- IoT Datalakes and dashboards

---

<img width="1207" height="1034" alt="image" src="https://github.com/user-attachments/assets/7cd7d096-3d32-4b39-bc0d-0f12ac894096" />



## Open Source

The **Grefur Sensor Platform** is fully open source:
- Hardware specifications
- Firmware templates
- Config tools and APIs

Join the community to create your own modules or contribute to the ecosystem!

---

## General Specifications

This system supports connection of a **multiplexer (MUX) unit** to read multiple analog channels through the ESP8266.  

### Features
- Supports up to **4 analog channels** via the MUX.
- **MUX control pins:**
  - **A (select line 1):** GPIO2
  - **B (select line 2):** GPIO14
- Enables reading multiple analog signals with a single analog input pin on the microcontroller.
- Simple integration with ESP8266-based boards like NodeMCU.

### Usage
1. Connect the MUX control pins to the specified GPIOs:
   - `A` → GPIO2
   - `B` → GPIO14
2. Connect the MUX output to the analog input pin of the ESP8266.
3. Use a suitable code library or function to select channels A/B and read analog values.

### Notes
- Ensure GPIO2 and GPIO14 states do not conflict with boot requirements of the ESP8266.
- Only one analog input pin on the ESP8266 is needed; the MUX handles channel selection.


---

## License

This project is licensed under the **MIT License**. You're free to use, modify, and distribute – just give credit.

---

Made with ❤️ by [Grefur AS](https://grefur.com)  


