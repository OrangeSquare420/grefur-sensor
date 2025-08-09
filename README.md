# Grefur Sensor – Open Hardware IoT Sensor Platform

This is the main repository for **Grefur Sensor** project – an open-source, modular sensor platform designed for flexibility, extensibility, and easy integration into MQTT-based IoT ecosystems. Whether you're building environmental sensors, automation modules, or simple input/output devices – **Grefur's backplate** is the universal starting point.

---

## 🔧 Backplate – The Heart of the System

The **Grefur Backplate** will be standardized, with posibilites to be mounted on walls, cases, HVAC, outdoor +++:
   
- **Sensor Extension**: Any sensor module schould be design for easy fit on the backplate
- **Technology**: With ESP32 and ESP8622, the backplate is full of possibilites
- **Mentality**: Easy access for exposing measured values into the IoT enviroment


> Developed and maintained by **Grefur AS**, the Backplate Specification is open and public. Anyone can design compatible sensors and modules.

---

## Sensor Modules – Build Your Own

Anyone can design a **Grefur-compatible sensor** that fits the backplate. The sensors will the be:

- **Self-contained**: Include firmware, sensor logic, and configuration.  
- **Network-ready**: Designed to connect via Wi-Fi or Ethernet and speak MQTT.  
- **Configurable**: Receive settings remotely and publish sensor readings to any MQTT broker.  
- **Pluggable**: Easily swap modules while maintaining the same infrastructure.  

Example modules include:
- Temperature & Humidity sensors  
- Leak detectors  
- Soil moisture readers  
- Light sensors  
- Digital buttons and switches  

---

## MQTT Broker Integration

All Grefur sensors are designed to publish and receive messages through an MQTT broker. This allows seamless integration with:

- Home automation systems (Home Assistant, OpenHAB, etc.)
- Industrial control systems
- Cloud IoT dashboards

> MQTT topics and payloads are customizable to fit any integration scenario.

---

## Open Source

The **Grefur Sensor Platform** is fully open source:
- Hardware specifications
- Firmware templates
- Config tools and APIs

Join the community to create your own modules or contribute to the ecosystem!

---

## License

This project is licensed under the **MIT License**. You're free to use, modify, and distribute – just give credit.

---

Made with ❤️ by [Grefur AS](https://grefur.com)  


