#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <math.h>
#include <ArduinoJson.h>
#include <ESP8266mDNS.h>


#define EEPROM_SIZE 4096  // Pre-defined eeprom size

#define ONBOARD_LED LED_BUILTIN
#define ONBOARD_LED_ON  LOW   // ACTIVE on LOW
#define ONBOARD_LED_OFF HIGH  // LED OFF

#define CONFIG_START 0
#define CONFIG_SIZE sizeof(Config)
#define SENSOR_CONFIG_START (CONFIG_START + CONFIG_SIZE)

#define SENSOR_CONFIG_SIZE (EEPROM_SIZE - SENSOR_CONFIG_START) / MAX_SENSORS
#define SENSOR_NAME_MAX_LENGTH 16

#define MAX_SENSORS_ALLOWED 6

#define DEBUG true
#define DEBUG_ERROR_VISUALIZATION true


#define MUX_A 2
#define MUX_B 14

unsigned long lastPublishTime = 0;
bool winkActive = false;
bool activeError = true;

constexpr char FIRMWARE_VERSION[] = "0.2.3a";

// x.y.z.o
// x -> New hardware
// y -> New software
// z -> Changes
// o -> Operation state Ex alpha, beta, prod




/**
   Default admin:
    Username: admin
    Password: admin

   Running:
    While available wifi:
      Connecting to access point
      Load settings
      Starts broadcasting to broker
      Settings can be canged at given IP adress
    While not available wifi:
      Creates its own access point
      Load settings if existing
      Access it with connecting to its wifi endpoint
      Change settings for wifi and mqtt
      Save and restart module

     Version log:
      0_0_2:
        Firmware over internet
        Introduction of version control

*/

String generateAPName() {
  uint32_t chipId = ESP.getChipId();
  return "Grefur_" + String(chipId, HEX);
}

#pragma pack(push, 1) // Exact byte alignment

/******************************
          HEADER SECTION
 ******************************/
struct EEPROMHeader {
  uint32_t magic = 0xAA55AA55;
  char version[16] = {};
  uint8_t checksum;
  uint8_t _reserved[10];
};

/******************************
       DEVICE METADATA
 ******************************/
struct DeviceMetadata {
  char apName[32];              // Generate this during initialization
  char deviceType[16] = "Backplate";
  char serialNum[32];           // Populated during init
  char deviceName[32] = "Enhetsnavn";
  char location[32] = "Lokasjon";
  char adminUser[16] = "admin";
  char adminPass[16] = "admin";
};

/******************************
      NETWORK SETTINGS
 ******************************/
struct NetworkSettings {
  char wifiSSID[32];
  char wifiPassword[32];
  uint8_t _ethernet[16];       // Reserved
};

/******************************
         MQTT CONFIG
 ******************************/
struct MQTTConfig {
  char brokerURL[64];
  uint16_t port = 1883;
  char clientID[32];
  char username[32];
  char password[32];
  char baseTopic[64];
  uint8_t qos = 1;
  bool retain = true;
  bool useNestedJson = true;
  uint8_t _reserved[3];         // Padding
};

/******************************
      OPERATION SETTINGS
 ******************************/
struct OperationSettings {
  uint32_t publishInterval = 60000;
  float minValidTemp = -50.0;
  float maxValidTemp = 120.0;
  bool publishMeasuredValues = true;
  bool publishIP = true;
  bool publishStatus = true;
  bool publishDeviceInfo = true;
  bool publishRSSI = false;
  bool publishWatchdog = true;
  uint8_t _reserved[1];         // Padding
};

/******************************
      SENSOR CONFIGURATION
 ******************************/
struct SensorConfig {
  int8_t type = -1;                   // Sensor type identifier
  char name[16];                 // User-assigned name
  union {
    struct {
      float seriesResistor;      // Ω (Ohms)
      float nominalResistance;   // Ω (Ohms)
      float nominalTemp;         // °C (Celsius)
      float bCoefficient;        // K (Kelvin)
      int analogPin;
      char unit[8];              // Temperature unit (default Celsius)
    } ntc;
    struct {
      int analogPin;
      float inputMin;            // V (Volts)
      float inputMax;            // V (Volts)
      float scaleMin;            // Minimum scaled value
      float scaleMax;            // Maximum scaled value
      char scaleUnit[8];         // Unit for scaled values (e.g., "bar", "kPa")
    } analog;
    struct {
      int digitalPin;
      bool inverted;
    } digital;
  };
  float lastMeasurement;         // runtime-only value
  bool publishOnlyChange;
  uint8_t _reserved[27];        // Reduced reserved space to accommodate new fields
};

enum SensorType : int8_t {
  SENSOR_UNCONFIGURED = -1,
  SENSOR_NTC = 0,               // Temperature sensor
  SENSOR_ANALOG,                // Analog voltage/current sensor
  SENSOR_DIGITAL                // Digital on/off sensor
};



/******************************
      CALIBRATION DATA
 ******************************/
struct CalibrationData {
  float tempOffset;
  uint8_t _reserved[28];       // Padding
};

/******************************
    COMPLETE EEPROM LAYOUT
 ******************************/
struct EEPROMLayout {
  EEPROMHeader header;
  DeviceMetadata device;
  NetworkSettings network;
  MQTTConfig mqtt;
  OperationSettings operation;
  SensorConfig sensors[MAX_SENSORS_ALLOWED];
  CalibrationData calibration;
};

#pragma pack(pop) // Restore default packing

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer server(80);

float lastValidTemp = 0;
unsigned long lastPublish = 0;
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000;


EEPROMLayout config;  // Global configuration object

void testConfigIntegrity() {
  Serial.println("Testing config integrity...");

  // Check if config is loaded
  if (config.header.magic != 0xAA55AA55) {
    Serial.println("Config not loaded or invalid!");
    return;
  }

  Serial.printf("Firmware version: '%s'\n", config.header.version);
  Serial.printf("Device name: '%s'\n", config.device.deviceName);
  Serial.printf("WiFi SSID: '%s'\n", config.network.wifiSSID);
  // Add more fields as needed
}

uint8_t calculateChecksum(const uint8_t* data, size_t size) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < size; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Helper functions for EEPROM storage
bool saveConfig() {
  // Use the global config object instead of creating a new one

  // Update checksum (temporarily clear it for calculation)
  config.header.checksum = 0;
  config.header.checksum = calculateChecksum((uint8_t*)&config, sizeof(config));

  EEPROM.begin(sizeof(EEPROMLayout));
  EEPROM.put(0, config);  // Save the GLOBAL config
  if (!EEPROM.commit()) {
    Serial.println("EEPROM commit failed!");
    return false;
  } else {
    EEPROM.end();
    Serial.println("Config saved successfully");
    return true;
  }
}

bool loadConfig() {
  EEPROM.begin(sizeof(EEPROMLayout));
  EEPROM.get(0, config);  // Load into GLOBAL config
  EEPROM.end();

  // Validate
  if (config.header.magic != 0xAA55AA55) {
    Serial.println("Invalid magic number - using default config");
    return false;
  }

  uint8_t saved_checksum = config.header.checksum;
  config.header.checksum = 0; // Reset before calculating

  if (calculateChecksum((uint8_t*)&config, sizeof(config)) != saved_checksum) {
    Serial.println("Checksum mismatch - config may be corrupted");
    return false;
  }

  // Restore the checksum
  config.header.checksum = saved_checksum;

  Serial.println("Config loaded successfully");
  return true;
}

/* Middelware / Logic between the server and the client,
 *  */

/**
  BASIC HTTP AUTH
  Verify correct username and password
*/
bool isAuthenticated() {
  if (strlen(config.device.adminPass) == 0) return true;
  return server.authenticate("admin", config.device.adminPass);
}


String getFirmwareVersion() {
  return String(FIRMWARE_VERSION);
}



// Middleware to check authentication
bool authenticateRequest() {
  if (!isAuthenticated()) {
    server.send(401, "text/plain", "Unauthorized");
    return false;
  }
  return true;
}

/*
   @brief check if new pin in use, then send warning
*/

bool isPinUsed(uint8_t pin) {
  for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
    if (config.sensors[i].type == SENSOR_NTC && config.sensors[i].ntc.analogPin == pin) return true;
    if (config.sensors[i].type == SENSOR_ANALOG && config.sensors[i].analog.analogPin == pin) return true;
    if (config.sensors[i].type == SENSOR_DIGITAL && config.sensors[i].digital.digitalPin == pin) return true;
  }
  return false;
}

/*
  // Smart serialiseringsmakroer
  #define SERIALIZE_STRING(doc, parent, field) parent[#field] = config.field
  #define SERIALIZE_INT(doc, parent, field) parent[#field] = config.field
  #define SERIALIZE_FLOAT(doc, parent, field) parent[#field] = config.field
  #define SERIALIZE_BOOL(doc, parent, field) parent[#field] = config.field

  #define DESERIALIZE_STRING(doc, parent, field) \
  if (doc[#field]) strlcpy(config.field, doc[#field], sizeof(config.field))
  #define DESERIALIZE_INT(doc, parent, field) \
  if (doc[#field]) config.field = doc[#field]
  #define DESERIALIZE_FLOAT(doc, parent, field) \
  if (doc[#field]) config.field = doc[#field]
  #define DESERIALIZE_BOOL(doc, parent, field) \
  if (doc[#field]) config.field = doc[#field]

  // Sensor type håndtering
  const char* sensorTypeToString(int8_t type) {
  switch (type) {
    case SENSOR_NTC: return "NTC";
    case SENSOR_ANALOG: return "ANALOG";
    case SENSOR_DIGITAL: return "DIGITAL";
    default: return "UNCONFIGURED";
  }
  }

  int8_t stringToSensorType(const char* typeStr) {
  if (strcmp(typeStr, "NTC") == 0) return SENSOR_NTC;
  if (strcmp(typeStr, "ANALOG") == 0) return SENSOR_ANALOG;
  if (strcmp(typeStr, "DIGITAL") == 0) return SENSOR_DIGITAL;
  return SENSOR_UNCONFIGURED;
  }
*/


/*
   @ brief: Handler for gateway parametersavings. All parameters for connections
*/

class SaveConfigManager {
  public:
    SaveConfigManager(EEPROMLayout& config) : _config(config), _changed(false) {}

    // Middleware to check authentication
    bool authenticateRequest() {
      if (!isAuthenticated()) {
        server.send(401, "text/plain", "Unauthorized");
        return false;
      }
      return true;
    }

    // Update methods
    bool updateStringField(char* dest, size_t destSize, const String& newValue) {
      if (newValue.length() > 0 && strcmp(dest, newValue.c_str()) != 0) {
        strncpy(dest, newValue.c_str(), destSize);
        dest[destSize - 1] = '\0';
        return (_changed = true);
      }
      return false;
    }

    template <typename T>
    bool updateNumericField(T& dest, const String& newValue) {
      if (newValue.length() > 0) {
        T newVal;
        if constexpr (std::is_same_v<T, int>) newVal = newValue.toInt();
        else if constexpr (std::is_same_v<T, long>) newVal = newValue.toInt();  // toInt dekker long også
        else if constexpr (std::is_same_v<T, int8_t>) newVal = static_cast<int8_t>(newValue.toInt());
        else if constexpr (std::is_same_v<T, float>) newVal = newValue.toFloat();
        else if constexpr (std::is_same_v<T, uint32_t>) newVal = static_cast<uint32_t>(newValue.toInt());
        else if constexpr (std::is_same_v<T, uint16_t>) newVal = static_cast<uint16_t>(newValue.toInt());
        else if constexpr (std::is_same_v<T, uint8_t>) newVal = static_cast<uint8_t>(newValue.toInt());
        else {
          if (DEBUG) {
            Serial.println("Given type is not supported");
          }
          return false;
        }

        if (dest != newVal) {
          dest = newVal;
          if (DEBUG) {
            Serial.print("Value updated to: ");
            Serial.println(newVal);
          }
          return (_changed = true);
        } else {
          if (DEBUG) {
            Serial.println("Value is the same, no update needed");
          }
        }
      } else {
        if (DEBUG) {
          Serial.println("Given value has zero length");
        }
      }
      return false;
    }




    bool updateUint16Field(uint16_t& dest, const String& newValue) {
      if (newValue.length() > 0) {
        uint16_t newVal = static_cast<uint16_t>(newValue.toInt());
        if (dest != newVal) {
          dest = newVal;
          return (_changed = true);
        }
      }
      return false;
    }



    bool updateBoolField(bool& dest, bool newValue) {
      if (dest != newValue) {
        dest = newValue;
        return (_changed = true);
      }
      return false;
    }

    // This triggers restart if called before finalize
    void scheduleRestart() {
      _shouldRestart = true;
    }

    void finalize(const String& successMessage = "Settings updated") {
      if (_changed) {
        saveConfig();
        if (_shouldRestart) {
          server.send(200, "text/plain", successMessage + ". Server restarting...");
        } else {
          server.send(200, "text/plain", successMessage);
        }
      } else {
        server.send(200, "text/plain", "No changes detected");
      }

      // Delay restart to ensure response is sent
      if (_shouldRestart) {
        server.client().stop();  // Ensure client disconnects
        delay(500);              // Brief delay to ensure response is sent
        ESP.restart();           // Perform the restart
      }

      _changed = false;         // Reset for next use
      _shouldRestart = false;   // Clear restart flag
    }

    bool hasChanges() const {
      return _changed;
    }

  private:
    EEPROMLayout& _config;
    bool _changed;
    bool _shouldRestart;  // New flag for restart scheduling
};;





// Add this helper function
void printWiFiStatus() {
  switch (WiFi.status()) {
    case WL_IDLE_STATUS:    Serial.println("Idle"); break;
    case WL_NO_SSID_AVAIL:  Serial.println("SSID not found"); break;
    case WL_CONNECT_FAILED: Serial.println("Wrong password"); break;
    case WL_CONNECTION_LOST: Serial.println("Connection lost"); break;
    case WL_DISCONNECTED:   Serial.println("Disconnected"); break;
    default:                Serial.printf("Error %d\n", WiFi.status());
  }
}

void printEEPROMStatus() {
  Serial.println("\nEEPROM Status:");
  for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
    Serial.printf("Sensor %d: type=%d, name='%s'\n",
                  i, config.sensors[i].type, config.sensors[i].name);
  }
}

/**
   @brief Sensor Manager - Handles sensor configuration and measurements

   Provides functionality for:
   - Adding/removing sensors of different types (NTC, Analog, Digital)
   - Storing/loading sensor configurations to/from EEPROM
   - Taking measurements with automatic scaling and validation
   - Generating UI metadata for sensor configuration forms

   Supported sensor types:
   1. NTC Thermistors (temperature)
   2. Analog sensors (voltage/current)
   3. Digital sensors (on/off)

   All configurations are stored in the EEPROM structure and maintain
   checksum validation with the existing system.

   Usage:
   1. Initialize with global config reference
   2. Call addSensor() with configuration
   3. Use readSensor() to get measurements
   4. Access configuration via getSensorConfig()
*/


class SensorManager {
  private:
    EEPROMLayout& _config;
    bool _changed;

  public:
    SensorManager(EEPROMLayout& config) : _config(config), _changed(false) {}

    // Get available sensor types
    String retrieveSensorTypes() {
      DynamicJsonDocument doc(256);
      JsonArray types = doc.createNestedArray("types");

      types.add("NTC");
      types.add("Analog");
      types.add("Digital");

      String output;
      serializeJson(doc, output);
      return output;
    }

    // Get fields for a specific sensor type
    String retrieveSensorFieldsAsJson(int8_t type) {
      DynamicJsonDocument doc(512);
      JsonObject fields = doc.createNestedObject("fields");
      JsonObject labels = doc.createNestedObject("labels");
      JsonObject units = doc.createNestedObject("units");
      JsonObject placeholders = doc.createNestedObject("placeholders");

      switch (type) {
        case SENSOR_NTC:
          doc["type"] = "NTC";

          // Series Resistor
          fields["seriesResistor"] = "number";
          labels["seriesResistor"] = "Series Resistor";
          units["seriesResistor"] = "Ω";
          placeholders["seriesResistor"] = "e.g., 10000";

          // Nominal Resistance
          fields["nominalResistance"] = "number";
          labels["nominalResistance"] = "Nominal Resistance";
          units["nominalResistance"] = "Ω";
          placeholders["nominalResistance"] = "e.g., 10000";

          // Nominal Temperature
          fields["nominalTemp"] = "number";
          labels["nominalTemp"] = "Nominal Temperature";
          units["nominalTemp"] = "°C";
          placeholders["nominalTemp"] = "e.g., 25.0";

          // B Coefficient
          fields["bCoefficient"] = "number";
          labels["bCoefficient"] = "B Coefficient";
          units["bCoefficient"] = "";
          placeholders["bCoefficient"] = "e.g., 3950";

          fields["analogPin"] = "number";
          labels["analogPin"] = "Connected Input";
          units["analogPin"] = "";
          placeholders["analogPin"] = "e.g., A0";

          fields["unit"] = "text";
          labels["unit"] = "Temperature unit";
          units["unit"] = "";
          placeholders["unit"] = "e.g., °C";

          break;

        case SENSOR_ANALOG:
          doc["type"] = "Analog";

          // Analog Pin
          fields["analogPin"] = "number";
          labels["analogPin"] = "Analog Pin";
          units["analogPin"] = "";
          placeholders["analogPin"] = "e.g., A0";

          // Input Minimum
          fields["inputMin"] = "number";
          labels["inputMin"] = "Min Voltage";
          units["inputMin"] = "V";
          placeholders["inputMin"] = "e.g., 0.0";

          // Input Maximum
          fields["inputMax"] = "number";
          labels["inputMax"] = "Max Voltage";
          units["inputMax"] = "V";
          placeholders["inputMax"] = "e.g., 3.3";

          // Scale Minimum
          fields["scaleMin"] = "number";
          labels["scaleMin"] = "Scale Min";
          units["scaleMin"] = "";
          placeholders["scaleMin"] = "e.g., 0";

          // Scale Maximum
          fields["scaleMax"] = "number";
          labels["scaleMax"] = "Scale Max";
          units["scaleMax"] = "";
          placeholders["scaleMax"] = "e.g., 100";

          fields["scaleUnit"] = "text";
          labels["scaleUnit"] = "Scaled unit";
          units["scaleUnit"] = "";
          placeholders["scaleUnit"] = "e.g., kPa";

          break;

        case SENSOR_DIGITAL:
          doc["type"] = "Digital";

          // Digital Pin
          fields["digitalPin"] = "number";
          labels["digitalPin"] = "Digital Pin";
          units["digitalPin"] = "";
          placeholders["digitalPin"] = "e.g., 5";

          // Inverted Logic
          fields["inverted"] = "boolean";
          labels["inverted"] = "Inverted Logic";
          units["inverted"] = "";
          placeholders["inverted"] = "";
          break;
      }

      String output;
      serializeJson(doc, output);
      return output;
    }


    // Get all configured sensors
    String retrieveAllSensors(bool returnEverySensor = false) {
      DynamicJsonDocument doc(1024);
      JsonArray sensors = doc.createNestedArray("sensors");

      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        // Key change: Check both type AND name to identify truly unconfigured sensors
        bool isConfigured = (_config.sensors[i].type != 0) ||
                            (_config.sensors[i].type == 0 && strlen(_config.sensors[i].name) > 0);

        if (returnEverySensor || isConfigured) {
          JsonObject sensor = sensors.createNestedObject();
          sensor["id"] = i;
          sensor["type"] = static_cast<int8_t>(_config.sensors[i].type);
          sensor["name"] = _config.sensors[i].name;
          sensor["configured"] = isConfigured;

          // Add type-specific details for configured sensors
          if (isConfigured) {
            switch (_config.sensors[i].type) {
              case SENSOR_NTC:
                sensor["details"] = String(_config.sensors[i].ntc.nominalResistance) + "Ω";
                break;
              case SENSOR_ANALOG:
                sensor["details"] = "Pin A" + String(_config.sensors[i].analog.analogPin);
                break;
              case SENSOR_DIGITAL:
                sensor["details"] = "GPIO" + String(_config.sensors[i].digital.digitalPin);
                break;
              default:
                if (_config.sensors[i].type == 0) {
                  sensor["details"] = "Legacy NTC";  // Handle case where NTC is type 0
                }
            }
          } else {
            sensor["status"] = "unconfigured";
          }
        }
      }

      String output;
      serializeJson(doc, output);
      return output;
    }

    // Add a new sensor
    int addSensor(const SensorConfig& newSensor) {
      Serial.println("\nSearching for empty slots...");

      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        Serial.printf("Checking for slot %d: type=%d, name='%s'\n",
                      i, _config.sensors[i].type, _config.sensors[i].name);

        // Utvidet sjekk for ledige plasser
        if (_config.sensors[i].type == SENSOR_UNCONFIGURED ||
            (_config.sensors[i].type == 0 && strlen(_config.sensors[i].name) == 0)) {

          // Full kopiering med memcpy for å unngå medlemsvis kopiering
          memcpy(&_config.sensors[i], &newSensor, sizeof(SensorConfig));
          _changed = true;

          if (DEBUG) {
            Serial.println("Added new sensor:");
            Serial.printf("- Type: %d\n", _config.sensors[i].type);
            Serial.printf("- Navn: '%s'\n", _config.sensors[i].name);

            if (_config.sensors[i].type == 0) { // NTC-sensor
              Serial.printf("- Enhet: '%s'\n", _config.sensors[i].ntc.unit);
            }
          }
          return i;
        }
      }

      if (DEBUG) {
        Serial.println("No empty space found - expected empty slot");
      }

      printAllSensorSlots(); // Hjelpefunksjon for debugging
      return -1;
    }

    // Remove a sensor
    bool removeSensor(int id) {
      if (id < 0 || id >= MAX_SENSORS_ALLOWED) {
        Serial.printf("Invalid sensor ID %d\n", id);
        return false;
      }

      if (DEBUG) {
        Serial.printf("Deleting sensor %d - Before:\n", id);
        Serial.printf("Type: %d, Name: '%s'\n",
                      _config.sensors[id].type,
                      _config.sensors[id].name);
      }

      // Full reset
      SensorConfig blankConfig = {};
      blankConfig.type = SENSOR_UNCONFIGURED; // Explicitly set to -1
      memset(blankConfig.name, 0, sizeof(blankConfig.name));
      _config.sensors[id] = blankConfig;
      _changed = true;

      if (DEBUG) {
        Serial.printf("After deletion:\n");
        Serial.printf("Type: %d, Name: '%s'\n",
                      _config.sensors[id].type,
                      _config.sensors[id].name);
      }

      return true;
    }

    // Check if any sensor is configured
    bool isSensorConfigured() const {
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        if (_config.sensors[i].type < 0) {
          return true;
        }
      }
      return false;
    }

    // Read sensor value based on configured type
    float readSensor(int sensorIndex) {
      if (sensorIndex < 0 || sensorIndex >= MAX_SENSORS_ALLOWED) {
        return NAN;
      }

      switch (_config.sensors[sensorIndex].type) {
        case SENSOR_NTC:
          return readTemperature(sensorIndex);
        case SENSOR_ANALOG:
          return readVoltage(sensorIndex);
        case SENSOR_DIGITAL:
          return readDigital(sensorIndex) ? 1.0f : 0.0f;
        default:
          return NAN;
      }
    }


    // Get current sensor configuration
    const SensorConfig& getSensorConfig(int index) const {
      if (index >= 0 && index < MAX_SENSORS_ALLOWED) {
        return _config.sensors[index];
      }
      static SensorConfig emptyConfig = {0};
      return emptyConfig;
    }

    // Check if there are unsaved changes
    bool hasChanges() const {
      return _changed;
    }

    // Save changes to EEPROM
    bool saveChanges() {
      if (_changed) {
        Serial.println("Saving sensor changes to configuration...");
        _changed = false; // Reset first to prevent recursion

        // Use the global saveConfig() function
        if (saveConfig()) {
          Serial.println("Sensor changes successfully saved");
          return true;
        }

        Serial.println("Failed to save sensor changes");
        _changed = true; // Restore changed flag if save failed
        return false;
      }
      return true; // No changes to save
    }

    int getConfiguredSensorCount() const {
      int count = 0;
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        if (_config.sensors[i].type >= 0) count++;
      }
      return count;
    }

    void printSensorStates() {
      Serial.println("\nCurrent Sensor States:");
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        Serial.printf("Sensor %d: type=%d, name='%s'\n",
                      i, config.sensors[i].type, config.sensors[i].name);
      }
    }

    // Use when reset all slots
    void initializeSensorSlots() {
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        _config.sensors[i].type = SENSOR_UNCONFIGURED;
        memset(_config.sensors[i].name, 0, sizeof(_config.sensors[i].name));
      }
      _changed = true;
    }

    void printAllSensorSlots() {
      Serial.println("\nAlle sensorplasser:");
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        Serial.printf("[%d] type=%d, name='%s'",
                      i, _config.sensors[i].type, _config.sensors[i].name);

        if (_config.sensors[i].type == 0) { // NTC
          Serial.printf(", unit='%s'", _config.sensors[i].ntc.unit);
        }
        Serial.println();
      }
    }

    String getSensorUnit(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return "";  // Ugyldig indeks
      }

      switch (_config.sensors[index].type) {
        case SENSOR_NTC:
          // Enheten ligger i ntc.unit (f.eks "°C")
          return String(_config.sensors[index].ntc.unit);

        case SENSOR_ANALOG:
          // Enheten er i analog.scaleUnit
          return String(_config.sensors[index].analog.scaleUnit);

        case SENSOR_DIGITAL:
          // Digital sensorer har som regel ikke enhet, returner tom streng
          return "";

        default:
          return "";
      }
    }

    String getSensorName(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return "";  // Ugyldig indeks
      }

      return _config.sensors[index].name;
    }

    bool sameValueLastTime(int index, float value) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return false;  // Invalid index
      }

      return _config.sensors[index].lastMeasurement == value;
    }

    bool isPublishOnlyOnChange(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return false; // sikkerhets-sjekk
      }
      return _config.sensors[index].publishOnlyChange;
    }


    void setLastMeasurement(int index, float value) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) return;
      _config.sensors[index].lastMeasurement = value;
    }




  private:


    // read analog values using the multiplexer
    float analogReadMux(int sensorIndex) {
      if (DEBUG) {
        Serial.println("Read channel " + String(sensorIndex) + " using multiplexer...");
      }

      // Hent hvilken mux-kanal sensoren bruker
      int muxIndex = _config.sensors[sensorIndex].analog.analogPin; // 0..3

      Serial.println("Analog pin:" + String(_config.sensors[sensorIndex].analog.analogPin));
      Serial.println("Mux index:" + String(muxIndex));


      bool MUX_A_STATE = (muxIndex & 0b01) != 0; // Sjekk om bit 0 er satt
      bool MUX_B_STATE = (muxIndex & 0b10) != 0; // Sjekk om bit 1 er satt

      // Sett mux-kanal
      digitalWrite(MUX_A, MUX_A_STATE);  // MSB
      digitalWrite(MUX_B, MUX_B_STATE);         // LSB
      Serial.println("PIN A:" + String(MUX_A_STATE) + "| PIN B:" + String(MUX_B_STATE));


      delay(7); // stabilisation

      int readValue = analogRead(A0); // mux output read

      digitalWrite(MUX_A, LOW);
      digitalWrite(MUX_B, LOW);

      return (float)readValue;
    }




    float fmap(float x, float inMin, float inMax, float outMin, float outMax) {
      return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }


    float readTemperature(int index) {
      float adcValue = analogReadMux(index);
      float R = _config.sensors[index].ntc.seriesResistor / ((1023.0 / adcValue) - 1.0);
      return 1.0 / (log(R / _config.sensors[index].ntc.nominalResistance) /
                    _config.sensors[index].ntc.bCoefficient +
                    1.0 / (_config.sensors[index].ntc.nominalTemp + 273.15)) - 273.15;
    }

    float readVoltage(int index) {
      float raw = analogReadMux(index);
      float voltage = raw * (3.3 / 1023.0);
      return fmap(voltage,
                  _config.sensors[index].analog.inputMin,
                  _config.sensors[index].analog.inputMax,
                  _config.sensors[index].analog.scaleMin,
                  _config.sensors[index].analog.scaleMax);
    }

    bool readDigital(int index) {
      bool val = digitalRead(_config.sensors[index].digital.digitalPin);
      return _config.sensors[index].digital.inverted ? !val : val;
    }
};

SensorManager sensorManager(config);  // Global instance




void takeMeasurements(float outValues[]) {
  for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
    outValues[i] = NAN; // default verdi
    if (i < (sizeof(config.sensors) / sizeof(config.sensors[0]))) {
      if (config.sensors[i].type != SENSOR_UNCONFIGURED) {
        outValues[i] = sensorManager.readSensor(i);
      }
    }
  }
}





void checkMeasurementReadiness() {
  if (sensorManager.getConfiguredSensorCount() > 0) {
    Serial.println("System ready for measurements!");
    Serial.print("Configured sensors: ");
    Serial.println(sensorManager.getConfiguredSensorCount());

    // Print all sensor details
    sensorManager.printAllSensorSlots();
  } else {
    Serial.println("No sensors configured - please add sensors first");
  }
}



class BackupManager {
  public:
    // Eksporter hele config som JSON (100% dynamisk)
    static String exportFullConfig() {
      DynamicJsonDocument doc(8192);
      JsonObject root = doc.to<JsonObject>();
      serializeStruct(root, config);

      String output;
      serializeJson(doc, output);
      return output;
    }

    // Importer hele config fra JSON
    static bool importFullConfig(const String& jsonConfig) {
      DynamicJsonDocument doc(8192);
      DeserializationError error = deserializeJson(doc, jsonConfig);
      if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return false;
      }

      JsonObject root = doc.as<JsonObject>();
      deserializeStruct(root, config);
      return true;
    }


    // Backup til EEPROM
    static bool backupToEEPROM() {
      config.header.checksum = calculateChecksum();

      EEPROM.begin(sizeof(EEPROMLayout));
      EEPROM.put(0, config);
      bool success = EEPROM.commit();
      EEPROM.end();

      return success;
    }

    // Gjenopprett fra EEPROM
    static bool restoreFromEEPROM() {
      EEPROM.begin(sizeof(EEPROMLayout));
      EEPROM.get(0, config);
      EEPROM.end();

      return validateChecksum();
    }

    // Hjelpefunksjoner
    static uint8_t calculateChecksum() {
      uint8_t checksum = 0;
      const uint8_t* data = reinterpret_cast<const uint8_t*>(&config);
      size_t dataSize = sizeof(EEPROMLayout) - sizeof(config.header.checksum) - sizeof(config.header._reserved);

      for (size_t i = 0; i < dataSize; i++) {
        checksum ^= data[i];
      }
      return checksum;
    }

    static bool validateChecksum() {
      return config.header.checksum == calculateChecksum();
    }

  private:
    // Generiske serialiseringsfunksjoner
    template<typename T>
    static void serializeField(JsonObject& obj, const char* fieldName, const T& value) {
      obj[fieldName] = value;
    }

    // Spesialisering for char arrays
    template<size_t N>
    static void serializeField(JsonObject& obj, const char* fieldName, const char (&value)[N]) {
      if (strlen(value) > 0) {
        obj[fieldName] = value;
      }
    }

    template<typename T>
    static void deserializeField(const JsonObject& obj, const char* fieldName, T& value) {
      if (obj.containsKey(fieldName)) {
        value = obj[fieldName].as<T>();
      }
    }

    // Spesialisering for char arrays
    template<size_t N>
    static void deserializeField(const JsonObject& obj, const char* fieldName, char (&value)[N]) {
      if (obj.containsKey(fieldName)) {
        const char* str = obj[fieldName].as<const char*>();
        if (str) {
          strncpy(value, str, N - 1);
          value[N - 1] = '\0';
        }
      }
    }

    // Serialiser hele struct som byte-array
    static void serializeStruct(JsonObject& obj, const EEPROMLayout& data) {
      JsonArray byteArray = obj.createNestedArray("data");
      const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data);

      for (size_t i = 0; i < sizeof(EEPROMLayout); i++) {
        byteArray.add(ptr[i]);
      }
    }

    // Deserialiser byte-array tilbake til struct
    static void deserializeStruct(const JsonObject& obj, EEPROMLayout& data) {
      if (!obj.containsKey("data")) return;

      JsonArray byteArray = obj["data"];
      size_t len = min(byteArray.size(), sizeof(EEPROMLayout));
      uint8_t* ptr = reinterpret_cast<uint8_t*>(&data);

      for (size_t i = 0; i < len; i++) {
        ptr[i] = byteArray[i];
      }
    }

};

// Helperfunction to the backup manager
bool importEEPROMBackup(const String &json) {
  DynamicJsonDocument doc(8192);
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }

  if (!doc.containsKey("data")) return false;

  JsonArray arr = doc["data"];
  if (arr.size() != sizeof(EEPROMLayout)) {
    Serial.println("Error: Backup size mismatch");
    return false;
  }

  uint8_t* p = reinterpret_cast<uint8_t*>(&config);
  for (size_t i = 0; i < arr.size(); i++) p[i] = arr[i];

  return BackupManager::validateChecksum();
}

/**
   @brief Error visualization controller with non-blocking LED patterns

   Each error type has a unique blink pattern for easy identification:

   Error Type               Blink Pattern              Total Duration
   ---------------------------------------------------------------
   INVALID_MEASUREMENT      2 quick blinks (300ms)     1.2s
   NO_INTERNET              4 medium blinks (500ms)    4.0s
   NO_BROKER                6 rapid blinks (200ms)     2.4s
   GENERAL_ERROR            3 slow blinks (700ms)      4.2s
   NETWORK_ERROR            3 slow blinks (700ms)      4.2s
   BROKER_ERROR             3 slow blinks (700ms)      4.2s
 * */

void debugPrint(const char* msg) {
  if (DEBUG_ERROR_VISUALIZATION) {
    Serial.println(msg);
  }
}

void debugPrintValue(const char* label, int value) {
  if (DEBUG_ERROR_VISUALIZATION) {
    Serial.print(label);
    Serial.print(": ");
    Serial.println(value);
  }
}

// ---------------- Error types ----------------
enum ErrorType {
  GENERAL_ERROR,
  INVALID_MEASUREMENT,
  NO_INTERNET,
  NO_BROKER,
  NETWORK_ERROR,
  BROKER_ERROR,
  WINK
};

// Forward declarations
void visualizationOfError(ErrorType error);
void updateErrorVisualization();
void debugPrint(const char* msg);
void debugPrintValue(const char* label, int value);

// ---------------- Feiltilstand ----------------
struct ErrorState {
  ErrorType currentError;
  uint8_t blinkCountRemaining;
  uint16_t blinkInterval;
  bool isActive;
  bool blinkState;
  unsigned long lastBlinkTime;
};

ErrorState errorState = {
  GENERAL_ERROR,
  0,
  0,
  false,
  ONBOARD_LED_OFF,
  0
};

// ---------------- Patterns structure ----------------
struct ErrorPattern {
  uint8_t blinkCount;   // sum of ON+OFF
  uint16_t interval;    // time per cycle in ms
};

const ErrorPattern errorPatterns[] = {
  {3, 700},   // GENERAL_ERROR
  {2, 300},   // INVALID_MEASUREMENT
  {4, 500},   // NO_INTERNET
  {6, 200},   // NO_BROKER
  {3, 700},   // NETWORK_ERROR
  {3, 700},   // BROKER_ERROR
  {10, 700},  // WINK
};

// ---------------- Error visualization functions ----------------
void visualizationOfError(ErrorType error) {
  if (winkActive) return;

  // Dont start new cycle if error is active and the same 
  if (errorState.isActive && errorState.currentError == error) {
    return;
  }

  errorState.currentError = error;
  errorState.blinkCountRemaining = errorPatterns[error].blinkCount * 2;
  errorState.blinkInterval = errorPatterns[error].interval;
  errorState.isActive = true;
  errorState.blinkState = ONBOARD_LED_ON;
  errorState.lastBlinkTime = millis();

  digitalWrite(ONBOARD_LED, errorState.blinkState);

  debugPrint("visualizationOfError(): new error triggered");
  debugPrintValue("  error", error);
}

void updateErrorVisualization() {
  if (!errorState.isActive) return;
  if (winkActive) return;

  if (millis() - errorState.lastBlinkTime >= errorState.blinkInterval) {
    errorState.lastBlinkTime = millis();

    errorState.blinkState = (errorState.blinkState == ONBOARD_LED_ON) ? ONBOARD_LED_OFF : ONBOARD_LED_ON;
    digitalWrite(ONBOARD_LED, errorState.blinkState);

    if (errorState.blinkState == ONBOARD_LED_ON) {
      errorState.blinkCountRemaining--;

      if (errorState.blinkCountRemaining <= 0) {
        errorState.isActive = false;
        digitalWrite(ONBOARD_LED, ONBOARD_LED_OFF);
        debugPrint("Pattern finished -> LED OFF");
      }
    }
  }
}








/**
   @brief MQTT module for IoT device communication
*/

/* ======================== */
/*      MQTT Connection     */
/* ======================== */

/* ======================== */
/*      Core Publishing     */
/* ======================== */

/**
   @brief Publish value using nested JSON structure
   @param client MQTT client reference
   @param subtopic Endpoint subtopic
   @param payload JSON payload string
*/
void publishNestedValue(PubSubClient& client, const char* subtopic, const char* payload, const char* apName = nullptr) {
  char topic[128];
  if (apName && strlen(apName) > 0) {
    snprintf(topic, sizeof(topic), "%s/%s/%s", config.mqtt.baseTopic, apName, subtopic);
  } else {
    snprintf(topic, sizeof(topic), "%s/%s", config.mqtt.baseTopic, subtopic);
  }

  if (DEBUG) {
    Serial.print("[DEBUG] publishNestedValue to topic: ");
    Serial.println(topic);
    Serial.print("[DEBUG] Payload: ");
    Serial.println(payload);
  }
  client.publish(topic, payload, config.mqtt.retain);
}


/**
   @brief Publish flat value structure
   @param client MQTT client reference
   @param subtopic Endpoint subtopic
   @param value Raw value string
*/
void publishFlatValue(PubSubClient& client, const char* subtopic, const char* value, const char* apName = nullptr) {
  char topic[128];
  if (apName && strlen(apName) > 0) {
    snprintf(topic, sizeof(topic), "%s/%s/%s/value", config.mqtt.baseTopic, apName, subtopic);
  } else {
    snprintf(topic, sizeof(topic), "%s/%s/value", config.mqtt.baseTopic, subtopic);
  }

  if (DEBUG) {
    Serial.print("[DEBUG] publishFlatValue to topic: ");
    Serial.println(topic);
    Serial.print("[DEBUG] Value: ");
    Serial.println(value);
  }
  client.publish(topic, value, config.mqtt.retain);
}

/* ======================== */
/*    Data Publications     */
/* ======================== */

/**
   @brief Publish measured value with unit
   @param client MQTT client reference
   @param value The measurement value
   @param measurementType Type identifier (e.g. "temperature")
   @param unit Measurement unit (e.g. "°C")
*/
void publishMeasuredValue(PubSubClient& client, float value, const char* sensorName, const char* unit) {
  if (DEBUG) {
    Serial.print("[DEBUG] Publishing to broker - Sensor: ");
    Serial.print(sensorName);
    Serial.print(", Value: ");
    Serial.print(value, 2);
    Serial.print(" ");
    Serial.println(unit);
  }


  if (config.mqtt.useNestedJson) {
    char payload[128];
    snprintf(payload, sizeof(payload), "{\"value\":%.2f,\"unit\":\"%s\"}", value, unit);
    if (DEBUG) {
      Serial.print("[DEBUG] Using nested JSON payload: ");
      Serial.println(payload);
    }

    publishNestedValue(client, sensorName, payload);
  } else {
    char valueStr[16];
    dtostrf(value, 4, 2, valueStr);

    if (DEBUG) {
      Serial.print("[DEBUG] Using flat payload, value string: ");
      Serial.println(valueStr);
    }

    publishFlatValue(client, sensorName, valueStr);
    String topic = String(config.mqtt.baseTopic) + "/" + sensorName + "/unit";
    if (DEBUG) {
      Serial.print("[DEBUG] Publishing unit to topic: ");
      Serial.println(topic);
    }
    client.publish(topic.c_str(), unit, config.mqtt.retain);
  }
}


/**
   @brief Publish device IP address
   @param client MQTT client reference
   @param apName Optional AP name to include in topic
*/
void publishIP(PubSubClient& client, const char* apName = nullptr) {
  String ip = WiFi.localIP().toString();
  if (config.mqtt.useNestedJson) {
    char payload[128];
    snprintf(payload, sizeof(payload), "{\"value\":\"%s\"}", ip.c_str());
    publishNestedValue(client, "ip", payload, apName);
  } else {
    publishFlatValue(client, "ip", ip.c_str(), apName);
  }
}

/**
   @brief Publish device status
   @param client MQTT client reference
   @param status Status message
   @param apName Optional AP name to include in topic
*/
void publishStatus(PubSubClient& client, const char* status, const char* apName = nullptr) {
  if (config.mqtt.useNestedJson) {
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"value\":\"%s\",\"rssi\":%d,\"version\":\"%s\"}",
             status, WiFi.RSSI(), config.header.version);
    publishNestedValue(client, "status", payload, apName);
  } else {
    publishFlatValue(client, "status", status, apName);
    if (config.operation.publishRSSI) {
      char rssiStr[16];
      itoa(WiFi.RSSI(), rssiStr, 10);
      publishFlatValue(client, "status/rssi", rssiStr, apName);
    }
  }
}

/**
   @brief Publish device information
   @param client MQTT client reference
   @param apName Optional AP name to include in topic
*/
void publishDeviceInfo(PubSubClient& client, const char* apName = nullptr) {
  if (config.mqtt.useNestedJson) {
    StaticJsonDocument<512> doc;
    doc["deviceId"] = String(ESP.getChipId(), HEX);
    doc["deviceName"] = config.device.deviceName;
    doc["location"] = config.device.location;
    doc["firmware"] = config.header.version;
    doc["ip"] = WiFi.localIP().toString();
    doc["mac"] = WiFi.macAddress();

    String payload;
    serializeJson(doc, payload);
    publishNestedValue(client, "info", payload.c_str(), apName);
  } else {
    publishFlatValue(client, "info/deviceId", String(ESP.getChipId(), HEX).c_str(), apName);
    publishFlatValue(client, "info/deviceName", config.device.deviceName, apName);
    publishFlatValue(client, "info/location", config.device.location, apName);
    publishFlatValue(client, "info/ip", WiFi.localIP().toString().c_str(), apName);
  }
}

void publishWatchdog(PubSubClient& client, int& lastValue, const char* apName = nullptr) {
  lastValue++;
  if (lastValue > 255) lastValue = 1;

  char payload[128];
  snprintf(payload, sizeof(payload), "{\"value\":\"%d\"}", lastValue);

  if (config.mqtt.useNestedJson) {
    publishNestedValue(client, "watchdog", payload, apName);
  } else {
    publishFlatValue(client, "watchdog", String(lastValue).c_str(), apName);
  }
}





/**
   @brief Attempt MQTT reconnection with backoff
   @param client Reference to PubSubClient instance
*/
void reconnectMQTT(PubSubClient& client) {
  static unsigned long lastReconnectAttempt = 0;
  const unsigned long RECONNECT_INTERVAL = 5000; // 5 seconds between attempts

  // Don't attempt if already connected or too soon
  if (client.connected() || (millis() - lastReconnectAttempt < RECONNECT_INTERVAL)) {
    return;
  }

  lastReconnectAttempt = millis();

  Serial.println("Trying to reconnect to MQTT broker...");

  char apName[32];
  strncpy(apName, generateAPName().c_str(), sizeof(apName));
  apName[sizeof(apName) - 1] = '\0';

  char lastWillTopic[128];
  snprintf(lastWillTopic, sizeof(lastWillTopic), "%s/%s/status", config.mqtt.baseTopic, apName);

  if (client.connect(
        config.mqtt.clientID,
        config.mqtt.username,
        config.mqtt.password,
        lastWillTopic,
        config.mqtt.qos,
        config.mqtt.retain,
        "offline")) {

    Serial.print("Broker connected at: ");
    Serial.print(config.mqtt.brokerURL);
    Serial.print(" Port at: ");
    Serial.println(config.mqtt.port);

    if (config.operation.publishStatus) {
      publishStatus(client, "online", apName);
    }
    if (config.operation.publishDeviceInfo) {
      publishDeviceInfo(client, apName);
    }
    if (config.operation.publishIP) {
      publishIP(client, apName);
    }

  } else {
    Serial.print("MQTT connection failed, state: ");
    int mqttState = client.state();
    Serial.println(mqttState);

    // Sorter feilen basert på state
    if (mqttState == -2 || mqttState == -1) {
      // -2 = Connection failed (broker not found)
      // -1 = Connection lost
      visualizationOfError(NO_BROKER);
    } else {
      // State 1-5 = Connection refused (various reasons)
      visualizationOfError(BROKER_ERROR);
    }
  }
}

/*
  void runMQTTTests(PubSubClient& client) {
  // Test 1: Verify connection and last will
  Serial.println("\n=== Testing MQTT Connection ===");
  reconnectMQTT(client);
  delay(1000);

  // Test 2: Verify nested JSON publishing
  Serial.println("\n=== Testing Nested JSON Format ===");
  config.mqtt.useNestedJson = true;
  publishMeasuredValue(client, 23.5, "temperature", "°C");
  publishStatus(client, "testing");
  publishDeviceInfo(client);
  delay(1000);

  // Test 3: Verify flat value publishing
  Serial.println("\n=== Testing Flat Value Format ===");
  config.mqtt.useNestedJson = false;
  publishMeasuredValue(client, 45.0, "humidity", "%");
  publishIP(client);
  delay(1000);

  // Test 4: Verify all publish flags
  Serial.println("\n=== Testing All Publish Flags ===");
  config.operation.publishStatus = true;
  config.operation.publishDeviceInfo = true;
  config.operation.publishIP = true;
  config.operation.publishRSSI = true;
  reconnectMQTT(client); // Should publish all status info
  delay(1000);
  }
*/















/*
  // Views / HTML / graphical



*/

// Returns string of the logo as SVG
String getLogoSvg() {
  return String(
           "<svg width=\"50\" height=\"50\" viewBox=\"0 0 106 120\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\">"
           "<path d=\"M79.2727 39.25C78.4773 36.4848 77.3599 34.0417 75.9205 31.9205C74.4811 29.7614 72.7197 27.9432 70.6364 26.4659C68.5909 24.9508 66.2424 23.7955 63.5909 23C60.9773 22.2045 58.0795 21.8068 54.8977 21.8068C48.9508 21.8068 43.7235 23.2841 39.2159 26.2386C34.7462 29.1932 31.2614 33.4924 28.7614 39.1364C26.2614 44.7424 25.0114 51.5985 25.0114 59.7045C25.0114 67.8106 26.2424 74.7045 28.7045 80.3864C31.1667 86.0682 34.6515 90.4053 39.1591 93.3977C43.6667 96.3523 48.9886 97.8295 55.125 97.8295C60.6932 97.8295 65.447 96.8447 69.3864 94.875C73.3636 92.8674 76.3939 90.0455 78.4773 86.4091C80.5985 82.7727 81.6591 78.4735 81.6591 73.5114L86.6591 74.25H56.6591V55.7273H105.352V70.3864C105.352 80.6136 103.193 89.4015 98.875 96.75C94.5568 104.061 88.6098 109.705 81.0341 113.682C73.4583 117.621 64.7841 119.591 55.0114 119.591C44.1023 119.591 34.5189 117.186 26.2614 112.375C18.0038 107.527 11.5644 100.652 6.94318 91.75C2.35985 82.8106 0.0681819 72.2045 0.0681819 59.9318C0.0681819 50.5 1.43182 42.0909 4.15909 34.7045C6.92424 27.2803 10.7879 20.9924 15.75 15.8409C20.7121 10.6894 26.4886 6.76893 33.0795 4.07954C39.6705 1.39015 46.8106 0.0454502 54.5 0.0454502C61.0909 0.0454502 67.2273 1.01136 72.9091 2.94318C78.5909 4.83712 83.6288 7.52651 88.0227 11.0114C92.4545 14.4962 96.072 18.6439 98.875 23.4545C101.678 28.2273 103.477 33.4924 104.273 39.25H79.2727Z\" fill=\"url(#paint0_linear_113_123)\"/>"
           "<defs>"
           "<linearGradient id=\"paint0_linear_113_123\" x1=\"-8\" y1=\"60\" x2=\"114\" y2=\"60\" gradientUnits=\"userSpaceOnUse\">"
           "<stop stop-color=\"#FFBAFC\"/>"
           "<stop offset=\"1\" stop-color=\"#A0225E\"/>"
           "</linearGradient>"
           "</defs>"
           "</svg>"
         );
}

// Return a link of the logo as a anchor ref tag
String getLogoLink() {
  return String(
           "<a href=\"https://www.grefur.com\" target=\"_blank\" style=\"margin-left: 10px; margin-right: 10px;\">"
           "<svg width=\"50\" height=\"50\" viewBox=\"0 0 106 120\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\">"
           "<path d=\"M79.2727 39.25C78.4773 36.4848 77.3599 34.0417 75.9205 31.9205C74.4811 29.7614 72.7197 27.9432 70.6364 26.4659C68.5909 24.9508 66.2424 23.7955 63.5909 23C60.9773 22.2045 58.0795 21.8068 54.8977 21.8068C48.9508 21.8068 43.7235 23.2841 39.2159 26.2386C34.7462 29.1932 31.2614 33.4924 28.7614 39.1364C26.2614 44.7424 25.0114 51.5985 25.0114 59.7045C25.0114 67.8106 26.2424 74.7045 28.7045 80.3864C31.1667 86.0682 34.6515 90.4053 39.1591 93.3977C43.6667 96.3523 48.9886 97.8295 55.125 97.8295C60.6932 97.8295 65.447 96.8447 69.3864 94.875C73.3636 92.8674 76.3939 90.0455 78.4773 86.4091C80.5985 82.7727 81.6591 78.4735 81.6591 73.5114L86.6591 74.25H56.6591V55.7273H105.352V70.3864C105.352 80.6136 103.193 89.4015 98.875 96.75C94.5568 104.061 88.6098 109.705 81.0341 113.682C73.4583 117.621 64.7841 119.591 55.0114 119.591C44.1023 119.591 34.5189 117.186 26.2614 112.375C18.0038 107.527 11.5644 100.652 6.94318 91.75C2.35985 82.8106 0.0681819 72.2045 0.0681819 59.9318C0.0681819 50.5 1.43182 42.0909 4.15909 34.7045C6.92424 27.2803 10.7879 20.9924 15.75 15.8409C20.7121 10.6894 26.4886 6.76893 33.0795 4.07954C39.6705 1.39015 46.8106 0.0454502 54.5 0.0454502C61.0909 0.0454502 67.2273 1.01136 72.9091 2.94318C78.5909 4.83712 83.6288 7.52651 88.0227 11.0114C92.4545 14.4962 96.072 18.6439 98.875 23.4545C101.678 28.2273 103.477 33.4924 104.273 39.25H79.2727Z\" fill=\"url(#paint0_linear_113_123)\"/>"
           "<defs>"
           "<linearGradient id=\"paint0_linear_113_123\" x1=\"-8\" y1=\"60\" x2=\"114\" y2=\"60\" gradientUnits=\"userSpaceOnUse\">"
           "<stop stop-color=\"#FFBAFC\"/>"
           "<stop offset=\"1\" stop-color=\"#A0225E\"/>"
           "</linearGradient>"
           "</defs>"
           "</svg>"
           "</a>"
         );
}

String getNavigationBar() {
  String html =
    "<style>"
    ".nav-container {"
    "  margin-bottom: 20px;"
    "  display: flex;"
    "  gap: 12px;"
    "  flex-wrap: wrap;"
    "  align-items: center;"
    "  padding: 10px;"
    "  background: #f8f9fa;"
    "  box-shadow: 0 2px 8px rgba(0,0,0,0.1);"
    "  border-radius: 8px;"
    "}"
    ".nav-button {"
    "  padding: 10px 20px;"
    "  border-radius: 6px;"
    "  text-decoration: none;"
    "  font-weight: 600;"
    "  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;"
    "  color: #ffffff;"
    "  background-color: #6c757d;"
    "  border: none;"
    "  cursor: pointer;"
    "  transition: all 0.3s ease;"
    "  box-shadow: 0 2px 5px rgba(0,0,0,0.1);"
    "}"
    ".nav-button:hover {"
    "  background-color: #5a6268;"
    "  box-shadow: 0 4px 8px rgba(0,0,0,0.15);"
    "}"
    ".nav-button.home-button {"
    "  background-color: #4CAF50;"
    "}"
    ".nav-button.home-button:hover {"
    "  background-color: #3e8e41;"
    "}"
    "</style>"
    "<nav class='nav-container'>";

  html += getLogoLink();  // Logo first

  String homeButton = "<a href='/' class='nav-button home-button'>Home</a>";
  String firmwareButton = "<a href='/firmware' class='nav-button'>Firmware</a>";
  //String certButton = "<a href='/certificate' class='nav-button'>Certificates</a>";
  String backupButton = "<a href='/config/backup' class='nav-button'>Backup</a>";

  html += homeButton + firmwareButton + certButton + backupButton;
  html += "</nav>";
  return html;
}

// Default header to use on all pages
String getHTMLHeader(const String & title) {
  return String("<html><head><meta charset='UTF-8'><title>") + title +
         "</title><meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>"
         "<style>"
         "body {font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 20px;}"
         "h1 {color: #444;}"
         "form {background: #f9f9f9; padding: 20px; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1);}"
         "input, select {width: 100%; padding: 10px; margin: 8px 0 15px; border: 1px solid #ddd; border-radius: 6px; box-sizing: border-box; font-size: 16px; transition: all 0.3s ease;}"
         "input:focus, select:focus {border-color: #6c757d; outline: none; box-shadow: 0 0 0 2px rgba(108, 117, 125, 0.2);}"
         "input[type='submit'] {"
         "  background-color: #4CAF50;"
         "  color: white;"
         "  padding: 10px 20px;"
         "  border: none;"
         "  border-radius: 6px;"
         "  cursor: pointer;"
         "  font-weight: 600;"
         "  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;"
         "  transition: all 0.3s ease;"
         "  box-shadow: 0 2px 5px rgba(0,0,0,0.1);"
         "}"
         "input[type='submit']:hover {"
         "  background-color: #3e8e41;"
         "  box-shadow: 0 4px 8px rgba(0,0,0,0.15);"
         "}"
         "input[type='checkbox'] {width: auto; margin-right: 8px;}"
         "button {"
         "  font-size: 16px;"
         "  padding: 10px 20px;"
         "  border-radius: 6px;"
         "  cursor: pointer;"
         "  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;"
         "  transition: all 0.3s ease;"
         "}"
         ".info {background: #e7f3fe; border-left: 6px solid #2196F3; padding: 12px; margin: 15px 0; border-radius: 0 4px 4px 0;}"
         ".section {margin-bottom: 25px; border-bottom: 1px solid #eee; padding-bottom: 25px;}"
         ".grid {display: grid; grid-template-columns: 1fr 1fr; gap: 20px;}"
         "@media (max-width: 600px) {.grid {grid-template-columns: 1fr; gap: 15px;}}"
         "</style>"
         "<link rel='icon' href='/favicon.svg' type='image/svg+xml'>"
         "</head>"
         "<body>" +
         getNavigationBar();
}

// Default footer to use on all pages
String getHTMLFooter() {
  return String(
           "<footer style='margin-top: 20px; text-align: center; color: #777;'>"
           "Temperature Sensor v" + getFirmwareVersion() + "<br>"
           "All rights reserved &copy; Grefur"
           "</footer></body></html>"
         );
}

String getDeviceInfoHTML() {
  String html;
  html += "<div class='info'>";
  html += "<p><strong>Device ID:</strong> " + String(ESP.getChipId(), HEX) + "</p>";
  html += "<p><strong>Firmware Version:</strong> " + getFirmwareVersion() + "</p>";

  if (WiFi.getMode() == WIFI_AP) {
    html += "<p><strong>Mode:</strong> Configuration (AP)</p>";
    html += "<p><strong>AP Name:</strong> " + generateAPName() + "</p>";
  } else {

    String operationMode;
    if (!mqttClient.connected()) {
      operationMode = "No broker connected";
    } else {
      operationMode = "Normal operation";
    }

    html += "<p><strong>Mode: </strong>" + operationMode + "</p>";
    html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>WiFi RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>";
    //html += "<p><strong>MQTT Status:</strong> " + getMQTTStatus() + "</p>";
  }

  html += "</div>";
  return html;
}

// HTML-generator
String getLEDControls() {
  String html =
    "<style>"
    ".led-button {"
    "  color: white;"
    "  border: none;"
    "  padding: 10px 20px;"
    "  margin-right: 10px;"
    "  border-radius: 6px;"
    "  font-weight: 600;"
    "  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;"
    "  cursor: pointer;"
    "  transition: all 0.3s ease;"
    "  box-shadow: 0 2px 5px rgba(0,0,0,0.1);"
    "}"
    ".led-on {"
    "  background-color: #2196F3;"
    "}"
    ".led-on:hover {"
    "  background-color: #0b7dda;"
    "  box-shadow: 0 4px 8px rgba(0,0,0,0.15);"
    "}"
    ".led-off {"
    "  background-color: #f44336;"
    "}"
    ".led-off:hover {"
    "  background-color: #da190b;"
    "  box-shadow: 0 4px 8px rgba(0,0,0,0.15);"
    "}"
    "</style>"
    "<div style='margin: 20px 0;'>"
    "<button class='led-button led-on' onclick=\"controlLED('on')\">Turn LED ON</button>"
    "<button class='led-button led-off' onclick=\"controlLED('off')\">Turn LED OFF</button>"
    "</div>";

  return html;
}

String getLedControlHTML() {
  String html;
  html += "<div class='section' style='margin-top: 20px;'>";
  html += "<h2>LED Control</h2>";
  html += getLEDControls();

  html += "<script>"
          "function controlLED(state) {"
          "  fetch('/config/wink?state=' + state)"
          "    .then(response => response.text())"
          "}"
          "</script>";
  html += "</div>";
  return html;
}

String DisplayDeviceInformationHTML() {
  String html;

  // Device Info Section
  html += "<div class='section'>";
  html += "<h2>Device Information</h2>";
  html += "Device Name: <input type='text' name='deviceName' value='" + String(config.device.deviceName) + "'><br>";
  html += "Location: <input type='text' name='deviceLocation' value='" + String(config.device.location) + "'><br>";
  html += "</div>";
  return html;
}


String getWiFiConfigForm() {
  String html;
  html += "<form action='/config/wifi' method='POST'>";
  html += "<h2>WiFi Configuration</h2>";

  // Prefill SSID if it exists
  html += "SSID: <input type='text' name='ssid' value='" + String(config.network.wifiSSID) + "'><br>";

  // Password field (note: browsers won't show the actual password, but it'll be sent if unchanged)
  html += "Password: <input type='password' name='password' placeholder='";
  if (strlen(config.network.wifiPassword) > 0) {
    html += "[Password is set]";  // Placeholder text (actual password isn't shown for security)
  } else {
    html += "Enter new password";
  }
  html += "'><br>";

  html += "<input type='submit' value='Save WiFi'>";
  html += "</form>";
  html += "<br>";
  return html;
}

String DisplayMQTTSettingsHTML() {
  String html;

  html += "<form method='POST' action='/config/mqtt'>";
  html += "<div class='section'>";
  html += "<h2>MQTT Settings</h2>";
  html += "<div class='grid'>";

  html += "<div>";
  html += "<label for='mqttServer'>Broker URL:</label><br>";
  html += "<input id='mqttServer' type='text' name='mqttServer' value='" + String(config.mqtt.brokerURL) + "'><br>";
  html += "<label for='mqttPort'>Port:</label><br>";
  html += "<input id='mqttPort' type='number' name='mqttPort' value='" + String(config.mqtt.port) + "' min='1' max='65535'><br>";
  html += "<label for='mqttClientId'>Client ID:</label><br>";
  html += "<input id='mqttClientId' type='text' name='mqttClientId' value='" + String(config.mqtt.clientID) + "'><br>";
  html += "</div>";

  html += "<div>";
  html += "<label for='mqttUser'>Username:</label><br>";
  html += "<input id='mqttUser' type='text' name='mqttUser' value='" + String(config.mqtt.username) + "'><br>";
  html += "<label for='mqttPassword'>Password:</label><br>";
  html += "<input id='mqttPassword' autocomplete='off' type='password' name='mqttPassword' placeholder='";
  if (strlen(config.mqtt.password) > 0) {
    html += "[Password is set]";
  } else {
    html += "Enter new password";
  }
  html += "'><br>";
  html += "<label for='mqttTopic'>Topic base:</label><br>";
  html += "<input id='mqttTopic' type='text' name='mqttTopic' value='" + String(config.mqtt.baseTopic) + "'><br>";
  html += "</div>";

  html += "</div>"; // Close grid

  html += "<label for='mqttQos'>QoS:</label><br>";
  html += "<select id='mqttQos' name='mqttQos'>";
  html += "<option value='0'" + String(config.mqtt.qos == 0 ? " selected" : "") + ">0 - At most once</option>";
  html += "<option value='1'" + String(config.mqtt.qos == 1 ? " selected" : "") + ">1 - At least once</option>";
  html += "<option value='2'" + String(config.mqtt.qos == 2 ? " selected" : "") + ">2 - Exactly once</option>";
  html += "</select><br>";

  html += "<label><input type='checkbox' name='mqttRetain'" + String(config.mqtt.retain ? " checked" : "") + "> Retain Messages</label><br>";

  // Message Format Section
  html += "<h3>Message Format</h3>";
  html += "<label for='useNestedJson'>Message Format:</label><br>";
  html += "<select id='useNestedJson' name='useNestedJson'>";
  html += "<option value='1'" + String(config.mqtt.useNestedJson ? " selected" : "") + ">Nested JSON</option>";
  html += "<option value='0'" + String(!config.mqtt.useNestedJson ? " selected" : "") + ">Flat Topics</option>";
  html += "</select><br>";

  html += "<input type='submit' value='Save MQTT Settings'>";
  html += "</div>"; // Close section
  html += "</form>";

  return html;
}


String DisplayOperationSettings() {
  String html;

  html += "<form method='POST' action='/config/operations'>";
  html += "<div class='section'>";

  html += "<div class='grid-container'>"; // Changed class to grid-container

  // Left column (50%)
  html += "<div class='grid-left '>";
  html += "<h2>Operation Settings</h2>";

  // Publish Interval input group
  html += "<div style='margin-bottom: 1em;'>";
  html += "<label for='publishInterval' style='display: block;'>Publish Interval (ms):</label>";
  html += "<input id='publishInterval' type='number' name='publishInterval' value='" + String(config.operation.publishInterval) + "' style='max-width: 350px; width: 100%;'>";
  html += "</div>";

  // Config Password input group
  html += "<div style='margin-bottom: 1em;'>";
  html += "<label for='configPassword' style='display: block;'>Config Password:</label>";
  html += "<input id='configPassword' autocomplete='off' type='password' name='configPassword' placeholder='Enter new password' style='max-width: 350px; width: 100%;'>";
  html += "</div>";

  html += "</div>"; // Close grid-left

  // Right column (50%)
  html += "<div class='grid-right'>";
  html += "<h2>Publish Options</h2>";
  html += "<div class='input-group'>";
  html += "<label><input type='checkbox' name='publishTemperature'" + String(config.operation.publishMeasuredValues ? " checked" : "") + "> Measured values</label>";
  html += "<label><input type='checkbox' name='publishIP'" + String(config.operation.publishIP ? " checked" : "") + "> IP Address</label>";
  html += "<label><input type='checkbox' name='publishStatus'" + String(config.operation.publishStatus ? " checked" : "") + "> Status</label>";
  html += "<label><input type='checkbox' name='publishDeviceInfo'" + String(config.operation.publishDeviceInfo ? " checked" : "") + "> Device Info</label>";
  html += "<label><input type='checkbox' name='publishRSSI'" + String(config.operation.publishRSSI ? " checked" : "") + "> RSSI (with status)</label>";
  html += "</div>"; // Close checkbox-group
  html += "</div>"; // Close grid-right

  html += "</div>"; // Close grid-container

  html += "<div class='form-footer'>";
  html += "<input type='submit' value='Save Operation Settings'>";
  html += "</div>";
  html += "</div>"; // Close section
  html += "</form>";

  // Add CSS for the grid layout
  html += "<style>";
  html += ".grid-container { display: flex; gap: 20px; }";
  html += ".grid-left, .grid-right { flex: 1; flex-direction: column; gap: 8px; }"; // Each takes 50% of space
  html += ".input-group { display: flex; flex-direction: column; gap: 8px; }";
  html += "</style>";

  return html;
}

// One row for each sensor with all the informationfields, and a button for edit and one for delete. Make these dummy for so long
String DisplaySensorsInProduction() {
  String html;

  html += "<div class='sensor-list'>";
  html += "<h2>Configured Sensors</h2>";
  html += "<table class='sensor-table'>";
  html += "<thead><tr>";
  html += "<th>ID</th>";
  html += "<th>Name</th>";
  html += "<th>Type</th>";
  html += "<th>Details</th>";
  html += "<th>Actions</th>";
  html += "</tr></thead>";
  html += "<tbody>";

  // Loop through all possible sensors
  for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
    Serial.println("Sensortype:" + config.sensors[i].type);
    if (config.sensors[i].type == SENSOR_UNCONFIGURED) continue; // Skip unconfigured sensors

    html += "<tr>";
    html += "<td>" + String(i) + "</td>";
    html += "<td>" + String(config.sensors[i].name) + "</td>";

    // Convert type code to readable name
    String typeName = "Unknown";
    if (config.sensors[i].type == SENSOR_NTC) typeName = "NTC";
    else if (config.sensors[i].type == SENSOR_ANALOG) typeName = "Analog";
    else if (config.sensors[i].type == SENSOR_DIGITAL) typeName = "Digital";

    html += "<td>" + typeName + "</td>";

    // Get sensor details
    String details = "";

    String publishMode = (config.sensors[i].publishOnlyChange == 1) ? "Publish: Changes only" : "Publish: Interval ";

    if (config.sensors[i].type == SENSOR_NTC) {
      details = "Pin A" + String(config.sensors[i].ntc.analogPin) + ", " +
                String(config.sensors[i].ntc.nominalResistance) + "Ω @ " +
                String(config.sensors[i].ntc.nominalTemp) + "°C, B=" +
                String(config.sensors[i].ntc.bCoefficient) + ", " +
                String(config.sensors[i].ntc.unit) + " " +
                publishMode;
    }
    else if (config.sensors[i].type == SENSOR_ANALOG) {
      details = "GPIO" + String(config.sensors[i].analog.analogPin) + ", " +
                String(config.sensors[i].analog.inputMin) + "-" +
                String(config.sensors[i].analog.inputMax) + "V → " +
                String(config.sensors[i].analog.scaleMin) + "-" +
                String(config.sensors[i].analog.scaleMax) + " " +
                String(config.sensors[i].analog.scaleUnit) + " " +
                publishMode;
    }
    else if (config.sensors[i].type == SENSOR_DIGITAL) {
      details = "GPIO" + String(config.sensors[i].digital.digitalPin) +
                (config.sensors[i].digital.inverted ? " (Inverted)" : "") + " " +
                publishMode;
    }


    html += "<td>" + details + "</td>";
    html += "<td class='actions'>";
    html += "<a class='btn-edit' href='/edit/" + String(i) + "'>Edit</a>";
    html += "<button class='btn-delete' onclick='deleteSensor(" + String(i) + ")'>Delete</button>";
    html += "</td>";
    html += "</tr>";
  }

  html += "</tbody>";
  html += "</table>";
  html += "</div>";

  // Add JavaScript functions for the buttons
  html += "<script>";
  html += "function deleteSensor(id) {";
  html += "  if (confirm('Are you sure you want to delete sensor ' + id + '?')) {";
  html += "    fetch('/config/sensor?id=' + id, { method: 'DELETE' })";
  html += "      .then(response => {";
  html += "        if (response.ok) {";
  html += "          window.location.reload();";
  html += "        } else {";
  html += "          console.error('Delete failed');";
  html += "        }";
  html += "      })";
  html += "      .catch(error => console.error('Error:', error));";
  html += "  }";
  html += "}";
  html += "</script>";

  // Add some basic CSS
  html += "<style>";
  html += ".sensor-table { width: 100%; border-collapse: collapse; }";
  html += ".sensor-table th, .sensor-table td { padding: 8px; text-align: left; border-bottom: 1px solid #ddd; }";
  html += ".btn-edit { background-color: #4CAF50; color: white; padding: 5px 10px; border: none; border-radius: 4px; cursor: pointer; margin-right: 5px; }";
  html += ".btn-delete { background-color: #f44336; color: white; padding: 5px 10px; border: none; border-radius: 4px; cursor: pointer; }";
  html += ".actions { white-space: nowrap; }";
  html += "</style>";

  return html;
}

String DisplayAddSensorFields() {
  String html = "";

  html += "<div class='sensor-form'>";
  html += "<h2>Add New Sensor</h2>";

  int availableSlots = MAX_SENSORS_ALLOWED - sensorManager.getConfiguredSensorCount();
  if (DEBUG) {
    Serial.println("Available slots: " + String(availableSlots));
  }

  html += "<div class='slots-info'>Available slots: " + String(availableSlots) + "</div>";

  if (availableSlots <= 0) {
    html += "<div class='no-slots'>No available sensor slots. Please delete a sensor to add a new one.</div>";
    html += "</div>";  // sensor-form
    return html;
  }

  html += "<form id='sensorForm' action='/config/sensor' method='POST'>";

  // Sensor type dropdown
  html += "<div class='form-group'>";
  html += "<label for='sensorType'>Sensor Type:</label>";
  html += "<select id='sensorType' name='type' required>";
  html += "<option value=''>Select a type</option>";

  // Populate sensor types
  String typesJson = sensorManager.retrieveSensorTypes();
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, typesJson);
  if (!error && doc.containsKey("types")) {
    for (JsonVariant typeVar : doc["types"].as<JsonArray>()) {
      String type = typeVar.as<String>();
      String typeLower = type;
      typeLower.toLowerCase();
      html += "<option value='" + typeLower + "'>" + type + "</option>";

    }
  } else {
    html += "<option value=''>Error loading types</option>";
  }

  html += "</select>";
  html += "</div>";

  // Sensor name
  html += "<div class='form-group'>";
  html += "<label for='name'>Sensor Name:</label>";
  html += "<input type='text' id='name' name='name' required>";
  html += "</div>";

  // Placeholder for dynamic fields
  html += "<div id='dynamicFields'></div>";

  html += "<input type='submit' value='Add Sensor'>";
  html += "</form>";

  // JavaScript for dynamic fields
  html += "<script>";
  html += "document.getElementById('sensorType').addEventListener('change', function() {";
  html += "  const type = this.value;";
  html += "  const container = document.getElementById('dynamicFields');";
  html += "  container.innerHTML = '';";

  // NTC
  html += "  if (type === 'ntc') {";
  html += "    container.innerHTML += \"<div class='form-group'><label>Series Resistor:</label><input type='number' step='any' name='seriesResistor' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Nominal Resistance:</label><input type='number' step='any' name='nominalResistance' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Nominal Temp (°C):</label><input type='number' step='any' name='nominalTemp' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>B Coefficient:</label><input type='number' step='any' name='bCoefficient' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Temperature unit:</label><input type='text' name='unit' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Analog Pin:</label><input type='number' name='analogPin' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Publish only changes:</label><input type='checkbox' name='publishOnlyChange'></div>\";";
  html += "  }";

  // Analog
  html += "  else if (type === 'analog') {";
  html += "    container.innerHTML += \"<div class='form-group'><label>Analog Pin:</label><input type='number' name='analogPin' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Input Min:</label><input type='number' step='any' name='inputMin' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Input Max:</label><input type='number' step='any' name='inputMax' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Scale Min:</label><input type='number' step='any' name='scaleMin' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Scale Max:</label><input type='number' step='any' name='scaleMax' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Scaled unit:</label><input type='text' name='scaleUnit' required></div>\";";
  html += "    container.innerHTML += \"<div class='form-group'><label>Publish only changes:</label><input type='checkbox' name='publishOnlyChange'></div>\";";
  html += "  }";

  // Digital
  html += "  else if (type === 'digital') {";
  html += "   container.innerHTML += '<div class=\"form-group\"><label>Digital Pin:</label><select name=\"digitalPin\" required>";
  html +=        "<option value=\"16\">D0 (GPIO16)</option>";
  html +=        "<option value=\"5\">D1 (GPIO5)</option>";
  html +=        "<option value=\"4\">D2 (GPIO4)</option>";
  html +=        "<option value=\"14\">D5 (GPIO14)</option>";
  html +=        "<option value=\"12\">D6 (GPIO12)</option>";
  html +=        "<option value=\"13\">D7 (GPIO13)</option>";
  html +=        "</select></div>';";

  html += "container.innerHTML += '<div class=\"form-group\"><label>Inverted:</label><input type=\"checkbox\" name=\"inverted\"></div>';";
  html += "container.innerHTML += \"<div class='form-group'><label>Publish only changes:</label><input type='checkbox' name='publishOnlyChange'></div>\";";

  html += "  }";

  html += "});";
  html += "</script>";

  // Add some basic CSS
  html += "<style>";
  html += ".form-group { margin-bottom: 15px; }";
  html += ".form-group label { display: inline-block; width: 150px; white-space: nowrap; }"; // prevent wrapping
  html += ".form-group input[type='checkbox'] { transform: scale(1.5); margin-right: 10px; margin-left: 10px; }"; // scale checkbox
  html += ".no-slots { color: red; font-weight: bold; margin: 20px 0; }";
  html += ".slots-info { margin-bottom: 15px; font-weight: bold; }";
  html += "</style>";


  html += "</div>";  // sensor-form
  return html;
}



void handleRoot() {
  if (DEBUG) {
    Serial.println("Kall for ROOT...");
  }

  if (!isAuthenticated()) return server.requestAuthentication();

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);  // Use chunked encoding
  server.send(200, "text/html", "");

  server.sendContent(getHTMLHeader("Configuration"));
  server.sendContent("<h1>Temperature Sensor Configuration</h1>");

  server.sendContent(getDeviceInfoHTML());
  server.sendContent(getLedControlHTML());
  server.sendContent(DisplayDeviceInformationHTML());
  server.sendContent(getWiFiConfigForm());
  server.sendContent(DisplayMQTTSettingsHTML());
  server.sendContent(DisplayOperationSettings());
  server.sendContent(DisplaySensorsInProduction());
  server.sendContent(DisplayAddSensorFields());

  server.sendContent(getHTMLFooter());
  server.sendContent("");

  server.client().stop();  // Close connection explicitly if needed
}

// GET request for uploading new firmware
void firmwareVersionPage() {
  if (!isAuthenticated()) return server.requestAuthentication();

  String html = getHTMLHeader("Firmware Update");
  html += "<h1>Firmware Update</h1>";
  html += "<div class='info'>";
  html += "<p><strong>Device ID:</strong> " + String(ESP.getChipId(), HEX) + "</p>";
  html += "<p><strong>Firmware Version:</strong> " + getFirmwareVersion() + "</p>";
  html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
  html += "</div>";
  html += "<h2>Upload New Firmware</h2>";
  html += "<form method='POST' action='/firmware' enctype='multipart/form-data'>";
  html += "<input type='file' name='firmware'><br><br>";
  html += "<input type='submit' value='Upload & Update'>";
  html += "</form>";
  html += getHTMLFooter();
  server.send(200, "text/html", html);
}

void backupPage() {
  if (!isAuthenticated()) return server.requestAuthentication();

  String html = getHTMLHeader("Backup");
  html += "<h1>Config backup system</h1>";
  html += "<div class='info'>";
  html += "<p><strong>Device ID:</strong> " + String(ESP.getChipId(), HEX) + "</p>";
  html += "<p><strong>Firmware Version:</strong> " + getFirmwareVersion() + "</p>";
  html += "</div>";

  html += "<h2>Create new backup</h2>";
  html += "<div style='position:relative; width:100%; height:30px; margin-bottom:10px;'>";
  html += "<input id='genBackupBtn' type='button' value='Generate backup' onclick='generateBackup()' style='width:100%; height:100%;'>";
  html += "<div id='progressBar' style='position:absolute; top:0; left:0; height:100%; width:100%; background:linear-gradient(to right, rgba(0,255,0,0), rgba(0,255,0,0.6), rgba(0,255,0,0)); opacity:0; transition: opacity 0.1s;'></div>";
  html += "</div>";

  html += "<input type='button' value='Download last backup' onclick='downloadBackup()'>";

  html += "<h2>Restore backup</h2>";
  html += "<form method='POST' action='/backup/upload' enctype='multipart/form-data'>";
  html += "<input type='file' name='backupfile' accept='.json'><br><br>";
  html += "<input type='submit' value='Upload and restore'>";
  html += "</form>";

  html += "<script>";
  html += "function generateBackup() {";
  html += "  let bar = document.getElementById('progressBar');";
  html += "  let opacity = 0;";
  html += "  let increasing = true;";
  html += "  bar.style.opacity = 0;";
  html += "  let interval = setInterval(() => {";
  html += "    if(increasing){ opacity += 0.01; if(opacity >= 0.6) increasing = false; }";
  html += "    else{ opacity -= 0.01; if(opacity <= 0) clearInterval(interval); }";
  html += "    bar.style.opacity = opacity;";
  html += "  }, 70);"; // 7 sekunder totalt for opp og ned (0.01 * 70ms * 60 ~ 4200ms, juster for eksakt 7s)
  html += "  fetch('/backup/download').then(response => response.blob())";
  html += "  .then(blob => {";
  html += "    bar.style.opacity = 0;";  // fjern gradient når ferdig
  html += "    const url = window.URL.createObjectURL(blob);";
  html += "    const a = document.createElement('a');";
  html += "    a.style.display = 'none'; a.href = url; a.download = 'config_backup.json';";
  html += "    document.body.appendChild(a); a.click();";
  html += "    window.URL.revokeObjectURL(url);";
  html += "  });";
  html += "}";

  html += "function downloadBackup() { window.open('/backup/download', '_blank'); }";
  html += "</script>";

  html += getHTMLFooter();
  server.send(200, "text/html", html);
}




/*
   @brief ROUTEHANDLERS WITH POST AND GET


*/

void handleWifiSettingsChange() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) return;

  // Capture original values
  char originalSSID[sizeof(config.network.wifiSSID)];
  char originalPass[sizeof(config.network.wifiPassword)];
  strncpy(originalSSID, config.network.wifiSSID, sizeof(originalSSID));
  strncpy(originalPass, config.network.wifiPassword, sizeof(originalPass));

  // Update fields
  bool ssidChanged = mgr.updateStringField(config.network.wifiSSID,
                     sizeof(config.network.wifiSSID),
                     server.arg("ssid"));
  bool passChanged = mgr.updateStringField(config.network.wifiPassword,
                     sizeof(config.network.wifiPassword),
                     server.arg("password"));

  // Handle changes
  if (mgr.hasChanges()) {
    mgr.scheduleRestart();
    mgr.finalize("WiFi settings updated");
  }
}

/**
   @brief Handle MQTT configuration updates

   Processes and saves MQTT broker settings including:
   - Connection parameters (URL, port)
   - Credentials (username/password)
   - Publishing options (QoS, retain flag)
   Schedules restart if changes were made.
*/
void handleMQTTSettingsChange() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) return;

  // Update all fields
  mgr.updateStringField(config.mqtt.brokerURL, sizeof(config.mqtt.brokerURL), server.arg("mqttServer"));
  mgr.updateStringField(config.mqtt.clientID, sizeof(config.mqtt.clientID), server.arg("mqttClientId"));
  mgr.updateStringField(config.mqtt.username, sizeof(config.mqtt.username), server.arg("mqttUser"));
  mgr.updateStringField(config.mqtt.password, sizeof(config.mqtt.password), server.arg("mqttPassword"));
  mgr.updateStringField(config.mqtt.baseTopic, sizeof(config.mqtt.baseTopic), server.arg("mqttTopic"));

  mgr.updateUint16Field(config.mqtt.port, server.arg("mqttPort"));
  ///mgr.updateNumericField(config.mqtt.port, server.arg("mqttPort"));

  mgr.updateNumericField(config.mqtt.qos, server.arg("mqttQos"));

  mgr.updateBoolField(config.mqtt.retain, server.hasArg("mqttRetain"));
  mgr.updateBoolField(config.mqtt.useNestedJson, server.hasArg("useNestedJson"));

  if (mgr.hasChanges()) {
    mgr.scheduleRestart();
  }
  mgr.finalize("MQTT settings updated");
}

/**
   @brief Handle device operation settings updates

   Manages:
   - Measurement publish interval
   - Admin password
   - Data publishing flags (temperature, IP, status etc.)
   Triggers restart on configuration changes.
*/
void handleOperationsSettingsChange() {
  SaveConfigManager mgr(config);
  //if (!mgr.authenticateRequest()) return;

  mgr.updateNumericField(config.operation.publishInterval, server.arg("publishInterval"));
  mgr.updateStringField(config.device.adminPass, sizeof(config.device.adminPass), server.arg("configPassword"));

  mgr.updateBoolField(config.operation.publishMeasuredValues, server.hasArg("publishTemperature"));
  mgr.updateBoolField(config.operation.publishIP, server.hasArg("publishIP"));
  mgr.updateBoolField(config.operation.publishStatus, server.hasArg("publishStatus"));
  mgr.updateBoolField(config.operation.publishDeviceInfo, server.hasArg("publishDeviceInfo"));
  mgr.updateBoolField(config.operation.publishRSSI, server.hasArg("publishRSSI"));

  if (mgr.hasChanges()) {
    mgr.scheduleRestart();
  }
  mgr.finalize("Operation settings updated");
}



/*
  Flexible Handling:
    Returns fields for a specific sensor type if type parameter provided
    Returns all available types if no parameter specified
*/
void handleSensorTypeRequest() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) return;

  // Håndter GET-parameter for sensortype
  if (server.method() == HTTP_GET) {
    if (server.hasArg("type")) {
      // Hent og valider type-parameter
      String typeStr = server.arg("type");
      typeStr.toLowerCase();

      int8_t type = -1;
      if (typeStr == "ntc") {
        type = SENSOR_NTC;
      } else if (typeStr == "analog") {
        type = SENSOR_ANALOG;
      } else if (typeStr == "digital") {
        type = SENSOR_DIGITAL;
      }

      if (type != -1) {
        String response = sensorManager.retrieveSensorFieldsAsJson(type);
        server.send(200, "application/json", response);
      } else {
        server.send(400, "text/plain", "Not compatible sensortype'");
      }
    } else {
      // Hvis ingen type er spesifisert, returner alle typer
      String response = sensorManager.retrieveSensorTypes();
      server.send(200, "application/json", response);
    }
  } else {
    server.send(405, "text/plain", "Method not supported");
  }
}


void handleSensorConfig() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  // Check available slots first
  if (sensorManager.getConfiguredSensorCount() >= MAX_SENSORS_ALLOWED) {
    server.send(409, "text/plain", "Maximum number of sensors reached (" + String(MAX_SENSORS_ALLOWED) + ")");
    return;
  }

  SensorConfig newConfig;
  memset(&newConfig, 0, sizeof(SensorConfig)); // Clear memory

  // Read and validate common fields
  String typeStr = server.arg("type");
  String name = server.arg("name");

  if (name.length() >= sizeof(newConfig.name)) {
    server.send(400, "text/plain", "Sensor name too long");
    return;
  }

  Serial.println("=== Sensor Config Submission ===");
  Serial.println("Type: " + typeStr);
  Serial.println("Name: " + name);

  strncpy(newConfig.name, name.c_str(), sizeof(newConfig.name));
  newConfig.name[sizeof(newConfig.name) - 1] = '\0'; // Ensure null-terminated

  if (typeStr == "ntc") {
    newConfig.type = SENSOR_NTC;
    newConfig.ntc.seriesResistor = server.arg("seriesResistor").toFloat();
    newConfig.ntc.nominalResistance = server.arg("nominalResistance").toFloat();
    newConfig.ntc.nominalTemp = server.arg("nominalTemp").toFloat();
    newConfig.ntc.bCoefficient = server.arg("bCoefficient").toFloat();
    newConfig.ntc.analogPin = server.arg("analogPin").toInt();

    newConfig.publishOnlyChange = server.arg("publishOnlyChange") == "on";
    Serial.println("PublishOnlyChange: " + String(newConfig.publishOnlyChange));
    Serial.println("PublishOnlyChange: " + String(server.arg("publishOnlyChange") == "on"));

    // Handle unit as char array
    String unitStr = server.arg("unit");
    strncpy(newConfig.ntc.unit, unitStr.c_str(), sizeof(newConfig.ntc.unit) - 1);
    newConfig.ntc.unit[sizeof(newConfig.ntc.unit) - 1] = '\0';

    Serial.println("NTC Config:");
    Serial.println("  seriesResistor: " + String(newConfig.ntc.seriesResistor));
    Serial.println("  nominalResistance: " + String(newConfig.ntc.nominalResistance));
    Serial.println("  nominalTemp: " + String(newConfig.ntc.nominalTemp));
    Serial.println("  bCoefficient: " + String(newConfig.ntc.bCoefficient));
    Serial.println("  analogPin: " + String(newConfig.ntc.analogPin));
    Serial.println("  unit: " + String(newConfig.ntc.unit));
  }
  else if (typeStr == "analog") {
    newConfig.type = SENSOR_ANALOG;
    newConfig.analog.analogPin = server.arg("analogPin").toInt();
    newConfig.analog.inputMin = server.arg("inputMin").toFloat();
    newConfig.analog.inputMax = server.arg("inputMax").toFloat();
    newConfig.analog.scaleMin = server.arg("scaleMin").toFloat();
    newConfig.analog.scaleMax = server.arg("scaleMax").toFloat();

    newConfig.publishOnlyChange = server.arg("publishOnlyChange") == "on";
    Serial.println("PublishOnlyChange: " + String(newConfig.publishOnlyChange));
    Serial.println("PublishOnlyChange: " + String(server.arg("publishOnlyChange") == "on"));

    // Handle scaleUnit as char array (fixed typo from ntc to analog)
    String scaleUnitStr = server.arg("scaleUnit");
    strncpy(newConfig.analog.scaleUnit, scaleUnitStr.c_str(), sizeof(newConfig.analog.scaleUnit) - 1);
    newConfig.analog.scaleUnit[sizeof(newConfig.analog.scaleUnit) - 1] = '\0';

    Serial.println("Analog Config:");
    Serial.println("  analogPin: " + String(newConfig.analog.analogPin));
    Serial.println("  inputMin: " + String(newConfig.analog.inputMin));
    Serial.println("  inputMax: " + String(newConfig.analog.inputMax));
    Serial.println("  scaleMin: " + String(newConfig.analog.scaleMin));
    Serial.println("  scaleMax: " + String(newConfig.analog.scaleMax));
    Serial.println("  scaleUnit: " + String(newConfig.analog.scaleUnit));
  }
  else if (typeStr == "digital") {
    newConfig.type = SENSOR_DIGITAL;
    newConfig.digital.digitalPin = server.arg("digitalPin").toInt();
    newConfig.digital.inverted = server.arg("inverted") == "on";
    newConfig.publishOnlyChange = server.arg("publishOnlyChange") == "on";

    Serial.println("Digital Config:");
    Serial.println("  digitalPin: " + String(newConfig.digital.digitalPin));
    Serial.println("  inverted: " + String(newConfig.digital.inverted ? "true" : "false"));
  }
  else {
    Serial.println("Unknown sensor type: " + typeStr);
    server.send(400, "text/plain", "Invalid sensor type");
    return;
  }

  // Add sensor config
  Serial.println("Attempting to add sensor config...");

  // Add sensor config
  int sensorId = sensorManager.addSensor(newConfig); // Add new changes
  sensorManager.saveChanges(); // Save the new changes

  Serial.println("Sensorid:" + sensorId);

  if (sensorId >= 0) {
    server.sendHeader("Location", "/"); // Or the path you want to redirect to
    server.send(303, "text/plain", "Sensor added successfully. ID: " + String(sensorId));

    //server.send(200, "text/plain", "Sensor added successfully. ID: " + String(sensorId));
  } else {
    server.send(500, "text/plain", "Failed to add sensor");
  }
}

void handleDeleteSensor() {
  SaveConfigManager mgr(config);

  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Missing sensor ID");
    return;
  }

  int id = server.arg("id").toInt();
  sensorManager.printSensorStates();

  if (sensorManager.removeSensor(id)) {
    Serial.println("Sensor marked for deletion in RAM");

    if (sensorManager.saveChanges()) {
      // Verify deletion in RAM (no EEPROM read needed)
      if (config.sensors[id].type == SENSOR_UNCONFIGURED) {
        server.send(200, "application/json",
                    "{\"status\":\"success\",\"id\":" + String(id) + "}");
      } else {
        server.send(500, "application/json",
                    "{\"error\":\"RAM state inconsistent\"}");
      }
    } else {
      server.send(500, "application/json",
                  "{\"error\":\"EEPROM save failed\"}");
    }
  } else {
    server.send(400, "application/json",
                "{\"error\":\"Deletion failed\"}");
  }
}


void handleGetAllSensors() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }


  // Set CORS headers if needed
  server.sendHeader("Access-Control-Allow-Origin", "*");

  // Get the sensor data
  bool returnEverySensor = true;
  String sensorData = sensorManager.retrieveAllSensors(returnEverySensor);

  // Check if we got valid data
  if (sensorData.length() == 0 || sensorData == "{\"sensors\":[]}") {
    server.send(200, "application/json", "{\"status\":\"no sensors configured\"}");
    return;
  }

  // Send the response
  server.send(200, "application/json", sensorData);

  // Debug output
  if (DEBUG) {
    Serial.println("Sent sensor data:");
    Serial.println(sensorData);
  }

}

// POST request new firmware and reboot
void firmwareUploadPage() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Updating: %s\n", upload.filename.c_str());
    if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Update Success: %u bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}






void handleWinkRequest() {
  // visualizationOfError[WINK];

  String state = server.arg("state"); // les ?state=on/off
  if (state == "on") {
    digitalWrite(LED_BUILTIN, LOW);   // LED ON (ESP8266 aktiv lav)
    winkActive = true;
    server.send(200, "text/plain", "LED turned ON");
  } else if (state == "off") {
    digitalWrite(LED_BUILTIN, HIGH);  // LED OFF
    winkActive = false;
    server.send(200, "text/plain", "LED turned OFF");
  } else {
    server.send(400, "text/plain", "Invalid state");
  }
}



void handleBackupRequest() {
  Serial.println("HTTP GET /backup - Backup request received");

  // Generer backup JSON
  String backupJson = BackupManager::exportFullConfig();

  // Send som JSON response
  server.send(200, "application/json", backupJson);

  Serial.println("Backup JSON sent to client");
}

void handleBackupDownloadRequest() {
  Serial.println("Backup download request received");
  //updateBackupTimestamp(); // Oppdater tidspunkt
  String backupJson = BackupManager::exportFullConfig();
  server.sendHeader("Content-Disposition", "attachment; filename=config_backup.json");
  server.send(200, "application/json", backupJson);
}


// Global variabel for filopplasting
HTTPUpload& upload = server.upload();

void handleBackupUploadAndRestore() {
  Serial.println("HTTP POST /backup/upload - Backup upload request received");

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Upload start: %s\n", upload.filename.c_str());
  }
  else if (upload.status == UPLOAD_FILE_WRITE) {
    // Vi leser bytes direkte fra upload.buf
    Serial.printf("Uploading %u bytes...\n", upload.currentSize);
  }
  else if (upload.status == UPLOAD_FILE_END) {
    Serial.printf("Upload end: %s (%u bytes)\n", upload.filename.c_str(), upload.totalSize);

    // Konverter buffer til JSON-streng
    String uploadedJson;
    uploadedJson.reserve(upload.totalSize);
    for (size_t i = 0; i < upload.totalSize; i++) {
      uploadedJson += char(upload.buf[i]);
    }

    if (importEEPROMBackup(uploadedJson)) {
      if (BackupManager::backupToEEPROM()) {
        server.send(200, "application/json", "{\"status\":\"success\",\"message\":\"Backup restored successfully! Rebooting...\"}");
        delay(2000);
        ESP.restart();
      } else {
        server.send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to save to EEPROM\"}");
      }
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid backup file format\"}");
    }
  }
  else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No file uploaded\"}");
  }
}








/*
   Main server boilerplate
    Wifi, webserver, readings etc
*/

void checkMemory() {
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Heap fragmentation: %d%%\n", ESP.getHeapFragmentation());
  Serial.printf("Max free block: %d bytes\n", ESP.getMaxFreeBlockSize());
}

bool connectWiFi() {
  // Use the GLOBAL config, not a local one!
  if (strlen(config.network.wifiSSID) == 0) {
    Serial.println("No WiFi credentials configured");
    return false;
  }

  Serial.printf("\nAttempting to connect to: %s\n", config.network.wifiSSID);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(config.network.wifiSSID, config.network.wifiPassword);


  int attempts = 0;
  const int maxAttempts = 20;
  bool connected = false;

  while (attempts < maxAttempts) {
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    }
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (connected) {
    Serial.println("\nConnected successfully!");
    Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

    String hostname = generateAPName();
    if (MDNS.begin(hostname.c_str())) {
      Serial.printf("mDNS responder started: %s.local\n", hostname.c_str());
    } else {
      Serial.println("Error setting up mDNS responder!");
    }

    return true;
  } else {
    Serial.println("\nConnection failed. Status:");
    printWiFiStatus(); // Helper function to decode error (see below)
    return false;
  }
}

void setupServerRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config/wifi", HTTP_POST, handleWifiSettingsChange);
  server.on("/config/mqtt", HTTP_POST, handleMQTTSettingsChange);
  server.on("/config/operations", HTTP_POST, handleOperationsSettingsChange);
  server.on("/favicon.svg", HTTP_GET, []() {
    String svg = getLogoSvg();
    server.sendHeader("Cache-Control", "public, max-age=86400");
    server.send(200, "image/svg+xml", svg);
  });
  server.on("/config/sensor/types", HTTP_GET, handleSensorTypeRequest);
  server.on("/config/sensors", HTTP_GET, handleGetAllSensors);
  server.on("/config/sensor", HTTP_POST, handleSensorConfig);
  server.on("/config/sensor", HTTP_DELETE, handleDeleteSensor);
  server.on("/firmware", HTTP_GET, firmwareVersionPage);
  server.on("/firmware", HTTP_POST, []() {
    if (Update.hasError()) {
      server.send(501, "text/plain", "FAIL");
    } else {
      String html = "<html><head><meta http-equiv='refresh' content='3;url=/'></head>"
                    "<body>Update successful!</body></html>";
      server.send(200, "text/html", html);
      delay(3000);
      ESP.restart();
    }
  }, firmwareUploadPage);

  // Wink routes
  server.on("/config/wink", HTTP_GET, handleWinkRequest);

  // Backup routes
  server.on("/config/backup", HTTP_GET, backupPage);
  server.on("/backup", HTTP_GET, handleBackupRequest);
  server.on("/backup/download", HTTP_GET, handleBackupDownloadRequest);
  server.on("/backup/upload", HTTP_POST, handleBackupUploadAndRestore);


}


// IP: 192.168.4.1

void startConfigPortal() {
  String apName = generateAPName();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName.c_str());

  Serial.println("Started configuration portal");
  Serial.println("AP Name: " + apName);
  Serial.println("IP address: " + WiFi.softAPIP().toString());
  visualizationOfError(GENERAL_ERROR);

  setupServerRoutes();
  server.begin();
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nStarting Grefur Sensors v" + getFirmwareVersion());

  // Initialize EEPROM and load configuration FIRST
  EEPROM.begin(sizeof(EEPROMLayout));
  bool loadDefault = false;

  if (!loadConfig() || loadDefault) {
    Serial.println("Initializing with default config");
    memset(&config, 0, sizeof(config));
    config.header.magic = 0xAA55AA55;
    strncpy(config.header.version, FIRMWARE_VERSION, sizeof(config.header.version));


    saveConfig();  // Save the defaults
    visualizationOfError(GENERAL_ERROR);
  }

  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, HIGH); // Turn off initially

  // Define the pins used in the mux algorithm
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);

  testConfigIntegrity();  // Checking that all config is loaded and ready for usage
  checkMemory();

  //checkMeasurementReadiness();

  //sensorManager.initializeSensorSlots();

  // Try to connect to WiFi
  if (connectWiFi()) {
    mqttClient.setServer(config.mqtt.brokerURL, config.mqtt.port);
    mqttClient.setKeepAlive(60); // 60 second keepalive
    mqttClient.setSocketTimeout(30); // 30 second timeout

    setupServerRoutes();

    Serial.println("HTTP server started");
  } else {
    Serial.println("Config portal started...");
    startConfigPortal();
  }
  server.begin();

}

void loop() {
  server.handleClient();
  updateErrorVisualization();

  unsigned long currentTime = millis();

  static int lastWatchdogValue = 0;

  if (WiFi.status() == WL_CONNECTED) {
    MDNS.update();

    if (mqttClient.connected()) {

      mqttClient.loop();

      if (currentTime - lastPublishTime >= config.operation.publishInterval) {
        if (DEBUG) {
          Serial.println("Starting sensor-readings...");
          Serial.println("Admin password:" + String(config.device.adminPass));
        };

        if (!config.operation.publishWatchdog) {
          //publishWatchdog(mqttClient, lastWatchdogValue, generateAPName().c_str());
        }

        float readings[MAX_SENSORS_ALLOWED];
        takeMeasurements(readings);

        delay(10);

        for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
          float measuredValueAtIndex = readings[i];
          bool publishOnlyChangeRule = sensorManager.isPublishOnlyOnChange(i);
          bool isSame = sensorManager.sameValueLastTime(i, measuredValueAtIndex);

          if (!isnan(measuredValueAtIndex) && (!publishOnlyChangeRule || !isSame)) {
            String unit = sensorManager.getSensorUnit(i);
            String sensorName = sensorManager.getSensorName(i);

            publishMeasuredValue(mqttClient, measuredValueAtIndex, sensorName.c_str(), unit.c_str());
            sensorManager.setLastMeasurement(i, measuredValueAtIndex);

          }

          delay(10);
        }



        lastPublishTime = currentTime;

      }

    } else {
      reconnectMQTT(mqttClient);
    }

  } else {
    visualizationOfError(NO_INTERNET);

    // Handle WiFi reconnect if needed
    static unsigned long lastWifiCheck = 0;
    if (currentTime - lastWifiCheck > 10000) { // Check every 10 seconds
      lastWifiCheck = currentTime;
      Serial.println("WiFi disconnected, attempting reconnect...");
      WiFi.reconnect();
    }
  }

  // Small delay to prevent watchdog issues
  delay(10);
}
