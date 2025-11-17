//
// ============================================================================
// ENGINE MONITOR FIRMWARE — ESP32 + SensESP 3.x
// Supports:
//   • 1 × Analog Coolant Temp Sender (Teleflex/Faria US 240–33Ω or EU 450–33Ω)
//   • 1-Wire DS18B20 system (up to 3 sensors) with Web UI mapping
//   • Magnetic-pickup RPM with debounce + tach loss
//   • Full dynamic Web UI configuration stored in /config
//
// ============================================================================

#include <Arduino.h>

// SensESP Core
#include "sensesp.h"
#include "sensesp_app_builder.h"

// Sensors
#include "sensesp/sensors/analog_input.h"

// Transforms
#include "sensesp/transforms/ema.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/lambda_transform.h"

// Signal K Output
#include "sensesp/signalk/signalk_output.h"

// Config & Filesystem
#include "sensesp/system/filesystem.h"
#include "sensesp/system/valueconsumer.h"
#include "sensesp/system/observable.h"

// OneWire + DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

using namespace sensesp;

// ============================================================================
// CONSTANTS AND PIN ASSIGNMENTS
// ============================================================================

static const uint8_t COOLANT_ADC_PIN = 34;     // buffered ADC input
static const uint8_t ONE_WIRE_PIN = 4;         // DS18B20 bus
static const uint8_t RPM_INPUT_PIN = 33;       // op-isolated RPM pulses

static const uint16_t RING_GEAR_TEETH = 116;   // Yanmar 3JH3E / 3GM30F
static const uint8_t MAX_DS18 = 3;


// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

class VoltageToResistance;
class ResistanceToTemperature;
class CelsiusToKelvin;

class DS18Reader;
void discover_onewire_devices();
void setup_onewire_sensors();

class TachDebounceConfig;
void setup_rpm_system();

class CoolantSenderConfig;
void setup_coolant_system();

// ============================================================================
// COOLANT TEMPERATURE LOOKUP TABLES
// ============================================================================
//
// These were confirmed in the previous session.
//
//   US Sender (Teleflex/Faria): 240 → 33Ω
//   EU Sender (VDO):            450 → 33Ω
//
// ============================================================================

const std::vector<std::pair<float, float>> lookup_us_sender = {
    {240, 20}, {200, 30}, {150, 40}, {110, 50},
    {80, 60},  {60, 70},  {48, 80},  {38, 90},
    {33, 100}, {29,110},  {26,120}
};

const std::vector<std::pair<float, float>> lookup_eu_sender = {
    {450,20}, {360,30}, {290,40}, {240,50},
    {190,60}, {150,70}, {120,80}, {100,90},
    {80,100}, {60,110}, {45,120}
};


// ============================================================================
// VOLTAGE → RESISTANCE TRANSFORM
// Using confirmed R1 = 220kΩ, R2 = 100kΩ
// ============================================================================

class VoltageToResistance : public Transform<float, float> {
 public:
  float r1 = 220000.0f;   // confirmed as correct for your circuit
  float r2 = 100000.0f;

  VoltageToResistance() : Transform<float, float>("volt2res") {}

  bool to_json(JsonObject& root) override {
    root["r1"] = r1;
    root["r2"] = r2;
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    if (cfg["r1"].is<float>()) r1 = cfg["r1"].as<float>();
    if (cfg["r2"].is<float>()) r2 = cfg["r2"].as<float>();
    return true;
  }

  static String get_config_schema() {
    return R"JSON({
      "type":"object",
      "properties":{
        "r1":{"title":"R1 (ohms)", "type":"number"},
        "r2":{"title":"R2 (ohms)", "type":"number"}
      }
    })JSON";
  }

  void set(const float& v_adc) override {

    if (v_adc < 0.02f || v_adc > 3.25f) {
      emit(NAN);
      return;
    }

    // Solve divider
    // v_adc = 3.3 * (R2 / (R_sensor + R2))
    float vs = 3.3f;
    float rs = (r2 * (vs / v_adc)) - r2;

    emit(rs);
  }
};


// ============================================================================
// RESISTANCE → TEMPERATURE TRANSFORM
// ============================================================================

class ResistanceToTemperature : public Transform<float, float> {
 public:

  enum SenderType {
    US_240_33 = 0,
    EU_450_33 = 1
  };

  SenderType sender = US_240_33;

  ResistanceToTemperature() : Transform<float, float>("res2temp") {}

  bool to_json(JsonObject& root) override {
    root["sender"] = (int)sender;
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    if (cfg["sender"].is<int>()) {
      sender = (SenderType)cfg["sender"].as<int>();
    }
    return true;
  }

  static String get_config_schema() {
    return String("{\"type\":\"object\",\"properties\":{\"sender\":{\"title\":\"Coolant Sender Type\",\"type\":\"string\",\"enum\":[\"US_240_33\",\"EU_450_33\"]}}}");
  }

  const std::vector<std::pair<float,float>>& table() {
    return (sender == US_240_33) ? lookup_us_sender : lookup_eu_sender;
  }

  void set(const float& r) override {

    if (isnan(r)) {
      emit(NAN);
      return;
    }

    const auto& t = table();

    // clamp above/below table
    if (r >= t.front().first) { emit(t.front().second); return; }
    if (r <= t.back().first)  { emit(t.back().second);  return; }

    // interpolate
    for (size_t i=0; i<t.size()-1; i++) {
      float r1 = t[i].first;
      float t1 = t[i].second;
      float r2 = t[i+1].first;
      float t2 = t[i+1].second;

      if (r <= r1 && r >= r2) {
        float ratio = (r - r2) / (r1 - r2);
        float tc = t2 + ratio * (t1 - t2);
        emit(tc);
        return;
      }
    }

    emit(NAN);
  }
};


// ============================================================================
// °C → Kelvin
// ============================================================================

class CelsiusToKelvin : public Transform<float, float> {
 public:
  CelsiusToKelvin() : Transform<float,float>("c2k") {}

  void set(const float& c) override {
    emit(isnan(c) ? NAN : c + 273.15f);
  }
};


// ============================================================================
// COOLANT SENDER CONFIG (Persistent)
// ============================================================================

class CoolantSenderConfig : public FileSystemSaveable {
 public:
  ResistanceToTemperature::SenderType sender =
      ResistanceToTemperature::US_240_33;

  CoolantSenderConfig() : FileSystemSaveable("coolant") {}

  bool to_json(JsonObject& root) override {
    root["sender"] = (int)sender;
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    if (cfg["sender"].is<int>()) {
      sender = (ResistanceToTemperature::SenderType)cfg["sender"].as<int>();
    }
    return true;
  }
};

CoolantSenderConfig coolant_sender_cfg;
// ============================================================================
// SECTION 2 — DS18B20 SENSOR REGISTRY + DISCOVERY + READER
// ============================================================================
//
// Supports up to MAX_DS18 sensors.
// Stored in /config/onewire.json:
//
//  {
//    "sensors":[
//       {
//         "address":"28FF4A9123160456",
//         "name":"Exhaust Elbow",
//         "sk_path":"environment.exhaust.temperature"
//       },
//       ...
//    ]
//  }
//
// The registry is edited through the SensESP Web UI.
// ============================================================================


// ============================================================================
// Sensor registry entry
// ============================================================================

struct DS18Entry {
  String address_hex;     // 16-char ROM ID in hex
  String name;            // user-friendly sensor name
  String sk_path;         // Signal K output path

  DS18Entry()
      : address_hex("0000000000000000"),
        name("DS18B20 Sensor"),
        sk_path("environment.unknown.temperature") {}
};


// ============================================================================
// Persistent OneWire Registry (/config/onewire.json)
// ============================================================================

class OneWireRegistry : public FileSystemSaveable {
 public:
  std::vector<DS18Entry> entries;

  OneWireRegistry() : FileSystemSaveable("onewire") {}

  bool to_json(JsonObject& root) override {
    JsonArray arr = root["sensors"].to<JsonArray>();
    for (auto& e : entries) {
      JsonObject o = arr.add<JsonObject>();
      o["address"] = e.address_hex;
      o["name"] = e.name;
      o["sk_path"] = e.sk_path;
    }
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    if (!cfg["sensors"].is<JsonArray>()) return true;

    entries.clear();
    JsonArray arr = cfg["sensors"].as<JsonArray>();

    for (JsonObject o : arr) {
      DS18Entry e;
      e.address_hex = o["address"] | "";
      e.name        = o["name"]    | "";
      e.sk_path     = o["sk_path"] | "";
      entries.push_back(e);
    }
    return true;
  }

  static String get_config_schema() {
    return R"({
      "type":"object",
      "properties":{
        "sensors":{
          "title":"DS18B20 Sensors",
          "type":"array",
          "items":{
            "type":"object",
            "properties":{
              "address":{"title":"ROM Address","type":"string"},
              "name":{"title":"Name","type":"string"},
              "sk_path":{"title":"SignalK Path","type":"string"}
            }
          }
        }
      }
    })";
  }
};

OneWireRegistry onewire_registry;

// Provide ConfigSchema overloads for custom transforms so ConfigItem can
// generate UI schema correctly.
inline const String ConfigSchema(const VoltageToResistance& obj) {
  return VoltageToResistance::get_config_schema();
}

inline const String ConfigSchema(const ResistanceToTemperature& obj) {
  return ResistanceToTemperature::get_config_schema();
}
inline const String ConfigSchema(const CoolantSenderConfig& obj) {
  // reuse the ResistanceToTemperature schema for sender selection UI
  return String("{\"type\":\"object\",\"properties\":{\"sender\":{\"title\":\"Coolant Sender Type\",\"type\":\"string\",\"enum\":[\"US_240_33\",\"EU_450_33\"]}}}");
}


// ============================================================================
// Convert 8-byte ROM → 16-character hexadecimal string
// ============================================================================

String rom_to_hex(const DeviceAddress addr) {
  char buf[17];
  for (int i = 0; i < 8; i++) {
    sprintf(buf + i * 2, "%02X", addr[i]);
  }
  buf[16] = 0;
  return String(buf);
}


// ============================================================================
// Convert hex string → ROM address (8 bytes)
// ============================================================================

bool hex_to_rom(const String& hex, DeviceAddress addr) {
  if (hex.length() != 16) return false;

  for (int i = 0; i < 8; i++) {
    char hi = hex[i * 2];
    char lo = hex[i * 2 + 1];
    char buf[3] = { hi, lo, '\0' };
    addr[i] = (uint8_t)strtol(buf, nullptr, 16);
  }
  return true;
}


// ============================================================================
// DISCOVER DEVICES ON THE ONE-WIRE BUS
// ============================================================================
//
// Adds new devices to registry, does NOT remove missing devices.
// You may unplug sensors without losing configuration.
//
// ============================================================================

OneWire* ow_bus = nullptr;
DallasTemperature* ds_bus = nullptr;

void discover_onewire_devices() {
  ow_bus = new OneWire(ONE_WIRE_PIN);
  ds_bus = new DallasTemperature(ow_bus);

  ds_bus->begin();
  int count = ds_bus->getDeviceCount();

  debugI("OneWire: %d devices detected", count);

  DeviceAddress addr;

  for (int i = 0; i < count && i < MAX_DS18; i++) {
    if (!ds_bus->getAddress(addr, i)) continue;

    String hex = rom_to_hex(addr);
    bool found = false;

    for (auto& e : onewire_registry.entries) {
      if (e.address_hex == hex) {
        found = true;
        break;
      }
    }

    if (!found) {
      // New device found — add to registry
      DS18Entry e;
      e.address_hex = hex;

      // Default naming:
      e.name    = String("Sensor ") + String(onewire_registry.entries.size() + 1);
      e.sk_path = String("environment.temp.") +
                  String(onewire_registry.entries.size() + 1);

      onewire_registry.entries.push_back(e);
    }
  }

  onewire_registry.save();  // persist any additions
}


// ============================================================================
// DS18 READER — produces °C
// ============================================================================

class DS18Reader : public RepeatSensor<float> {
 public:
  DeviceAddress rom;

  DS18Reader(DeviceAddress address, int interval_ms)
      : RepeatSensor<float>(interval_ms, [this]() {
          if (ds_bus == nullptr) return NAN;
          ds_bus->requestTemperatures();
          float c = ds_bus->getTempC(this->rom);
          return (c == DEVICE_DISCONNECTED_C) ? NAN : c;
        }) {
    memcpy(rom, address, 8);
  }
};

// ============================================================================
// SECTION 3 — DS18 PIPELINES + COOLANT SYSTEM + RPM SYSTEM
// ============================================================================


// ============================================================================
// Build DS18 pipelines after registry is loaded
// ============================================================================

void setup_onewire_sensors() {
  for (auto& entry : onewire_registry.entries) {

    // Validate ROM
    if (entry.address_hex.length() != 16) {
      debugE("Invalid ROM: %s", entry.address_hex.c_str());
      continue;
    }

    // Convert hex → binary
    DeviceAddress rom;
    if (!hex_to_rom(entry.address_hex, rom)) {
      debugE("ROM conversion failed: %s", entry.address_hex.c_str());
      continue;
    }

    // Create DS18B20 reader (°C)
    auto reader = std::make_shared<DS18Reader>(rom, 1000);

    // Convert °C → Kelvin
    auto to_kelvin = std::make_shared<CelsiusToKelvin>();

    // SK temperature output
    auto sk_temp = std::make_shared<SKOutput<float>>(
        entry.sk_path.c_str(),
        String("/Sensors/") + entry.name + "/Temperature"
    );

    // Pipeline:
    //   DS18 → (°C) → Kelvin → SK
    reader->connect_to(to_kelvin)->connect_to(sk_temp);

    debugI("DS18 %s mapped to %s (%s)",
           entry.address_hex.c_str(),
           entry.name.c_str(),
           entry.sk_path.c_str());
  }
}



// ============================================================================
// COOLANT TEMPERATURE SYSTEM
// ============================================================================
//
// Pipeline:
//
//    ADC → VoltageToResistance → ResistanceToTemperature → CelsiusToKelvin → SK
//
// ============================================================================

void setup_coolant_system() {

  // 1) Raw ADC voltage
  auto adc = std::make_shared<AnalogInput>(
      COOLANT_ADC_PIN,
      1000,    // sampling interval
      "",
      3.3f     // scale 1:1
  );

  // 2) Voltage → Resistance using known R1=220kΩ, R2=100kΩ
  auto v2r = std::make_shared<VoltageToResistance>();
  ConfigItem(v2r)->set_title("Coolant Divider R1/R2");

  // 3) Resistance → Temperature °C based on sender type
  auto r2t = std::make_shared<ResistanceToTemperature>();
  r2t->sender = coolant_sender_cfg.sender;
  ConfigItem(r2t)->set_title("Coolant Sender Type");

  // 4) °C → Kelvin
  auto to_kelvin = std::make_shared<CelsiusToKelvin>();

  // 5) SK Output
  auto sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.engine.coolantTemperature",
      "/Engine/CoolantTemperature"
  );

  // Connect the pipeline
  adc->connect_to(v2r)->connect_to(r2t)->connect_to(to_kelvin)->connect_to(sk_output);

  debugI("Coolant temperature system initialized.");
}



// ============================================================================
// RPM SYSTEM
// ============================================================================
//
// Includes:
//   • Persistent debounce config (/config/rpm.json)
//   • ISR-based pulse counting
//   • RPM calculator (10 Hz)
//   • EMA smoothing
//   • Tach-loss detection
// ============================================================================


// ============================================================================
// Persistent Debounce Config
// ============================================================================

class TachDebounceConfig : public FileSystemSaveable {
 public:
  uint32_t debounce_us = 1500;

  TachDebounceConfig() : FileSystemSaveable("rpm") {}

  bool to_json(JsonObject& root) override {
    root["debounce_us"] = debounce_us;
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    if (cfg["debounce_us"].is<uint32_t>()) {
      debounce_us = cfg["debounce_us"].as<uint32_t>();

      // safety limits
      if (debounce_us < 200) debounce_us = 200;
      if (debounce_us > 20000) debounce_us = 20000;
    }
    return true;
  }

  static String get_config_schema() {
    return String("{\"type\":\"object\",\"properties\":{\"debounce_us\":{\"title\":\"RPM Debounce (microseconds)\",\"type\":\"number\"}}}");
  }
};

TachDebounceConfig tach_debounce_cfg;

// Provide ConfigSchema overload for TachDebounceConfig now that the type is
// fully defined.
inline const String ConfigSchema(const TachDebounceConfig& obj) {
  return TachDebounceConfig::get_config_schema();
}


// ============================================================================
// ISR pulse counting
// ============================================================================

volatile uint32_t rpm_pulse_count = 0;
volatile uint32_t rpm_last_us = 0;

void IRAM_ATTR rpm_isr() {
  uint32_t now = micros();
  if ((now - rpm_last_us) >= tach_debounce_cfg.debounce_us) {
    rpm_pulse_count++;
    rpm_last_us = now;
  }
}


// ============================================================================
// Tach-loss detector
// ============================================================================

class TachLossDetector : public RepeatSensor<bool> {
 public:
  TachLossDetector(int interval_ms) : RepeatSensor<bool>(interval_ms, [this]() {
      static uint32_t last_seen = millis();
      if (rpm_pulse_count > 0) last_seen = millis();
      return ((millis() - last_seen) > 2000);
    }) {}
};


// ============================================================================
// RPM Calculator
// ============================================================================

class RPMCalculator : public RepeatSensor<float> {
 public:
  RPMCalculator(int interval_ms) : RepeatSensor<float>(interval_ms, [this, interval_ms]() {
      uint32_t pulses = rpm_pulse_count;
      rpm_pulse_count = 0;
      float sample_period = (float)interval_ms / 1000.0f;  // ms → seconds
      float revs = float(pulses) / float(RING_GEAR_TEETH);
      float rps = revs / sample_period;
      return rps * 60.0f;  // RPS → RPM
    }) {}
};


// ============================================================================
// RPM system builder
// ============================================================================

void setup_rpm_system() {

  pinMode(RPM_INPUT_PIN, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(RPM_INPUT_PIN),
      rpm_isr,
      RISING
  );

  // raw RPM @ 10Hz
  auto rpm_raw = std::make_shared<RPMCalculator>(100);

  auto sk_rpm_raw = std::make_shared<SKOutput<float>>(
      "sensors.engine.rpm.raw",
      "/Debug/Engine/RPMRaw"
  );
  rpm_raw->connect_to(sk_rpm_raw);

  // smoothing
  auto rpm_smooth = std::make_shared<EMA>(0.3f);
  ConfigItem(rpm_smooth)->set_title("RPM Smoothing (EMA)");

  auto sk_rpm = std::make_shared<SKOutput<float>>(
      "propulsion.engine.revolutions",
      "/Engine/Revolutions"
  );
  rpm_raw->connect_to(rpm_smooth)->connect_to(sk_rpm);

  // tach loss
  auto tach_loss = std::make_shared<TachLossDetector>(500);
  auto sk_tach_loss = std::make_shared<SKOutput<bool>>(
      "propulsion.engine.tachLoss",
      "/Engine/TachLoss"
  );

  tach_loss->connect_to(sk_tach_loss);

  ConfigItem(&tach_debounce_cfg)->set_title("RPM Debounce");

  debugI("RPM system initialized.");
}
// ============================================================================
// SECTION 4 — SYSTEM BUILDER + SETUP() + LOOP()
// ============================================================================
//
// This ties together:
//    • OneWire registry + DS18B20 pipelines
//    • Coolant temperature system
//    • RPM system
//    • WiFi + Web UI
//    • Signal K WebSocket client
//
// ============================================================================


// ============================================================================
// SYSTEM BUILDER — called from setup()
// ============================================================================

void build_engine_monitor_system() {

  debugI("=== Engine Monitor: Loading configuration ===");

  // Filesystem/config is initialized by SensESP app; individual config
  // objects will be loaded below.

  // Load persistent configuration for each subsystem:
  coolant_sender_cfg.load();
  onewire_registry.load();
  tach_debounce_cfg.load();

  debugI("=== Discovering OneWire sensors ===");
  discover_onewire_devices();

  debugI("=== Setting up DS18 pipelines ===");
  setup_onewire_sensors();

  debugI("=== Setting up coolant sender system ===");
  setup_coolant_system();

  debugI("=== Setting up RPM system ===");
  setup_rpm_system();

  debugI("=== Engine Monitor system ready ===");
}


// ============================================================================
// SETUP — called once at boot
// ============================================================================

void setup() {

  // Start logging
  SetupLogging(ESP_LOG_DEBUG);
  delay(100);

  debugI("===== Engine Monitor Boot =====");

  // SensESP Application Builder
  SensESPAppBuilder builder;

  //
  // WiFi + Signal K configuration:
  //
  // • User can edit WiFi SSID + password from Web UI
  // • User can edit Signal K server IP + port from Web UI
  //
  // Configuration stored in:
  //   /config/system/networking.json
  //   /config/system/skserver.json
  //

    // Create the SensESP application object. Keep configuration minimal here;
    // the web UI will expose more settings at runtime.
    sensesp_app = builder.set_hostname("engine-monitor")->get_app();

  debugI("=== SensESP base app created ===");

  // Build all subsystems
  build_engine_monitor_system();

  // Start SensESP networking, filesystem watchers, SK client, etc.
  sensesp_app->start();

  debugI("===== Engine Monitor Ready =====");
}


// ============================================================================
// LOOP — must call tick()
// ============================================================================

void loop() {
  event_loop()->tick();
}
