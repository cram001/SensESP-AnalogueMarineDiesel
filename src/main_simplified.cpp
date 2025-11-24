//
// ============================================================================
// ANALOGUE DIESEL ENGINE MONITOR FIRMWARE — ESP32 + SensESP 3.x
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
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/moving_average.h"

// Sensors
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"

// Transforms
#include "sensesp/transforms/ema.h"
#include "sensesp/transforms/curveinterpolator.h"

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

// ADC with ESP-IDF calibration wrapper
#include "esp_adc_cal.h"

// Calibrate the ADC

class CalibratedADC : public RepeatSensor<float> {
 public:
  int pin;
  adc1_channel_t channel;
  esp_adc_cal_characteristics_t cal;

  CalibratedADC(int pin, int interval_ms)
      : RepeatSensor<float>(interval_ms, [this]() {
          uint32_t raw = analogRead(this->pin);
          uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &this->cal);
          return float(mv) / 1000.0f;
        }),
        pin(pin) {

    // Convert GPIO → ADC1 channel (Valid only for ADC1: pins 32–39)
    channel = (adc1_channel_t)(pin - 32);

    // Configure ADC width and attenuation
    analogReadResolution(12);                 // 12-bit ADC
    analogSetPinAttenuation(pin, ADC_11db);   // Full scale ≈ 3.3V

    // Characterize using eFuse or default
    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_11,
        ADC_WIDTH_BIT_12,
        1100,         // default Vref if not in eFuse
        &cal
    );
  }
};

// ============================================================================
// FUEL FLOW INTERPOLATOR Yanmar 3JH3E with 18x11 3 blade propeller on 21 000 lbs sailboat
// ============================================================================

class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to m3/s
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, m3/s));
    add_sample(CurveInterpolator::Sample(500, 0.00000022));
    add_sample(CurveInterpolator::Sample(1000, 0.00000022));
    add_sample(CurveInterpolator::Sample(1500, 0.00000031));
    add_sample(CurveInterpolator::Sample(1800, 0.00000036));
    add_sample(CurveInterpolator::Sample(2000, 0.00000047));
    add_sample(CurveInterpolator::Sample(2200, 0.00000056));
    add_sample(CurveInterpolator::Sample(2400, 0.00000064));
    add_sample(CurveInterpolator::Sample(2600, 0.00000075));
    add_sample(CurveInterpolator::Sample(2800, 0.00000083));
    add_sample(CurveInterpolator::Sample(3000, 0.00000111));
    add_sample(CurveInterpolator::Sample(3200, 0.00000139));
    add_sample(CurveInterpolator::Sample(3400, 0.00000167));
    add_sample(CurveInterpolator::Sample(3800, 0.00000194));  
    add_sample(CurveInterpolator::Sample(3900, 0.00000200));  
  }
};

class TemperatureUSInterpreter : public CurveInterpolator {
 public:
  TemperatureUSInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our temperature sender to degrees Kelvin
    // (300-23 ohm european 400-30 ohm american)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin)) American Sender;
    add_sample(CurveInterpolator::Sample(20, 410.00));
    add_sample(CurveInterpolator::Sample(29.6, 394.26));
    add_sample(CurveInterpolator::Sample(40, 373.15));
    add_sample(CurveInterpolator::Sample(55, 363.15));
    add_sample(CurveInterpolator::Sample(70, 353.15));
    add_sample(CurveInterpolator::Sample(100, 343.15));
    add_sample(CurveInterpolator::Sample(112, 345.37));
    add_sample(CurveInterpolator::Sample(131, 337.04));
    add_sample(CurveInterpolator::Sample(140, 333.15));
    add_sample(CurveInterpolator::Sample(207, 327.04));
    add_sample(CurveInterpolator::Sample(300, 317.15));
    add_sample(CurveInterpolator::Sample(400, 313.15)); 
    add_sample(CurveInterpolator::Sample(450, 310.93)); 
    add_sample(CurveInterpolator::Sample(750, 288.15)); 
    add_sample(CurveInterpolator::Sample(900, 273.00 )); 
    
  }
};


// ============================================================================
// COOLANT TEMPERATURE LOOKUP TABLES (PROGMEM — flash-optimized)
// ============================================================================
//
//   US Sender (Teleflex/Faria): 240 → 33Ω
//   EU Sender (VDO):            450 → 33Ω
//
// ============================================================================

// Lookup tables in regular memory (simpler, maintains functionality)
static const float lookup_us_sender_r[] = {240, 200, 150, 110, 80, 60, 48, 38, 33, 29, 26};
static const float lookup_us_sender_t[] = {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120};
static const float lookup_eu_sender_r[] = {450, 360, 290, 240, 190, 150, 120, 100, 80, 60, 45};
static const float lookup_eu_sender_t[] = {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120};
static const size_t LOOKUP_TABLE_SIZE = 11;

// ============================================================================
// VOLTAGE → RESISTANCE TRANSFORM
// Voltage divider: R1 = 220kΩ, R2 = 100kΩ coolant temp sender
// ============================================================================

class VoltageToResistance : public Transform<float, float> {
 public:
  float r1 = 220000.0f;   
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
    return "{\"type\":\"object\",\"properties\":{\"r1\":{\"title\":\"R1 (ohms)\",\"type\":\"number\"},\"r2\":{\"title\":\"R2 (ohms)\",\"type\":\"number\"}}}";
  }

  //error trapping for out-of-bounds voltage
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

const float Vin = 3.5;
const float R1 = 120.0;
auto* analog_input = new AnalogInput(36, 2000);

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new TemperatureUSInterpreter("/Engine Temp/curve"))
      ->connect_to(new Linear(1.0, 0.9, "/Engine Temp/calibrate"))
      ->connect_to(new MovingAverage(4, 1.0,"/Engine Temp/movingAVG"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature", "/Engine Temp/sk_path"));

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature.raw"));

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
// Supports up to 3 DS18B20 sensors.
// SignalK config stored in /config/onewire.json:
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

  DS18Entry() {
  address_hex = "";
  name = "";
  sk_path = "";
  }
};


// ============================================================================
// Persistent OneWire Registry (/config/onewire.json)
// ============================================================================

class OneWireRegistry : public FileSystemSaveable {
 public:
  std::vector<DS18Entry> entries;

  OneWireRegistry() : FileSystemSaveable("onewire") {}

  bool to_json(JsonObject& root) override {
   // JsonArray arr = root["sensors"].to<JsonArray>();
   JsonArray arr = root.createNestedArray("sensors"); 
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
      //e.address_hex = o["address"] | "";
      if (o["address"].is<const char*>()) e.address_hex = o["address"].as<const char*>();
else e.address_hex = "";
      
      e.name        = o["name"]    | "";
      e.sk_path     = o["sk_path"] | "";
      entries.push_back(e);
    }
    return true;
  }

  static String get_config_schema() {
    return "{\"type\":\"object\",\"properties\":{\"sensors\":{\"title\":\"DS18B20 Sensors\",\"type\":\"array\",\"items\":{\"type\":\"object\",\"properties\":{\"address\":{\"title\":\"ROM Address\",\"type\":\"string\"},\"name\":{\"title\":\"Name\",\"type\":\"string\"},\"sk_path\":{\"title\":\"SignalK Path\",\"type\":\"string\"}}}}}}";
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
  return String("{\"type\":\"object\",\"properties\":{\"sender\":{\"title\":\"Coolant Sender Type\",\"type\":\"string\",\"enum\":[\"US_240_33\",\"EU_450_33\"]}}}");
}

// ============================================================================
// Convert 8-byte ROM → 16-character hexadecimal string
// ============================================================================

String rom_to_hex(const DeviceAddress addr) {
  char buf[17];
  for (int i = 0; i < 8; i++) {
    snprintf(buf + i * 2, 3, "%02X", addr[i]);
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

      // Default naming: use snprintf for efficient string building
      char name_buf[32];
      char path_buf[64];
      snprintf(name_buf, sizeof(name_buf), "Sensor %zu", onewire_registry.entries.size() + 1);
      snprintf(path_buf, sizeof(path_buf), "environment.temp.%zu", onewire_registry.entries.size() + 1);
      e.name = String(name_buf);
      e.sk_path = String(path_buf);

      onewire_registry.entries.push_back(e);
    }
  }

  onewire_registry.save();  // persist any additions
}



// ============================================================================
// DS18B20 READER — staggered reads using DallasTemperature
// Each sensor is read once per cycle; sensors are offset by 1s so that
// at most one sensor performs a conversion at any second.
// ============================================================================

class DS18Reader : public RepeatSensor<float> {
 public:
  DeviceAddress rom;
  uint32_t offset_ms = 0;

  DS18Reader(DeviceAddress address, uint32_t interval_ms, uint32_t offset = 0)
      : RepeatSensor<float>(interval_ms, [this]() { return this->do_read(); }),
        offset_ms(offset) {
    memcpy(rom, address, 8);
    // Schedule first read at offset (so sensors are staggered)
    if (offset_ms > 0) {
      event_loop()->onDelay(offset_ms, [this]() { this->do_read(); });
    }
  }

  float do_read() {
    if (ds_bus == nullptr) return NAN;
    // Request temperature (this triggers conversion on the bus)
    ds_bus->requestTemperatures();
    float c = ds_bus->getTempC(this->rom);
    return (c == DEVICE_DISCONNECTED_C) ? NAN : c;
  }
};

// ============================================================================
// SECTION 3 — DS18B20 PIPELINES + COOLANT SYSTEM + RPM SYSTEM
// ============================================================================


// ============================================================================
// Build DS18B20 pipelines after registry is loaded
// ============================================================================

void setup_onewire_sensors() {
  size_t total = onewire_registry.entries.size();
  if (total == 0) return;
  for (size_t idx = 0; idx < total; ++idx) {
    auto& entry = onewire_registry.entries[idx];

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

    // Ensure we have a valid Signal K path. If empty, assign a default
    // path so SensESP does not register an empty key. Persist the change.
    {
      String sk = entry.sk_path;
      sk.trim();
      if (sk.length() == 0) {
        String default_sk = String("environment.temp.unknown") + String(idx + 1);
        entry.sk_path = default_sk;
        onewire_registry.save();
        debugW("OneWire entry %s had empty sk_path; assigned default %s",
               entry.address_hex.c_str(), entry.sk_path.c_str());
      }
    }

    // Create DS18B20 reader (°C) with stagger: one sensor read per second
    uint32_t interval_ms = uint32_t(total * 1000); // each sensor read every total seconds
    uint32_t offset_ms = uint32_t(idx * 1000);     // stagger reads by 1s per sensor
    auto reader = std::make_shared<DS18Reader>(rom, interval_ms, offset_ms);

    // Convert °C → Kelvin
    auto to_kelvin = std::make_shared<CelsiusToKelvin>();

    // Pre-build display path: "/Sensors/{name}/Temperature"
    String display_path = "/Sensors/" + entry.name + "/Temperature";

    // SK temperature output
    auto sk_temp = std::make_shared<SKOutput<float>>(
        entry.sk_path.c_str(),
        display_path
    );

    // Pipeline:
    //   DS18 → (°C) → Kelvin → SK
    reader->connect_to(to_kelvin)->connect_to(sk_temp);

    debugI("DS18 %s -> %s",
           entry.address_hex.c_str(),
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
    auto adc = std::make_shared<CalibratedADC>(
      COOLANT_ADC_PIN,
      2000    // sampling interval
    );

  // 2) Voltage → Resistance using known R1=220kΩ, R2=100kΩ
  auto v2r = std::make_shared<VoltageToResistance>();
  ConfigItem(v2r)->set_title("Coolant Divider R1/R2");

  // 3) Resistance → Temperature °C based on sender type
  auto r2t = std::make_shared<ResistanceToTemperature>();
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
  return String("{\"type\":\"object\",\"properties\":{\"debounce_us\":{\"title\":\"RPM Debounce (microseconds)\",\"type\":\"number\"}}}");
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
  // Setup RPM input pin with ISR-based interrupt for high-frequency pulse counting
  // Using attachInterrupt for real-time responsiveness required for tachometer
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

  debugI("Loading configuration...");

  // Filesystem/config is initialized by SensESP app; individual config
  // objects will be loaded below.

  // Load persistent configuration for each subsystem:
  coolant_sender_cfg.load();
  onewire_registry.load();
  tach_debounce_cfg.load();

  discover_onewire_devices();
  setup_onewire_sensors();
  setup_coolant_system();
  setup_rpm_system();

  debugI("Engine Monitor ready");
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

// fuel flow to SignalK


  auto* sensor = new DigitalInputCounter(RPM_INPUT_PIN, INPUT_PULLUP, RISING, 500);

  sensor->connect_to(new Frequency(6))
  // times by 6 to go from Hz to RPM
          ->connect_to(new MovingAverage(4, 1.0,"/Engine Fuel/movingAVG"))
          ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
          ->connect_to(new SKOutputFloat("propulsion.engine.fuel.rate", "/Engine Fuel/sk_path"));  


  debugI("===== Engine Monitor Ready =====");

}


// ============================================================================
// LOOP — must call tick()
// ============================================================================

void loop() {
  event_loop()->tick();
}
