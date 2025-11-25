//
// ============================================================================
// ANALOGUE DIESEL ENGINE MONITOR — ESP32 + SensESP 3.x
// Features:
//   • Coolant temperature sender (Teleflex/Faria US 240–33Ω, extended range)
//   • Up to 3 × DS18B20 sensors with persistent registry + SK paths
//   • ISR-based magnetic pickup RPM + tach-loss
//   • Fuel burn estimation via CurveInterpolator
//   • WiFi + SK config handled by SensESP
// ============================================================================

#include <Arduino.h>

// SensESP Core
#include "sensesp.h"
#include "sensesp_app_builder.h"

// Sensors / Transforms
#include "sensesp/sensors/analog_input.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/ema.h"
#include "sensesp/transforms/moving_average.h"

// Output
#include "sensesp/signalk/signalk_output.h"

// Config / FS
#include "sensesp/system/filesystem.h"

// OneWire + SensESP Dallas wrapper
// #include "sensesp_onewire/onewire_bus.h"
// #include "sensesp_onewire/dallas_temperature_sensors.h"

// ESP32 ADC calibration
#include "esp_adc_cal.h"

using namespace sensesp;

// ============================================================================
// CONSTANTS
// ============================================================================
static const uint8_t COOLANT_ADC_PIN = 34;
static const uint8_t ONE_WIRE_PIN    = 4;
static const uint8_t RPM_INPUT_PIN   = 33;

static const uint16_t RING_GEAR_TEETH = 116;
static const uint8_t  MAX_DS18 = 3;

// ============================================================================
// ADC with ESP calibration
// ============================================================================
class CalibratedADC : public RepeatSensor<float> {
 public:
  int pin;
  esp_adc_cal_characteristics_t cal;

  CalibratedADC(int pin, uint32_t interval_ms)
      : RepeatSensor<float>(interval_ms, [this]() {
          uint32_t raw = analogRead(pin);
          uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, &cal);
          return mv / 1000.0f;
        }),
        pin(pin) {

    analogReadResolution(12);
    analogSetPinAttenuation(pin, ADC_11db);

    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_11,
        ADC_WIDTH_BIT_12,
        1100,
        &cal);
  }
};

// ============================================================================
// Voltage Divider → Resistance
// ============================================================================
class VoltageToResistance : public Transform<float, float> {
 public:
  float r_fixed = 100000.0f;  // bottom resistor

  VoltageToResistance() : Transform<float, float>("volt2res") {}

  void set(const float& v_adc) override {
    if (v_adc < 0.02f || v_adc > 3.25f) {
      emit(NAN);
      return;
    }
    float vs = 3.3f;
    float rs = (r_fixed * (vs / v_adc)) - r_fixed;
    emit(rs);
  }
};

// ============================================================================
// Temperature sender curve interpolation (Kelvin output)
// ============================================================================
class TemperatureUSInterpreter : public CurveInterpolator {
 public:
  TemperatureUSInterpreter(String path = "")
    : CurveInterpolator(NULL, path) {

    clear_samples();
    add_sample({20,  410.00});
    add_sample({29.6,394.26});
    add_sample({40,  373.15});
    add_sample({55,  363.15});
    add_sample({70,  353.15});
    add_sample({100, 343.15});
    add_sample({112, 345.37});
    add_sample({131, 337.04});
    add_sample({140, 333.15});
    add_sample({207, 327.04});
    add_sample({300, 317.15});
    add_sample({400, 313.15});
    add_sample({450, 310.93});
    add_sample({750, 288.15});
    add_sample({900, 273.00});
  }
};

// ============================================================================
// Fuel burn curve (m³/s)
// ============================================================================
class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String path = "")
    : CurveInterpolator(NULL, path) {

    clear_samples();
    add_sample({500,  0.00000022});
    add_sample({1000, 0.00000022});
    add_sample({1500, 0.00000031});
    add_sample({1800, 0.00000036});
    add_sample({2000, 0.00000047});
    add_sample({2200, 0.00000056});
    add_sample({2400, 0.00000064});
    add_sample({2600, 0.00000075});
    add_sample({2800, 0.00000083});
    add_sample({3000, 0.00000111});
    add_sample({3200, 0.00000139});
    add_sample({3400, 0.00000167});
    add_sample({3800, 0.00000194});
    add_sample({3900, 0.00000200});
  }
};

// ============================================================================
// OneWire Registry (persistent)
// ============================================================================
class OneWireRegistry : public FileSystemSaveable {
 public:
  std::vector<String> addrs;
  std::vector<String> names;
  std::vector<String> sk_paths;

  OneWireRegistry() : FileSystemSaveable("onewire") {}

  bool to_json(JsonObject& root) override {
    JsonArray a = root.createNestedArray("sensors");
    for (size_t i = 0; i < addrs.size(); i++) {
      JsonObject o = a.add<JsonObject>();
      o["address"]  = addrs[i];
      o["name"]     = names[i];
      o["sk_path"]  = sk_paths[i];
    }
    return true;
  }

  bool from_json(const JsonObject& cfg) override {
    addrs.clear();
    names.clear();
    sk_paths.clear();

    if (!cfg["sensors"].is<JsonArray>()) return true;
    for (auto o : cfg["sensors"].as<JsonArray>()) {
      addrs.push_back(o["address"] | "");
      names.push_back(o["name"] | "");
      sk_paths.push_back(o["sk_path"] | "");
    }
    return true;
  }
};

OneWireRegistry registry;

// ============================================================================
// ISR RPM
// ============================================================================
volatile uint32_t rpm_pulses = 0;
volatile uint32_t last_us = 0;

void IRAM_ATTR rpm_isr() {
  uint32_t now = micros();
  if (now - last_us > 1500) {
    rpm_pulses++;
    last_us = now;
  }
}

class RPMCalc : public RepeatSensor<float> {
 public:
  RPMCalc(uint32_t interval_ms)
      : RepeatSensor<float>(interval_ms, [interval_ms]() {
          uint32_t p = rpm_pulses;
          rpm_pulses = 0;
          float revs = float(p) / float(RING_GEAR_TEETH);
          float rps  = revs / (interval_ms / 1000.0f);
          return rps * 60.0f;
        }) {}
};

// ============================================================================
// BUILD SUBSYSTEMS
// ============================================================================
void setup_coolant() {
  auto adc = std::make_shared<CalibratedADC>(COOLANT_ADC_PIN, 1000);
  auto v2r = std::make_shared<VoltageToResistance>();
  auto tcurve = std::make_shared<TemperatureUSInterpreter>();

  auto sk = std::make_shared<SKOutput<float>>(
      "propulsion.engine.coolantTemperature",
      "/Engine/Coolant");

  adc->connect_to(v2r)
     ->connect_to(tcurve)
     ->connect_to(sk);
}

void setup_onewire() {
  auto bus = std::make_shared<OneWireBus>(ONE_WIRE_PIN);
  auto sensors = std::make_shared<DallasTemperatureSensors>(bus);

  sensors->set_read_delay(1000);

  for (size_t i = 0; i < sensors->num_sensors() && i < MAX_DS18; i++) {

    String sk = (i < registry.sk_paths.size())
                  ? registry.sk_paths[i]
                  : "environment.temp.ds18." + String(i+1);

    auto skout = std::make_shared<SKOutput<float>>(sk,
                    "/DS18/" + String(i+1));

    // *** FIXED ***
    sensors->temperature(i)->connect_to(skout);
  }
}


void setup_rpm() {
  pinMode(RPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_INPUT_PIN), rpm_isr, RISING);

  auto rpm = std::make_shared<RPMCalc>(100);
  auto ema = std::make_shared<EMA>(0.3f);

  auto out = std::make_shared<SKOutput<float>>(
      "propulsion.engine.revolutions",
      "/Engine/RPM");

  rpm->connect_to(ema)->connect_to(out);

  // fuel burn
  auto fuel = std::make_shared<FuelInterpreter>();
  auto fuel_out = std::make_shared<SKOutput<float>>(
      "propulsion.engine.fuel.rate",
      "/Engine/FuelRate");

  rpm->connect_to(fuel)->connect_to(fuel_out);
}

// ============================================================================
// BUILD + SETUP
// ============================================================================
void build_system() {
  registry.load();
  setup_onewire();
  setup_coolant();
  setup_rpm();
}

void setup() {
  SetupLogging(ESP_LOG_INFO);
  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("engine-monitor")->get_app();

  build_system();
  sensesp_app->start();
}

void loop() {
  event_loop()->tick();
}
