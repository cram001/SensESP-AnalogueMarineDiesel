//
// ============================================================================
// ANALOGUE DIESEL ENGINE MONITOR — ESP32 + SensESP 3.x
// ============================================================================
// Features:
//   • Coolant temp via Teleflex/Faria 450–30Ω sender (ADC → volts → Ω → K)
//   • Up to 3 × DS18B20 sensors (OneWire v3.x, SensESP)
//   • Engine RPM via magnetic pickup ISR + RepeatSensor<float>
//   • Fuel burn estimation via CurveInterpolator + smoothing
//   • Automatic publishing to Signal K
//   • OTA update page at /ota
//   • OTA rollback protection
//
// Developer Notes:
//   • Designed for SensESP 3.x — NOT compatible with 2.x.
//   • RepeatSensor<T> returns values, Transform<T> emits values.
//   • Avoid floating point in ISR. Keep it tiny.
// ============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// SensESP Core
// ---------------------------------------------------------------------------
#include "sensesp.h"
#include "sensesp_app_builder.h"
#include "sensesp/system/filesystem.h"

// Transforms / Outputs
#include "sensesp/sensors/sensor.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/ema.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/signalk/signalk_output.h"

// Analog Input
#include "sensesp/sensors/analog_input.h"

// OneWire subsystem
#include "sensesp_onewire/onewire_temperature.h"

// ESP32 ADC calibration
#include "esp_adc_cal.h"

// OTA support
#include <WebServer.h>
#include "esp_ota_ops.h"


// ============================================================================
// GLOBALS
// ============================================================================
std::shared_ptr<sensesp::SensESPApp> sensesp_app;
WebServer ota_server(80);


// ============================================================================
// HARDWARE CONSTANTS
// ============================================================================
static const uint8_t COOLANT_ADC_PIN = 34;
static const uint8_t ONE_WIRE_PIN    = 4;
static const uint8_t RPM_INPUT_PIN   = 33;

static const uint16_t RING_GEAR_TEETH = 116;


// ============================================================================
// CALIBRATED ADC SENSOR (returns Volts)
// ============================================================================
class CalibratedADC : public sensesp::RepeatSensor<float> {
 public:
  int adc_pin;
  esp_adc_cal_characteristics_t cal;

  CalibratedADC(int adc_pin, uint32_t interval_ms)
      : adc_pin(adc_pin),
        sensesp::RepeatSensor<float>(
            interval_ms,
            [this]() -> float {
              uint32_t raw = analogRead(this->adc_pin);
              uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, &this->cal);
              return mv / 1000.0f;  // mV → V
            }) 
  {
    analogReadResolution(12);
    analogSetPinAttenuation(adc_pin, ADC_11db);

    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_11,
        ADC_WIDTH_BIT_12,
        1100,   // mV reference
        &cal);
  }
};


// ============================================================================
// VOLTAGE → RESISTANCE (Thermistor divider conversion)
// ============================================================================
class VoltageToResistance : public sensesp::Transform<float, float> {
 public:
  float r_fixed = 100000.0f; // 100k resistor

  VoltageToResistance() : sensesp::Transform<float, float>("volt2res") {}

  void set(const float& v_adc) override {
    if (v_adc < 0.02f || v_adc > 3.25f) {
      emit(NAN);
      return;
    }
    const float vs = 3.3f;
    float rs = (r_fixed * (vs / v_adc)) - r_fixed;
    emit(rs);
  }
};


// ============================================================================
// TEMPERATURE CURVE (Ω → K)
// ============================================================================
class TemperatureUSInterpreter : public sensesp::CurveInterpolator {
 public:
  TemperatureUSInterpreter(String path = "")
      : sensesp::CurveInterpolator(NULL, path) 
  {
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
// FUEL BURN CURVE (RPM → m³/s) + smoothing added later
// ============================================================================
class FuelInterpreter : public sensesp::CurveInterpolator {
 public:
  FuelInterpreter(String path = "")
      : sensesp::CurveInterpolator(NULL, path)
  {
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
// RPM READING VIA ISR + RepeatSensor<float>
// ============================================================================
volatile uint32_t rpm_pulses = 0;
volatile uint32_t last_us = 0;

void IRAM_ATTR rpm_isr() {
  uint32_t now = micros();
  if (now - last_us > 100) {
    rpm_pulses++;
    last_us = now;
  }
}

class RPMCalc : public sensesp::RepeatSensor<float> {
 public:
  uint32_t interval_ms;

  RPMCalc(uint32_t interval_ms)
      : interval_ms(interval_ms),
        sensesp::RepeatSensor<float>(
            interval_ms,
            [this, interval_ms]() -> float {
              uint32_t p = rpm_pulses;
              rpm_pulses = 0;

              float revs = float(p) / float(RING_GEAR_TEETH);
              float rpm = (revs / (float(interval_ms) / 1000.0f)) * 60.0f;

              return rpm;
            }) {}
};


// ============================================================================
// COOLANT SENSOR SETUP
// ============================================================================
void setup_coolant() {

  auto adc   = std::make_shared<CalibratedADC>(COOLANT_ADC_PIN, 1000);
  auto v2r   = std::make_shared<VoltageToResistance>();
  auto curve = std::make_shared<TemperatureUSInterpreter>();

  auto sk = std::make_shared<sensesp::SKOutputFloat>(
      "propulsion.engine.coolantTemperature",
      "/Engine/CoolantTemperature");

  adc->connect_to(v2r)->connect_to(curve)->connect_to(sk);
}


// ============================================================================
// ONEWIRE TEMP SETUP (3 sensors)
// ============================================================================
void setup_onewire() {

  auto* dts = new sensesp::onewire::DallasTemperatureSensors(ONE_WIRE_PIN);
  uint read_delay = 1000;

  for (int i = 1; i <= 3; i++) {
    String path = "/OneWire/S" + String(i);

    auto t  = new sensesp::onewire::OneWireTemperature(dts, read_delay, path);
    auto sk = new sensesp::SKOutputFloat(
        "environment.sensors.onewire." + String(i),
        path + "/SK");

    t->connect_to(sk);
  }
}


// ============================================================================
// RPM + FUEL OUTPUT SETUP
// ============================================================================
void setup_rpm() {

  pinMode(RPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_INPUT_PIN), rpm_isr, RISING);

  auto rpm = std::make_shared<RPMCalc>(100);
  auto ema = std::make_shared<sensesp::EMA>(0.3);

  auto skrpm = std::make_shared<sensesp::SKOutputFloat>(
      "propulsion.engine.revolutions",
      "/Engine/RPM");

  rpm->connect_to(ema)->connect_to(skrpm);

  auto fuel         = std::make_shared<FuelInterpreter>();
  auto fuel_smooth  = std::make_shared<sensesp::EMA>(0.2);    // fuel smoothing
  auto skfuel       = std::make_shared<sensesp::SKOutputFloat>(
      "propulsion.engine.fuel.rate",
      "/Engine/FuelRate");

  rpm->connect_to(fuel)->connect_to(fuel_smooth)->connect_to(skfuel);
}


// ============================================================================
// BUILD ALL SUBSYSTEMS
// ============================================================================
void build_system() {
  setup_coolant();
  setup_onewire();
  setup_rpm();
}


// ============================================================================
// OTA WEB UI
// ============================================================================
void setup_ota_web() {

  ota_server.on("/", HTTP_GET, []() {
    ota_server.send(200, "text/html",
      "<h2>Engine Monitor OTA Update</h2>"
      "<form method='POST' action='/update' enctype='multipart/form-data'>"
      "<input type='file' name='firmware'/>"
      "<input type='submit' value='Update Firmware'/>"
      "</form>");
  });

  ota_server.on("/update", HTTP_POST,
    []() {
      bool ok = !Update.hasError();
      ota_server.send(200, "text/plain", ok ? "OK" : "FAIL");
      delay(500);
      if (ok) ESP.restart();
    },
    []() {
      HTTPUpload& up = ota_server.upload();
      if (up.status == UPLOAD_FILE_START) Update.begin();
      else if (up.status == UPLOAD_FILE_WRITE) Update.write(up.buf, up.currentSize);
      else if (up.status == UPLOAD_FILE_END) Update.end(true);
    });

  ota_server.begin();
}


// ============================================================================
// MAIN SETUP
// ============================================================================
void setup() {

  sensesp::SensESPAppBuilder builder;
  builder.set_hostname("engine-monitor");

  sensesp_app = builder.get_app();

  build_system();      // initialize sensors/transforms
  setup_ota_web();     // start OTA
  sensesp_app->start();

  // OTA rollback protection
  esp_ota_mark_app_valid_cancel_rollback();
}


// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  sensesp::event_loop()->tick();
  ota_server.handleClient();
}
