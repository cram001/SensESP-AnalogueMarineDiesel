//
// ============================================================================
// ANALOGUE DIESEL ENGINE MONITOR — ESP32 + SensESP 3.x
// ============================================================================
// Features:
//   • Coolant temp via Teleflex/Faria 240–33Ω sender (ADC → volts → Ω → K)
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

// SensESP Core
#include "sensesp.h"
#include "sensesp_app_builder.h"
#include "sensesp/system/filesystem.h"

// Transforms / outputs
#include "sensesp/transforms/transform.h"
#include "sensesp/transforms/ema.h"
#include "sensesp/signalk/signalk_output.h"

// OneWire subsystem
#include "sensesp_onewire/onewire_temperature.h"

// Persistent value
#include "sensesp/system/observablevalue.h"

// ADC calibration
#include "esp_adc_cal.h"

// OTA Support
#include <WebServer.h>
#include <Update.h>
#include "esp_ota_ops.h"

using namespace sensesp;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

WebServer ota_server(80);

// persists internally with 2 decimals resolution
std::shared_ptr<PersistingObservableValue<float>> engine_hours;

static float g_last_rpm = 0.0f;

// ---------------------------------------------------------------------------
// Hardware constants (CHANGE PIN ASSIGNMENTS AS NEEDED, REFER
// TO ESP32 DOCUMENTATION BEFORE CHANGING PINT OUT)
// ---------------------------------------------------------------------------
static const uint8_t COOLANT_ADC_PIN = 34;
static const uint8_t ONE_WIRE_PIN    = 4;
static const uint8_t RPM_INPUT_PIN   = 33;

static const uint16_t RING_GEAR_TEETH = 116;

// ============================================================================
// LINEAR INTERPOLATION ENGINE
// ============================================================================

float linear_interp(float x,
                    const float* xv,
                    const float* yv,
                    size_t n) {

  if (x <= xv[0])     return yv[0];
  if (x >= xv[n-1])   return yv[n-1];

  for (size_t i = 0; i < n - 1; i++) {
    float x0 = xv[i];
    float x1 = xv[i+1];
    if (x >= x0 && x <= x1) {
      float t = (x - x0) / (x1 - x0);
      return yv[i] + t*(yv[i+1] - yv[i]);
    }
  }
  return yv[n-1];
}

// ============================================================================
// Calibrated ADC
// ============================================================================

class CalibratedADC : public RepeatSensor<float> {
 public:
  int pin_;
  esp_adc_cal_characteristics_t cal_;

  CalibratedADC(int pin, uint32_t interval_ms)
      : pin_(pin),
        RepeatSensor<float>(
            interval_ms,
            [this]() -> float {
              uint32_t raw = analogRead(pin_);
              uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, &cal_);
              return mv / 1000.0f;
            }) {

    analogReadResolution(12);
    analogSetPinAttenuation(pin_, ADC_11db);

    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_11,
        ADC_WIDTH_BIT_12,
        1100,
        &cal_);
  }
};

// ============================================================================
// Voltage → Resistance (Thermistor divider)
// Change value of R_fixed based on R2 of the voltage divider
// Values of R1 and R2 must be suffiiently high to not overload
// the ADC input (e.g., 200kΩ and 100kΩ), recommend testing voltage at sender
// under various temperatures / battery voltages to confirm it does not exceed 3.3V
// after the divider
// ============================================================================

class VoltageToResistance : public Transform<float, float> {
 public:
  float R_fixed = 100000.0f;   // edit based on voltage divider R2 value

  VoltageToResistance() : Transform<float, float>("volt2res") {}

  void set(const float& v_adc) override {
    if (v_adc < 0.02f || v_adc > 3.25f) {
      emit(NAN);
      return;
    }
    const float V_supply = 3.3f;
    float R_sender = (R_fixed * (V_supply / v_adc)) - R_fixed;
    emit(R_sender);
  }
};

// ============================================================================
// EXACT ORIGINAL COOLANT CURVE (Ω → Kelvin)
// coolant resistance to temperature (in kelvins) lookup table
// calibrated for American coolant temp sender (most Yanmar engines
// use a European sender with different characteristics)
// X being the resistance in ohms, Y being the temperature in kelvins
// ============================================================================

// ============================================================================
// EXACT ORIGINAL COOLANT CURVE (Ω → Kelvin)
// coolant resistance to temperature (in kelvins) lookup table
// calibrated for American coolant temp sender
// X: resistance in ohms
// Y: temperature in kelvin
// ============================================================================

static const size_t RESISTANCE_POINTS = 15;

static const float COOLANT_X[RESISTANCE_POINTS] = {
    20.0f,   // -> 410.00 K
    29.6f,   // -> 394.26 K
    40.0f,   // -> 373.15 K
    55.0f,   // -> 363.15 K
    70.0f,   // -> 353.15 K
    100.0f,  // -> 343.15 K
    112.0f,  // -> 345.37 K
    131.0f,  // -> 337.04 K
    140.0f,  // -> 333.15 K
    207.0f,  // -> 327.04 K
    300.0f,  // -> 317.15 K
    400.0f,  // -> 313.15 K
    450.0f,  // -> 310.93 K
    750.0f,  // -> 288.15 K
    900.0f   // -> 273.00 K
};

static const float COOLANT_Y[RESISTANCE_POINTS] = {
    410.00f, // at 20.0 Ω
    394.26f, // at 29.6 Ω
    373.15f, // at 40.0 Ω
    363.15f, // at 55.0 Ω
    353.15f, // at 70.0 Ω
    343.15f, // at 100.0 Ω
    345.37f, // at 112.0 Ω
    337.04f, // at 131.0 Ω
    333.15f, // at 140.0 Ω
    327.04f, // at 207.0 Ω
    317.15f, // at 300.0 Ω
    313.15f, // at 400.0 Ω
    310.93f, // at 450.0 Ω
    288.15f, // at 750.0 Ω
    273.00f  // at 900.0 Ω
};

class CoolantTempLookup : public Transform<float, float> {
 public:
  CoolantTempLookup() : Transform<float, float>("coolant_lookup") {}

  void set(const float& r) override {
    float k = linear_interp(r, COOLANT_X, COOLANT_Y, sizeof(COOLANT_X)/sizeof(float));
    emit(k);
  }
};

// ============================================================================
// EXACT ORIGINAL FUEL CURVE (RPM → m³/h)
// Fuel consumption lookup table for Yanmar 3YM30 engine with 17x13 propeller
// on a 22 000 lb sailboat at various RPMs. Your values WILL be different.
// Recommend reccording consumtion at 2000 RPM and cruising RPM (2700-2000)
// to calibrate for your engine/propeller/boat combination.
// ============================================================================

static const size_t FUEL_POINTS = 14;

static const float FUEL_X[FUEL_POINTS] = {
  500.0f, 1000.0f, 1500.0f, 2000.0f,
  2200.0f, 2400.0f, 2600.0f, 2800.0f,
  3000.0f, 3200.0f, 3400.0f, 3600.0f,
  3800.0f, 3900.0f
};

// Fuel flow in m³/h, 100% accurate at 2800 RPM (cruising RPM)
static const float FUEL_Y[FUEL_POINTS] = {
  0.000792f,  // 500
  0.000800f,  // 1000
  0.001116f,  // 1500
  0.001296f,  // 2000
  0.001692f,  // 2200
  0.002016f,  // 2400
  0.002304f,  // 2600
  0.002700f,  // 2800
  0.002988f,  // 3000
  0.003996f,  // 3200
  0.005004f,  // 3400
  0.006012f,  // 3600
  0.006984f,  // 3800
  0.007200f   // 3900
};


class FuelLookup : public Transform<float, float> {
 public:
  FuelLookup() : Transform<float, float>("fuel_lookup") {}

  void set(const float& rpm) override {
    float m3h = linear_interp(rpm, FUEL_X, FUEL_Y, sizeof(FUEL_X)/sizeof(float));
    emit(m3h);
  }
};

// ============================================================================
// RPM ISR + Reader
// ============================================================================

volatile uint32_t rpm_pulses = 0;
volatile uint32_t last_us    = 0;

void IRAM_ATTR rpm_isr() {
  uint32_t now = micros();

  // debounced to < 75µs pulse separation
  if (now - last_us > 75) {
    rpm_pulses++;
    last_us = now;
  }
}

class RPMCalc : public RepeatSensor<float> {
 public:
  uint32_t interval_;

  RPMCalc(uint32_t interval_ms)
      : interval_(interval_ms),
        RepeatSensor<float>(
            interval_ms,
            [this]() -> float {

              uint32_t p = rpm_pulses;
              rpm_pulses = 0;

              float revs = float(p) / float(RING_GEAR_TEETH);
              float rpm  = (revs / (interval_ / 1000.0f)) * 60.0f;

              g_last_rpm = rpm;
              return rpm;
            }) {}
};

// ============================================================================
// Round to 1 decimal
// ============================================================================

class Round1Decimal : public Transform<float, float> {
 public:
  Round1Decimal() : Transform<float, float>("round1") {}

  void set(const float& v) override {
    float r = roundf(v * 10.0f) / 10.0f;
    emit(r);
  }
};

// ============================================================================
// Coolant chain
// ============================================================================

void setup_coolant() {

  auto adc   = std::make_shared<CalibratedADC>(COOLANT_ADC_PIN, 1000);
  auto v2r   = std::make_shared<VoltageToResistance>();
  auto cvt   = std::make_shared<CoolantTempLookup>();

  auto sk = std::make_shared<SKOutputFloat>(
      "propulsion.engine.coolantTemperature",
      "/Engine/CoolantTemperature");

  adc->connect_to(v2r)
     ->connect_to(cvt)
     ->connect_to(sk);
}

// ============================================================================
// OneWire (preallocated bus)
// ============================================================================

sensesp::onewire::DallasTemperatureSensors* dts = nullptr;

void setup_onewire() {

  dts = new sensesp::onewire::DallasTemperatureSensors(ONE_WIRE_PIN);

  for (int i = 1; i <= 3; i++) {

    String idx  = String(i);
    String path = "/OneWire/S" + idx;

    auto t = new sensesp::onewire::OneWireTemperature(dts, 1000, path);

    auto sk = new SKOutputFloat(
        "environment.sensors.onewire." + idx,
        path + "/SK");

    t->connect_to(sk);
  }
}

// ============================================================================
// RPM + Fuel + Engine Hours
// ============================================================================

void setup_rpm_and_fuel() {

  pinMode(RPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_INPUT_PIN), rpm_isr, RISING);

  auto rpm_sens   = std::make_shared<RPMCalc>(100);
  auto rpm_smooth = std::make_shared<EMA>(0.3f);

  auto rpm_out = std::make_shared<SKOutputFloat>(
      "propulsion.engine.revolutions",
      "/Engine/RPM");

  rpm_sens->connect_to(rpm_smooth)->connect_to(rpm_out);

  auto fuel_map   = std::make_shared<FuelLookup>();
  auto fuel_smooth= std::make_shared<EMA>(0.2f);
  auto fuel_out   = std::make_shared<SKOutputFloat>(
      "propulsion.engine.fuel.rate",
      "/Engine/FuelRate");

  rpm_sens->connect_to(fuel_map)
          ->connect_to(fuel_smooth)
          ->connect_to(fuel_out);

  // Engine hour accumulation
  event_loop()->onRepeat(1000, []() {
    if (g_last_rpm > 500.0f) {
      float v = engine_hours->get();
      engine_hours->set(v + (1.0f / 3600.0f));
    }
  });

  // Save every 30 seconds
  event_loop()->onRepeat(30000, []() {
    engine_hours->save();
  });

  auto rounder = std::make_shared<Round1Decimal>();
  auto eh_out  = std::make_shared<SKOutputNumeric<float>>(
      "propulsion.engine.runTime",
      "/Engine/Hours");

  engine_hours->connect_to(rounder)->connect_to(eh_out);
}

// ============================================================================
// OTA page
// ============================================================================

void setup_ota_web() {

  ota_server.on("/", HTTP_GET, []() {
    ota_server.send(200, "text/html",
      "<h3>Engine Monitor OTA</h3>"
      "<form method='POST' enctype='multipart/form-data' action='/update'>"
      "<input type='file' name='firmware'>"
      "<input type='submit' value='Upload'>"
      "</form>");
  });

  ota_server.on(
      "/update",
      HTTP_POST,
      []() {
        bool ok = !Update.hasError();
        ota_server.send(200, "text/plain", ok ? "OK" : "FAIL");
        delay(200);
        if (ok) ESP.restart();
      },
      []() {
        HTTPUpload& u = ota_server.upload();
        if (u.status == UPLOAD_FILE_START) {
          Update.begin();
        } else if (u.status == UPLOAD_FILE_WRITE) {
          Update.write(u.buf, u.currentSize);
        } else if (u.status == UPLOAD_FILE_END) {
          Update.end(true);
        }
      });

  ota_server.begin();
}

// ============================================================================
// Build subsystems
// ============================================================================

void build_system() {
  setup_coolant();
  setup_onewire();
  setup_rpm_and_fuel();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {

  SensESPAppBuilder builder;
  builder.set_hostname("engine-monitor");

  sensesp::sensesp_app = builder.get_app();

  engine_hours = std::make_shared<PersistingObservableValue<float>>(
      0.0f, "/engine/hours");

  engine_hours->load();

  build_system();
  setup_ota_web();

  sensesp::sensesp_app->start();

  esp_ota_mark_app_valid_cancel_rollback();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  event_loop()->tick();
  ota_server.handleClient();
}
