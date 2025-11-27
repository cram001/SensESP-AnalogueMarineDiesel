//
// ============================================================================
// ANALOGUE DIESEL ENGINE MONITOR — ESP32 + SensESP 3.x
// Optimized: no CurveInterpolator, static linear interpolation,
// reduced lambdas, fewer heap objects, faster execution.
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
// Hardware constants
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
// ============================================================================

class VoltageToResistance : public Transform<float, float> {
 public:
  float R_fixed = 100000.0f;

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
// ============================================================================

static const float COOLANT_X[] = {
  20, 29.6, 40, 55, 70, 100,
  112, 131, 140, 207, 300,
  400, 450, 750, 900
};

static const float COOLANT_Y[] = {
  410.00, 394.26, 373.15, 363.15, 353.15, 343.15,
  345.37, 337.04, 333.15, 327.04, 317.15,
  313.15, 310.93, 288.15, 273.00
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
// ============================================================================

static const float FUEL_X[] = {
  500, 1000, 1500, 1800, 2000,
  2200, 2400, 2600, 2800, 3000
};

static const float FUEL_Y[] = {
  0.00080f/1000.0f,
  0.00140f/1000.0f,
  0.00220f/1000.0f,
  0.00300f/1000.0f,
  0.00380f/1000.0f,
  0.00450f/1000.0f,
  0.00530f/1000.0f,
  0.00620f/1000.0f,
  0.00720f/1000.0f,
  0.00830f/1000.0f
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
