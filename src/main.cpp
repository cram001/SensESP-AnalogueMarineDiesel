//
// ============================================================================
// ANALOGUE DIESEL ENGINE MONITOR — ESP32 + SensESP 3.x
// ============================================================================
// Features:
//   • Coolant temp (ADC → V → Ω → K → Signal K)
//   • 3 × DS18B20 OneWire sensors
//   • RPM via magnetic pickup ISR + RepeatSensor<float>
//   • Fuel burn estimation + EMA smoothing
//   • Engine hours accumulator (stored, persists)
//   • OTA upload page (/) + rollback protection
// ============================================================================

#include <Arduino.h>

// SensESP Core
#include "sensesp.h"
#include "sensesp_app_builder.h"
#include "sensesp/system/filesystem.h"

// Sensors / transforms
#include "sensesp/sensors/sensor.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/ema.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/signalk/signalk_output.h"

// OneWire subsystem
#include "sensesp_onewire/onewire_temperature.h"

// Persistent values
#include "sensesp/system/observablevalue.h"

// ESP32 calibration
#include "esp_adc_cal.h"

// OTA
#include <WebServer.h>
#include <Update.h>
#include "esp_ota_ops.h"

using namespace sensesp;

// ---------------------------------------------------------------------------
// GLOBALS
// ---------------------------------------------------------------------------

// Use SensESP's official global
// extern std::shared_ptr<SensESPApp> sensesp::sensesp_app;

WebServer ota_server(80);

// Engine hours persistent (stored)
std::shared_ptr<PersistingObservableValue<float>> engine_hours;

static float g_last_rpm = 0.0f;

// ---------------------------------------------------------------------------
// HARDWARE CONSTANTS
// ---------------------------------------------------------------------------
static const uint8_t COOLANT_ADC_PIN = 34;
static const uint8_t ONE_WIRE_PIN    = 4;
static const uint8_t RPM_INPUT_PIN   = 33;

static const uint16_t RING_GEAR_TEETH = 116;

// ============================================================================
// ADC SENSOR (Calibrated)
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
              uint32_t raw = analogRead(this->pin_);
              uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &this->cal_);
              return mv / 1000.0f;
            }) 
  {
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
// VOLTAGE → RESISTANCE (Thermistor divider)
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
// THERMISTOR CURVE (Ω → Kelvin)
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
// FUEL INTERPOLATOR (RPM → m³/h)
// ============================================================================
class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String path = "")
      : CurveInterpolator(NULL, path) 
  {
    clear_samples();

    add_sample({500,  0.00080f / 1000.0f});
    add_sample({1000, 0.00140f / 1000.0f});
    add_sample({1500, 0.00220f / 1000.0f});
    add_sample({1800, 0.00300f / 1000.0f});
    add_sample({2000, 0.00380f / 1000.0f});
    add_sample({2200, 0.00450f / 1000.0f});
    add_sample({2400, 0.00530f / 1000.0f});
    add_sample({2600, 0.00620f / 1000.0f});
    add_sample({2800, 0.00720f / 1000.0f});
    add_sample({3000, 0.00830f / 1000.0f});
  }
};

// ============================================================================
// RPM ISR + SENSOR
// ============================================================================
volatile uint32_t rpm_pulses = 0;
volatile uint32_t last_us    = 0;

void IRAM_ATTR rpm_isr() {
  uint32_t now = micros();
  if (now - last_us > 75) {     // debounce: ignore pulses < 75us apart (works up to 4000 RPM), min 110 for 3800 RPM
    rpm_pulses++;
    last_us = now;
  }
}

class RPMCalc : public RepeatSensor<float> {
 public:
  const uint32_t interval_ms_;

  RPMCalc(uint32_t interval_ms)
      : interval_ms_(interval_ms),
        RepeatSensor<float>(
            interval_ms,
            [this]() -> float {
              uint32_t p = rpm_pulses;
              rpm_pulses = 0;

              float revs = float(p) / float(RING_GEAR_TEETH);
              float rpm = (revs / (interval_ms_ / 1000.0f)) * 60.0f;

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
// COOLANT CHAIN
// ============================================================================
void setup_coolant() {

  auto adc   = std::make_shared<CalibratedADC>(COOLANT_ADC_PIN, 1000);
  auto v2r   = std::make_shared<VoltageToResistance>();
  auto curve = std::make_shared<TemperatureUSInterpreter>();

  auto sk = std::make_shared<SKOutputFloat>(
      "propulsion.engine.coolantTemperature",
      "/Engine/CoolantTemperature");

  adc->connect_to(v2r)
     ->connect_to(curve)
     ->connect_to(sk);
}

// ============================================================================
// ONEWIRE
// ============================================================================
void setup_onewire() {

  auto* dts = new onewire::DallasTemperatureSensors(ONE_WIRE_PIN);

  for (int i = 1; i <= 3; i++) {
    String idx  = String(i);
    String base = "/OneWire/S" + idx;

    auto t  = new onewire::OneWireTemperature(dts, 1000, base);
    auto sk = new SKOutputFloat(
        "environment.sensors.onewire." + idx,
        base + "/SK");

    t->connect_to(sk);
  }
}

// ============================================================================
// RPM + Fuel + Engine Hours
// ============================================================================
void setup_rpm_and_fuel() {

  pinMode(RPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_INPUT_PIN), rpm_isr, RISING);

  auto rpm_sensor = std::make_shared<RPMCalc>(100);
  auto rpm_smooth = std::make_shared<EMA>(0.3f);

  auto rpm_out = std::make_shared<SKOutputFloat>(
      "propulsion.engine.revolutions",
      "/Engine/RPM");

  rpm_sensor->connect_to(rpm_smooth)->connect_to(rpm_out);

  auto fuel_curve  = std::make_shared<FuelInterpreter>();
  auto fuel_smooth = std::make_shared<EMA>(0.2f);
  auto fuel_out    = std::make_shared<SKOutputFloat>(
      "propulsion.engine.fuel.rate",
      "/Engine/FuelRate");

  rpm_sensor->connect_to(fuel_curve)
            ->connect_to(fuel_smooth)
            ->connect_to(fuel_out);

  // Engine hour accumulation
  event_loop()->onRepeat(1000, []() {
    if (g_last_rpm > 500.0f) {
      float val = engine_hours->get();
      engine_hours->set(val + 1.0f / 3600.0f);
    }
  });

  // Save to flash every 30 seconds
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
// BUILD SYSTEM
// ============================================================================
void build_system() {
  setup_coolant();
  setup_onewire();
  setup_rpm_and_fuel();
}

// ============================================================================
// OTA
// ============================================================================
void setup_ota_web() {

  ota_server.on("/", HTTP_GET, []() {
    ota_server.send(200, "text/html",
      "<h2>Engine Monitor OTA</h2>"
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
        delay(300);
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
// SETUP
// ============================================================================
void setup() {

  SensESPAppBuilder builder;
  builder.set_hostname("engine-monitor");

  // IMPORTANT: use SensESP's global instance
  sensesp::sensesp_app = builder.get_app();

  // Persistent engine hours (2 decimal internal resolution)
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
