//
// ============================================================================
// ANALOGUE DIESEL ENGINE MONITOR — ESP32 + SensESP 3.1.1
// AUTO-DETECT DS18B20 (0–3) WITH ROM-BASED UNIQUE SK PATHS
// ============================================================================

#include <Arduino.h>

// SensESP core
#include <sensesp.h>
#include <sensesp_app_builder.h>
#include <sensesp/system/filesystem.h>

// Sensors / transforms
#include <sensesp/sensors/sensor.h>
#include <sensesp/transforms/curveinterpolator.h>
#include <sensesp/transforms/ema.h>
#include <sensesp/signalk/signalk_output.h>

// OneWire Subsystem
#include <sensesp_onewire/onewire_temperature.h>

// Persistent storage
#include <sensesp/system/observablevalue.h>

// ADC calibration
#include <esp_adc_cal.h>

// OTA
#include <WebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>

using namespace sensesp;
using namespace sensesp::onewire;

// ============================================================================
// GLOBALS
// ============================================================================
extern std::shared_ptr<SensESPApp> sensesp_app;

WebServer ota_server(8080);

std::shared_ptr<PersistingObservableValue<float>> engine_hours;

static float g_last_rpm = 0.0f;

static const uint8_t COOLANT_ADC_PIN = 34;
static const uint8_t ONE_WIRE_PIN    = 4;
static const uint8_t RPM_INPUT_PIN   = 33;

static const uint16_t RING_GEAR_TEETH = 116;

// ============================================================================
// Helper: convert ROM to hex string
// ============================================================================
String rom_to_hex(const uint8_t* addr) {
  char buf[17];
  for (int i = 0; i < 8; i++) {
    sprintf(&buf[i * 2], "%02X", addr[i]);
  }
  return String(buf);
}

// ============================================================================
// ADC → voltage sensor
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
              uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, &this->cal_);
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
// Voltage → resistance
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

    float V_supply = 3.3f;
    float R_sender = (R_fixed * (V_supply / v_adc)) - R_fixed;
    emit(R_sender);
  }
};

// ============================================================================
// Coolant curve (Ω → Kelvin)
// ============================================================================
class TemperatureUSInterpreter : public CurveInterpolator {
 public:
  TemperatureUSInterpreter(String path = "")
      : CurveInterpolator(NULL, path) {
    clear_samples();
    add_sample({20.0f,   410.00f});
    add_sample({29.6f,   394.26f});
    add_sample({40.0f,   373.15f});
    add_sample({55.0f,   363.15f});
    add_sample({70.0f,   353.15f});
    add_sample({100.0f,  343.15f});
    add_sample({112.0f,  345.37f});
    add_sample({131.0f,  337.04f});
    add_sample({140.0f,  333.15f});
    add_sample({207.0f,  327.04f});
    add_sample({300.0f,  317.15f});
    add_sample({400.0f,  313.15f});
    add_sample({450.0f,  310.93f});
    add_sample({750.0f,  288.15f});
    add_sample({900.0f,  273.00f});
  }
};

// ============================================================================
// Fuel curve (RPM → m³/h)
// ============================================================================
class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String path = "")
      : CurveInterpolator(NULL, path) {

    clear_samples();
    add_sample({500,  0.0008f});
    add_sample({1000, 0.0008f});
    add_sample({1500, 0.0011f});
    add_sample({2000, 0.0013f});
    add_sample({2200, 0.0017f});
    add_sample({2400, 0.0020f});
    add_sample({2600, 0.0023f});
    add_sample({2800, 0.0027f});
    add_sample({3000, 0.0030f});
    add_sample({3200, 0.0040f});
    add_sample({3400, 0.0050f});
    add_sample({3600, 0.0060f});
    add_sample({3800, 0.0070f});
  }
};

// ============================================================================
// RPM ISR + Calculator
// ============================================================================
volatile uint32_t rpm_pulses = 0;
volatile uint32_t last_us    = 0;

void IRAM_ATTR rpm_isr() {
  uint32_t now = micros();
  if (now - last_us > 75) {
    rpm_pulses++;
    last_us = now;
  }
}

class RPMCalc : public RepeatSensor<float> {
 public:
  uint32_t interval_ms_;

  RPMCalc(uint32_t interval_ms)
      : interval_ms_(interval_ms),
        RepeatSensor<float>(
            interval_ms,
            [this]() -> float {
              uint32_t p = rpm_pulses;
              rpm_pulses = 0;

              float revs = float(p) / float(RING_GEAR_TEETH);
              float rpm  = (revs / (interval_ms_ / 1000.0f)) * 60.0f;

              g_last_rpm = rpm;
              return rpm;
            }) {}
};

// ============================================================================
// Rounder
// ============================================================================
class Round1Decimal : public Transform<float, float> {
 public:
  Round1Decimal() : Transform<float, float>("round1") {}
  void set(const float& v) override {
    emit(roundf(v * 10.0f) / 10.0f);
  }
};

// ============================================================================
// COOLANT SENSOR PIPELINE
// ============================================================================
void setup_coolant() {

  auto adc   = std::make_shared<CalibratedADC>(COOLANT_ADC_PIN, 1000);
  auto v2r   = std::make_shared<VoltageToResistance>();
  auto curve = std::make_shared<TemperatureUSInterpreter>();

  auto sk = std::make_shared<SKOutputFloat>(
      "propulsion.engine.coolantTemperature",
      "/Engine/CoolantTemperature");

  adc->connect_to(v2r)->connect_to(curve)->connect_to(sk);
}

// ============================================================================
// ONEWIRE: Three sensors, one per pin (S1=GPIO4, S2=GPIO16, S3=GPIO17)
// Fully compatible with SensESP 3.1.1 + OneWire 3.0.2
// ============================================================================

void setup_onewire() {

  const uint32_t read_delay = 1000;

  // -------------------------------------------------------
  // SENSOR 1 — GPIO 4
  // -------------------------------------------------------
  {
    auto* dts1 = new DallasTemperatureSensors(4);
    dts1->update_sensors();

    if (dts1->get_sensor_count() > 0) {

      String base1 = "/OneWire/S1";

      auto* t1 = new OneWireTemperature(
          dts1,
          read_delay,
          base1
      );

      auto* sk1 = new SKOutputFloat(
          "environment.sensors.onewire.1",
          base1 + "/SK"
      );

      t1->connect_to(sk1);
      Serial.println("OneWire S1 detected.");
    } else {
      Serial.println("OneWire S1 missing → sensor skipped.");
    }
  }

  // -------------------------------------------------------
  // SENSOR 2 — GPIO 16
  // -------------------------------------------------------
  {
    auto* dts2 = new DallasTemperatureSensors(16);
    dts2->update_sensors();

    if (dts2->get_sensor_count() > 0) {

      String base2 = "/OneWire/S2";

      auto* t2 = new OneWireTemperature(
          dts2,
          read_delay,
          base2
      );

      auto* sk2 = new SKOutputFloat(
          "environment.sensors.onewire.2",
          base2 + "/SK"
      );

      t2->connect_to(sk2);
      Serial.println("OneWire S2 detected.");
    } else {
      Serial.println("OneWire S2 missing → sensor skipped.");
    }
  }
 
  // -------------------------------------------------------
  // SENSOR 3 — GPIO 17
  // -------------------------------------------------------
  {
    auto* dts3 = new DallasTemperatureSensors(17);
    dts3->update_sensors();

    if (dts3->get_sensor_count() > 0) {

      String base3 = "/OneWire/S3";

      auto* t3 = new OneWireTemperature(
          dts3,
          read_delay,
          base3
      );

      auto* sk3 = new SKOutputFloat(
          "environment.sensors.onewire.3",
          base3 + "/SK"
      );

      t3->connect_to(sk3);
      Serial.println("OneWire S3 detected.");
    } else {
      Serial.println("OneWire S3 missing → sensor skipped.");
    }
  }
}



// ============================================================================
// RPM + Fuel + Hours
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

  auto fuel_curve = std::make_shared<FuelInterpreter>();
  auto fuel_smooth = std::make_shared<EMA>(0.2f);

  auto fuel_out = std::make_shared<SKOutputFloat>(
      "propulsion.engine.fuel.rate",
      "/Engine/FuelRate");

  rpm_sensor
      ->connect_to(fuel_curve)
      ->connect_to(fuel_smooth)
      ->connect_to(fuel_out);

  // Engine hours increment
  event_loop()->onRepeat(1000, []() {
    if (engine_hours && g_last_rpm > 500.0f) {
      engine_hours->set(engine_hours->get() + 1.0f / 3600.0f);
    }
  });

  // Persist engine hours every 30s
  event_loop()->onRepeat(30000, []() {
    if (engine_hours) engine_hours->save();
  });

  auto rounder = std::make_shared<Round1Decimal>();

  auto eh_out = std::make_shared<SKOutputNumeric<float>>(
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
// OTA SERVER
// ============================================================================
void setup_ota_web() {

  ota_server.on("/", HTTP_GET, []() {
    ota_server.send(
      200, "text/html",
      "<h2>Engine Monitor OTA Update</h2>"
      "<form method='POST' action='/update' enctype='multipart/form-data'>"
      "<input type='file' name='firmware'/>"
      "<input type='submit' value='Upload'/>"
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

#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

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
