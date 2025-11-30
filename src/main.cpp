//
// ============================================================================
// DIESEL ENGINE MONITOR — ESP32 + SensESP 3.1.1
// Multi-bus DS18B20 auto-detect with ROM-based SK paths
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

// OneWire system
#include <OneWire.h>
#include <DallasTemperature.h>

// Persistent
#include <sensesp/system/observablevalue.h>

// ADC calibration
#include <esp_adc_cal.h>

// OTA web updater
#include <WebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>

using namespace sensesp;

// ============================================================================
// GLOBALS
// ============================================================================
WebServer ota_server(8080);

static const uint8_t COOLANT_ADC_PIN = 34;

static const uint8_t ONEWIRE_PIN_1 = 4;
static const uint8_t ONEWIRE_PIN_2 = 16;
static const uint8_t ONEWIRE_PIN_3 = 17;

static const uint8_t RPM_INPUT_PIN = 33;
static const uint16_t RING_GEAR_TEETH = 116;

float g_last_rpm = 0.0f;

std::shared_ptr<PersistingObservableValue<float>> engine_hours;

// ============================================================================
// Helper: ROM address to hex string
// ============================================================================
String rom_to_hex(const uint8_t* addr) {
  char buf[17];
  for (int i = 0; i < 8; i++) {
    sprintf(&buf[i * 2], "%02X", addr[i]);
  }
  return String(buf);
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
        RepeatSensor<float>(interval_ms, [this]() -> float {
          uint32_t raw = analogRead(this->pin_);
          uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &this->cal_);
          return mv / 1000.0f;  // volts
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
// Voltage → Resistance transform
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
// Coolant temperature curve
// ============================================================================
class TemperatureUSInterpreter : public CurveInterpolator {
 public:
  TemperatureUSInterpreter(String path = "")
      : CurveInterpolator(nullptr, path) {

    clear_samples();

    add_sample({20.0f, 410.00f});
    add_sample({29.6f, 394.26f});
    add_sample({40.0f, 373.15f});
    add_sample({55.0f, 363.15f});
    add_sample({70.0f, 353.15f});
    add_sample({100.0f, 343.15f});
    add_sample({112.0f, 345.37f});
    add_sample({131.0f, 337.04f});
    add_sample({140.0f, 333.15f});
    add_sample({207.0f, 327.04f});
    add_sample({300.0f, 317.15f});
    add_sample({400.0f, 313.15f});
    add_sample({450.0f, 310.93f});
    add_sample({750.0f, 288.15f});
    add_sample({900.0f, 273.00f});
  }
};

// ============================================================================
// Fuel curve (RPM → m³/h)
// ============================================================================
class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String path = "")
      : CurveInterpolator(nullptr, path) {

    clear_samples();
    add_sample({500, 0.0008f});
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
// RPM ISR
// ============================================================================
volatile uint32_t rpm_pulses = 0;
volatile uint32_t last_us = 0;

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
        RepeatSensor<float>(interval_ms, [this]() -> float {

          uint32_t p = rpm_pulses;
          rpm_pulses = 0;

          float revs = float(p) / float(RING_GEAR_TEETH);
          float rpm = (revs / (interval_ms_ / 1000.0f)) * 60.0f;

          g_last_rpm = rpm;
          return rpm;
        }) {}
};

// ============================================================================
// Rounder (1 decimal)
// ============================================================================
class Round1Decimal : public Transform<float, float> {
 public:
  Round1Decimal() : Transform<float, float>("round1") {}
  void set(const float& v) override { emit(roundf(v * 10.0f) / 10.0f); }
};

// ============================================================================
// COOLANT SENSOR SETUP
// ============================================================================
void setup_coolant() {

  auto adc = std::make_shared<CalibratedADC>(COOLANT_ADC_PIN, 1000);
  auto v2r = std::make_shared<VoltageToResistance>();
  auto curve = std::make_shared<TemperatureUSInterpreter>();

  auto sk = std::make_shared<SKOutputNumeric<float>>(
      "propulsion.engine.coolantTemperature",
      "/Engine/CoolantTemperature");

  sk->set_metadata(new SKMetadata("K"));

  adc->connect_to(v2r)->connect_to(curve)->connect_to(sk);
}

// ============================================================================
// DS18B20 SENSOR (RepeatSensor<float> wrapper)
// ============================================================================
class DS18Sensor : public RepeatSensor<float> {
 public:
  DallasTemperature* dt_;
  DeviceAddress addr_;

  DS18Sensor(DallasTemperature* dt, const uint8_t* rom, uint32_t read_delay)
      : RepeatSensor<float>(
            read_delay,
            [this]() -> float {
              dt_->requestTemperatures();
              float tC = dt_->getTempC(addr_);
              if (tC > -100 && tC < 200) return tC;
              return NAN;
            }),
        dt_(dt) {
    memcpy(addr_, rom, 8);
  }
};

// ============================================================================
// Configure a single OneWire bus
// ============================================================================
void configure_bus(uint8_t pin) {

  auto* dt = new DallasTemperature(new OneWire(pin));
  dt->begin();

  int count = dt->getDeviceCount();

  if (count == 0) {
    Serial.printf("No DS18B20 sensors found on GPIO %u\n", pin);
    return;
  }

  Serial.printf("%d sensor(s) on GPIO %u\n", count, pin);

  DeviceAddress addr;

  for (int i = 0; i < count; i++) {

    if (!dt->getAddress(addr, i)) continue;

    String rom = rom_to_hex(addr);
    Serial.printf("ROM[%d]: %s\n", i, rom.c_str());

    auto* temp = new DS18Sensor(dt, addr, 1000);

    String sk_path = "environment.sensors." + rom + ".temperature";
    String cfg_path = "/OneWire/" + rom;

    auto* sk = new SKOutputNumeric<float>(sk_path, cfg_path + "/SK");
    sk->set_metadata(new SKMetadata("K"));   // minimal metadata

    temp->connect_to(sk);
  }
}

void setup_onewire() {
  configure_bus(ONEWIRE_PIN_1);
  configure_bus(ONEWIRE_PIN_2);
  configure_bus(ONEWIRE_PIN_3);
}

// ============================================================================
// RPM + Fuel + Hours
// ============================================================================
void setup_rpm_and_fuel() {

  pinMode(RPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_INPUT_PIN), rpm_isr, RISING);

  auto rpm_sensor = std::make_shared<RPMCalc>(100);
  auto rpm_smooth = std::make_shared<EMA>(0.3f);

  auto rpm_out = std::make_shared<SKOutputNumeric<float>>(
      "propulsion.engine.revolutions",
      "/Engine/RPM");

  rpm_out->set_metadata(new SKMetadata("Hz"));

  rpm_sensor->connect_to(rpm_smooth)->connect_to(rpm_out);

  auto fuel_curve = std::make_shared<FuelInterpreter>();
  auto fuel_smooth = std::make_shared<EMA>(0.2f);

  auto fuel_out = std::make_shared<SKOutputNumeric<float>>(
      "propulsion.engine.fuel.rate",
      "/Engine/FuelRate");

  fuel_out->set_metadata(new SKMetadata("m3/h"));

  rpm_sensor
      ->connect_to(fuel_curve)
      ->connect_to(fuel_smooth)
      ->connect_to(fuel_out);

  // Engine hours logic
  event_loop()->onRepeat(1000, []() {
    if (engine_hours && g_last_rpm > 500.0f) {
      engine_hours->set(engine_hours->get() + 1.0f / 3600.0f);
    }
  });

  event_loop()->onRepeat(30000, []() {
    if (engine_hours) engine_hours->save();
  });

  auto rounder = std::make_shared<Round1Decimal>();
  auto eh_out = std::make_shared<SKOutputNumeric<float>>(
      "propulsion.engine.runTime",
      "/Engine/Hours");

  eh_out->set_metadata(new SKMetadata("h"));

  engine_hours->connect_to(rounder)->connect_to(eh_out);
}

// ============================================================================
// OTA WEB SERVER
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
        delay(250);
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
// SYSTEM BUILD
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

#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  SensESPAppBuilder builder;
  builder.set_hostname("engine-monitor");

  sensesp_app = builder.get_app();

  engine_hours =
      std::make_shared<PersistingObservableValue<float>>(0.0f, "/engine/hours");
  engine_hours->load();

  build_system();
  setup_ota_web();

  sensesp_app->start();

  esp_ota_mark_app_valid_cancel_rollback();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  event_loop()->tick();
  ota_server.handleClient();
}
