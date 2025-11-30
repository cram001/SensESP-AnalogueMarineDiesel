//
// ============================================================================
// DIESEL ENGINE MONITOR — ESP32 + SensESP 3.1.1
// Safe FS Init + No SK Client + Kelvin Everywhere
// ============================================================================

#include <Arduino.h>

// SensESP Core
#include <sensesp.h>
#include <sensesp_app_builder.h>
#include <sensesp/system/filesystem.h>

// Sensors / transforms
#include <sensesp/sensors/sensor.h>
#include <sensesp/transforms/ema.h>
#include <sensesp/transforms/curveinterpolator.h>
#include <sensesp/signalk/signalk_output.h>

// OneWire
#include <OneWire.h>
#include <DallasTemperature.h>

// Persistent
#include <sensesp/system/observablevalue.h>

// ADC Calibration
#include <esp_adc_cal.h>

// OTA
#include <WebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>

using namespace sensesp;

// ============================================================================
// GLOBALS
// ============================================================================

WebServer ota_server(8080);

static const uint8_t COOLANT_ADC_PIN = 34;
static const uint8_t ONE_1 = 4;
static const uint8_t ONE_2 = 16;
static const uint8_t ONE_3 = 17;

static const uint8_t RPM_PIN = 33;
static const uint16_t RING_TEETH = 116;

float g_last_rpm = 0.0f;

std::shared_ptr<PersistingObservableValue<float>> engine_hours;

// ============================================================================
// SAFE FILESYSTEM INIT (AUTO-FORMAT)
// ============================================================================

void safe_init_filesystem() {

  bool mounted = Filesystem::instance()->mount();

  if (!mounted) {
    Serial.println("❗ LittleFS mount failed — formatting...");
    Filesystem::instance()->format();
    delay(500);

    // Try again
    if (!Filesystem::instance()->mount()) {
      Serial.println("❗ FATAL: LittleFS mount failed after format.");
    }
  }

  // Ensure /config exists
  if (!Filesystem::instance()->exists("/config")) {
    Filesystem::instance()->mkdir("/config");
  }
}

// ============================================================================
// ROM → Hex
// ============================================================================
String rom_to_hex(const uint8_t* addr) {
  char buf[17];
  for (int i = 0; i < 8; i++) sprintf(&buf[i * 2], "%02X", addr[i]);
  return String(buf);
}

// ============================================================================
// Calibrated ADC
// ============================================================================
class CalibratedADC : public RepeatSensor<float> {
 public:
  int pin_;
  esp_adc_cal_characteristics_t cal_;

  CalibratedADC(int pin, uint32_t interval)
      : pin_(pin),
        RepeatSensor<float>(interval, [this]() -> float {
          uint32_t raw = analogRead(pin_);
          uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &cal_);
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
// Voltage → Resistance
// ============================================================================
class VoltageToResistance : public Transform<float, float> {
 public:
  float Rf = 100000.0f;

  VoltageToResistance() : Transform<float, float>("V2R") {}

  void set(const float& Vadc) override {
    if (Vadc < 0.02f || Vadc > 3.25f) {
      emit(NAN);
      return;
    }
    float R = (Rf * (3.3f / Vadc)) - Rf;
    emit(R);
  }
};

// ============================================================================
// Coolant Curve (Faria/Teleflex US)
// ============================================================================
class TempInterpreter : public CurveInterpolator {
 public:
  TempInterpreter() : CurveInterpolator(nullptr, "") {
    clear_samples();
    add_sample({20.0f, 410.0f});
    add_sample({40.0f, 373.15f});
    add_sample({70.0f, 353.15f});
    add_sample({100.0f, 343.15f});
    add_sample({140.0f, 333.15f});
    add_sample({300.0f, 317.15f});
    add_sample({400.0f, 313.15f});
    add_sample({750.0f, 288.15f});
  }
};

// ============================================================================
// DS18B20 Sensor Wrapper
// ============================================================================
class DS18Sensor : public RepeatSensor<float> {
 public:
  DallasTemperature* dt_;
  DeviceAddress addr_;

  DS18Sensor(DallasTemperature* dt, const uint8_t* rom)
      : RepeatSensor<float>(1000, [this]() -> float {
          dt_->requestTemperatures();
          float t = dt_->getTempC(addr_);
          return (t > -100 && t < 200) ? (t + 273.15f) : NAN;
        }),
        dt_(dt) {
    memcpy(addr_, rom, 8);
  }
};

// ============================================================================
// RPM ISR
// ============================================================================
volatile uint32_t pulses = 0;
volatile uint32_t last_us = 0;

void IRAM_ATTR rpm_isr() {
  uint32_t now = micros();
  if (now - last_us > 75) {
    pulses++;
    last_us = now;
  }
}

class RPMCalc : public RepeatSensor<float> {
 public:
  uint32_t int_ms_;

  RPMCalc(uint32_t ms)
      : int_ms_(ms),
        RepeatSensor<float>(ms, [this]() -> float {
          uint32_t p = pulses;
          pulses = 0;
          float revs = float(p) / float(RING_TEETH);
          float rpm = (revs / (int_ms_ / 1000.0f)) * 60.0f;
          g_last_rpm = rpm;
          return rpm;
        }) {}
};

// ============================================================================
// Rounder
// ============================================================================
class Round1 : public Transform<float, float> {
 public:
  Round1() : Transform<float, float>("R1") {}
  void set(const float& v) override { emit(roundf(v * 10.0f) / 10.0f); }
};

// ============================================================================
// SETUP COOLANT
// ============================================================================
void setup_coolant() {

  auto adc = std::make_shared<CalibratedADC>(COOLANT_ADC_PIN, 1000);
  auto v2r = std::make_shared<VoltageToResistance>();
  auto cur = std::make_shared<TempInterpreter>();

  auto sk = std::make_shared<SKOutputNumeric<float>>(
      "propulsion.engine.coolantTemperature",
      "/Engine/CoolantTemp/SK");

  sk->set_metadata(new SKMetadata("K"));

  adc->connect_to(v2r)->connect_to(cur)->connect_to(sk);
}

// ============================================================================
// SETUP ONEWIRE BUS
// ============================================================================
void configure_bus(uint8_t pin) {

  auto* dt = new DallasTemperature(new OneWire(pin));
  dt->begin();

  int count = dt->getDeviceCount();
  Serial.printf("Bus GPIO %u: %d devices\n", pin, count);

  DevicAddress addr;

  for (int i = 0; i < count; i++) {

    if (!dt->getAddress(addr, i)) continue;

    String rom = rom_to_hex(addr);

    auto* sensor = new DS18Sensor(dt, addr);

    String sk_path = "environment.sensors." + rom + ".temperature";
    String cfg = "/OneWire/" + rom;

    auto* sk = new SKOutputNumeric<float>(sk_path, cfg + "/SK");
    sk->set_metadata(new SKMetadata("K"));

    sensor->connect_to(sk);
  }
}

void setup_onewire() {
  configure_bus(ONE_1);
  configure_bus(ONE_2);
  configure_bus(ONE_3);
}

// ============================================================================
// RPM + FUEL + HOURS
// ============================================================================
void setup_rpm_fuel_hours() {

  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpm_isr, RISING);

  auto rpm = std::make_shared<RPMCalc>(100);
  auto smooth = std::make_shared<EMA>(0.3f);

  auto sk_rpm = std::make_shared<SKOutputNumeric<float>>(
      "propulsion.engine.revolutions",
      "/Engine/RPM");
  sk_rpm->set_metadata(new SKMetadata("Hz"));

  rpm->connect_to(smooth)->connect_to(sk_rpm);

  // engine hours
  event_loop()->onRepeat(1000, []() {
    if (engine_hours && g_last_rpm > 500.0f)
      engine_hours->set(engine_hours->get() + 1.0f / 3600.0f);
  });

  event_loop()->onRepeat(30000, []() {
    if (engine_hours) engine_hours->save();
  });

  auto rounder = std::make_shared<Round1>();
  auto sk_hr = std::make_shared<SKOutputNumeric<float>>(
      "propulsion.engine.runTime",
      "/Engine/Hours");
  sk_hr->set_metadata(new SKMetadata("h"));

  engine_hours->connect_to(rounder)->connect_to(sk_hr);
}

// ============================================================================
// OTA
// ============================================================================
void setup_ota() {

  ota_server.on("/", HTTP_GET, []() {
    ota_server.send(200, "text/html",
                    "<h2>OTA Update</h2>"
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
        delay(200);
        if (ok) ESP.restart();
      },
      []() {
        HTTPUpload& u = ota_server.upload();
        if (u.status == UPLOAD_FILE_START) Update.begin();
        else if (u.status == UPLOAD_FILE_WRITE) Update.write(u.buf, u.currentSize);
        else if (u.status == UPLOAD_FILE_END) Update.end(true);
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

  safe_init_filesystem();

  SensESPAppBuilder builder;

  builder.set_hostname("engine-monitor");
  builder.set_sk_server("0.0.0.0", 3000);   // dummy but valid
  builder.set_wifi("engine-monitor", "12345678");

  sensesp_app = builder.get_app();

  engine_hours = std::make_shared<PersistingObservableValue<float>>(0.0f, "/engine/hours");
  engine_hours->load();

  setup_coolant();
  setup_onewire();
  setup_rpm_fuel_hours();
  setup_ota();

  // disable WS client completely
  sensesp_app->get_ws_client()->set_enabled(false);

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
