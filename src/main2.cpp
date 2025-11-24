// SenESP Engine Sensors
//Code - Nov 2025
//Change - Moved from Arduino to PlatformIO, moved from BMP280 to OneWire/Dallas
// One wire sensor ID store in JSON file
// Added Fuel Flow calculation based on RPM for Yanmar 3JH3E

#include <Arduino.h>

// SensESP Core

#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

// Sensors
#include "sensesp_onewire/dallas_temperature_sensors.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"

// OneWire + DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// Transforms
#include "sensesp/transforms/ema.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/voltagedivider.h"

// SignalK Output
#include "sensesp/signalk/signalk_output.h"

// Config & Filesystem
#include "sensesp/system/filesystem.h"
#include "sensesp/system/valueconsumer.h"
#include "sensesp/system/observable.h"

using namespace sensesp;

// ============================================================================
// CONSTANTS AND PIN ASSIGNMENTS
// ============================================================================

static const uint8_t COOLANT_ADC_PIN = 34;     // buffered ADC input via voltage divider
static const uint8_t ONE_WIRE_PIN = 4;         // DS18B20 bus
static const uint8_t RPM_INPUT_PIN = 33;       // op-isolated RPM pulses via op-amp buffer

static const uint16_t RING_GEAR_TEETH = 116;   // Yanmar 3JH3E 116/ 3GM30F 97
static const uint8_t MAX_DS18 = 3;  //max number of DS18B20 sensors to read

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

//analog_input->connect_to(new AnalogVoltage(Vin, Vin))
//      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
//      ->connect_to(new SKOutputFloat("propulsion.engine.temperature.raw"));

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
// COOLANT TEMPERATURE LOOKUP TABLES FOR US SENDER (450-30 Ohm)
// ============================================================================

class TemperatureUSInterpreter : public CurveInterpolator {
 public:
  TemperatureUSInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our temperature sender to degrees Kelvin
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
// FUEL FLOW INTERPOLATOR Yanmar 3JH3E with 18x11 3 blade propeller on 21 000 lbs sailboat
// Consumption correct @ 2800 rpm (2.7L/hr)
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


// reactesp::ReactESP app;

//  Adafruit_BMP280 bmp280;

//  float read_temp_callback() { return (bmp280.readTemperature() + 273.15);}
//  float read_pressure_callback() { return (bmp280.readPressure());}

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("SensESP")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->enable_uptime_sensor()
                    // ->enable_ota("raspberry")
                    ->get_app();

/// 1-Wire Temp Sensors - Exhaust Temp & Oil Temp Sensors ///

DallasTemperatureSensors* dts = new DallasTemperatureSensors(17);

//exhaust config
  auto* exhaust_temp =
      new OneWireTemperature(dts, 1000, "/Exhaust Temperature/oneWire");

//oil config (remove if not required, can also be copied for more sensors)
  auto* oil_temp =
      new OneWireTemperature(dts, 1000, "/Oil Temperature/oneWire");

//exhaust
  exhaust_temp->connect_to(new Linear(1.0, 0.0, "/Exhaust Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("propulsion.engine.1.exhaustTemperature","/Exhaust Temperature/sk_path"));
 
 //oil (remove if not required, can also be copied for more sensors)
 oil_temp->connect_to(new Linear(1.0, 0.0, "/Oil Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("propulsion.engine.1.oilTemperature","/Oil Temperature/sk_path"));

 //RPM Application/////

  const char* config_path_calibrate = "/Engine RPM/calibrate";
  const char* config_path_skpath = "/Engine RPM/sk_path";
  const float multiplier = 1.0;

  auto* sensor = new DigitalInputCounter(16, INPUT_PULLUP, RISING, 500);

  sensor->connect_to(new Frequency(multiplier, config_path_calibrate))  
  // connect the output of sensor to the input of Frequency()
         ->connect_to(new MovingAverage(2, 1.0,"/Engine RPM/movingAVG"))
         ->connect_to(new SKOutputFloat("propulsion.engine.revolutions", config_path_skpath));  
          // connect the output of Frequency() to a Signal K Output as a number

  sensor->connect_to(new Frequency(6))
  // times by 6 to go from Hz to RPM
          ->connect_to(new MovingAverage(4, 1.0,"/Engine Fuel/movingAVG"))
          ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
          ->connect_to(new SKOutputFloat("propulsion.engine.fuel.rate", "/Engine Fuel/sk_path"));                                       

/// BMP280 SENSOR CODE - Engine Room Temp Sensor ////  

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  bmp280.begin(0x76);

  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* engine_room_temp =
      new RepeatSensor<float>(5000, read_temp_callback);

  auto* engine_room_pressure = 
      new RepeatSensor<float>(60000, read_pressure_callback);

  // Send the temperature to the Signal K server as a Float
  engine_room_temp->connect_to(new SKOutputFloat("propulsion.engineRoom.temperature"));

  engine_room_pressure->connect_to(new SKOutputFloat("propulsion.engineRoom.pressure"));

  
//// Engine Temp Config ////

const float Vin = 3.5;
const float R1 = 120.0;
auto* analog_input = new AnalogInput(36, 2000);

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new TemperatureInterpreter("/Engine Temp/curve"))
      ->connect_to(new Linear(1.0, 0.9, "/Engine Temp/calibrate"))
      ->connect_to(new MovingAverage(4, 1.0,"/Engine Temp/movingAVG"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature", "/Engine Temp/sk_path"));

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature.raw"));

}



void loop() { app.tick(); }