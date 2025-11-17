#pragma once

#include <cmath>
#include "sensesp.h"

namespace sensesp {

class EMA : public Transform<float, float> {
 public:
  float alpha_ = 0.5f;
  bool initialized_ = false;
  float state_ = NAN;

  EMA(float alpha = 0.5f) : Transform<float, float>("ema") { alpha_ = alpha; }

  bool to_json(JsonObject& root) override {
    root["alpha"] = alpha_;
    return true;
  }

  bool from_json(const JsonObject& config) override {
    if (config["alpha"].is<float>()) alpha_ = config["alpha"].as<float>();
    return true;
  }

  void set(const float& input) override {
    if (std::isnan(input)) {
      initialized_ = false;
      this->emit(NAN);
      return;
    }

    if (!initialized_) {
      state_ = input;
      initialized_ = true;
    } else {
      state_ = alpha_ * input + (1.0f - alpha_) * state_;
    }

    this->emit(state_);
  }
};

inline const String ConfigSchema(const EMA&) {
  return "{\"type\":\"object\",\"properties\":{\"alpha\":{\"title\":\"EMA alpha\",\"type\":\"number\"}}}";
}

}  // namespace sensesp
