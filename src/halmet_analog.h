#ifndef __SRC_HALMET_ANALOG_H__
#define __SRC_HALMET_ANALOG_H__

#include <Adafruit_ADS1X15.h>

#include "sensesp/sensors/sensor.h"

using namespace sensesp;

// HALMET voltage divider scale factor
const float kVoltageDividerScale = 33.3 / 3.3;

FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115, int channel,
                                 String name, bool enable_signalk_output = true);

class ADS1115VoltageInput : public FloatSensor {
 public:
  ADS1115VoltageInput(Adafruit_ADS1115* ads1115, int channel,
                      String config_path, unsigned int read_interval = 1000,
                      float calibration_factor = 1.0)
      : FloatSensor(config_path),
        ads1115_{ads1115},
        channel_{channel},
        read_interval_{read_interval},
        calibration_factor_{calibration_factor} {
    load_configuration();

    repeat_reaction_ = set_repeat_reaction(read_interval_);
  }

  void update() {
    int16_t adc_output = ads1115_->readADC_SingleEnded(channel_);
    float adc_output_volts = ads1115_->computeVolts(adc_output);
    this->emit(calibration_factor_ * kVoltageDividerScale * adc_output_volts);
  }

  void get_configuration(JsonObject& root) {
    root["read_delay"] = read_interval_;
    root["calibration_value"] = calibration_factor_;
  };

  String get_config_schema() {
    const char SCHEMA[] = R"###({
      "type": "object",
      "properties": {
          "read_interval": { "title": "Read interval", "type": "number", "description": "Number of milliseconds between each reading" },
          "calibration_factor": { "title": "Calibration factor", "type": "number", "description": "Scale factor to fix the input calibration" }
      }
    })###";

    return SCHEMA;
  }

  bool set_configuration(const JsonObject& config) {
    String expected[] = {"read_interval"};
    for (auto str : expected) {
      if (!config.containsKey(str)) {
        return false;
      }
      if (str == "calibration_factor" &&
          config.containsKey("calibration_factor")) {
        calibration_factor_ = config["calibration_factor"];
      }
    }
    read_interval_ = config["read_interval"];
    return true;
  }

 protected:
  RepeatReaction* repeat_reaction_ = nullptr;

  RepeatReaction* set_repeat_reaction(unsigned int read_interval) {
    if (repeat_reaction_ != nullptr) {
      repeat_reaction_->remove();
    }

    repeat_reaction_ =
        ReactESP::app->onRepeat(read_interval, [this]() { this->update(); });
    return repeat_reaction_;
  }

 private:
  Adafruit_ADS1115* ads1115_;
  int channel_;
  unsigned int read_interval_;
  float calibration_factor_;
};

#endif
