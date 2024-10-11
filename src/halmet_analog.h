#ifndef HALMET_ANALOG_H_
#define HALMET_ANALOG_H_

#include <Adafruit_ADS1X15.h>

#include "sensesp/sensors/sensor.h"
#include "sensesp_base_app.h"

namespace halmet {

// HALMET voltage divider scale factor
const float kVoltageDividerScale = 33.3 / 3.3;

sensesp::FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115,
                                          int channel, const String& name,
                                          const String& sk_id, int sort_order,
                                          bool enable_signalk_output = true);

class ADS1115VoltageInput : public sensesp::FloatSensor {
 public:
  ADS1115VoltageInput(Adafruit_ADS1115* ads1115, int channel,
                      const String& config_path,
                      unsigned int read_interval = 1000,
                      float calibration_factor = 1.0)
      : sensesp::FloatSensor(config_path),
        ads1115_{ads1115},
        channel_{channel},
        read_interval_{read_interval},
        calibration_factor_{calibration_factor} {
    load();

    repeat_event_ = set_repeat_event(read_interval_);
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
    if (!config["read_interval"].is<int>()) {
      return false;
    }
    if (config["calibration_factor"].is<float>()) {
      calibration_factor_ = config["calibration_factor"];
    }
    read_interval_ = config["read_interval"];
    return true;
  }

 protected:
  reactesp::RepeatEvent* repeat_event_ = nullptr;

  reactesp::RepeatEvent* set_repeat_event(unsigned int read_interval) {
    if (repeat_event_ != nullptr) {
      repeat_event_->remove(sensesp::event_loop());
    }

    repeat_event_ = sensesp::event_loop()->onRepeat(
        read_interval, [this]() { this->update(); });
    return repeat_event_;
  }

 private:
  Adafruit_ADS1115* ads1115_;
  int channel_;
  unsigned int read_interval_;
  float calibration_factor_;
};

}  // namespace halmet

#endif
