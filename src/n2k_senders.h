#ifndef HALMET_SRC_N2K_SENDERS_H_
#define HALMET_SRC_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include "expiring_value.h"
#include "sensesp/system/configurable.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/startable.h"
#include "sensesp_app.h"

namespace sensesp {

class N2kEngineParameterRapidSender : public Startable, public Configurable {
 public:
  N2kEngineParameterRapidSender(String config_path, uint8_t engine_instance,
                                tNMEA2000* nmea2000)
      : Configurable{config_path},
        Startable(),
        engine_instance_{engine_instance},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{1000}           // In ms. When the inputs expire.
  {
    engine_speed_ = ExpiringValue<double>(N2kDoubleNA, expiry_, N2kDoubleNA);
    engine_boost_pressure_ =
        ExpiringValue<double>(N2kDoubleNA, expiry_, N2kDoubleNA);
    engine_tilt_trim_ = ExpiringValue<int8_t>(N2kInt8NA, expiry_, N2kInt8NA);
  }

  virtual void start() override {
    ReactESP::app->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      uint8_t instance = this->engine_instance_;
      double speed = this->engine_speed_.get();
      double boost_pressure = this->engine_boost_pressure_.get();
      double tilt_trim = this->engine_tilt_trim_.get();
      // At the moment, the PGN is sent regardless of whether all the values
      // are invalid or not.
      SetN2kEngineParamRapid(N2kMsg, instance, speed, boost_pressure,
                             tilt_trim);
      this->nmea2000_->SendMsg(N2kMsg);
    });
  }

  LambdaConsumer<double> engine_speed_consumer_{[this](double value) {
    // Internally we measure engine speed in Hz (revolutions per second) but
    // NMEA 2000 rpm.
    this->engine_speed_.update(60 * value);
  }};
  LambdaConsumer<double> engine_boost_pressure_consumer_{
      [this](double value) { this->engine_boost_pressure_.update(value); }};
  LambdaConsumer<int8_t> engine_tilt_trim_consumer_{
      [this](int8_t value) { this->engine_tilt_trim_.update(value); }};

  virtual String get_config_schema() override {
    return R"###({
    "type": "object",
    "properties": {
      "engine_instance": { "title": "Engine instance", "type": "integer", "description": "Engine NMEA 2000 instance number (0-253)" }
    }
  })###";
  }

  virtual bool set_configuration(const JsonObject& config) override {
    String expected[] = {"engine_instance"};
    for (auto str : expected) {
      if (!config.containsKey(str)) {
        debugE("N2kEngineParameterRapidSender: Missing configuration key %s",
               str.c_str());
        return false;
      }
    }
    engine_instance_ = config["engine_instance"];
    return true;
  }

  virtual void get_configuration(JsonObject& config) override {
    config["engine_instance"] = engine_instance_;
  }

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  uint8_t engine_instance_;
  ExpiringValue<double> engine_speed_;
  ExpiringValue<double> engine_boost_pressure_;
  ExpiringValue<int8_t> engine_tilt_trim_;
};

class N2kFluidLevelSender : public Configurable, public Startable {
 public:
  N2kFluidLevelSender(String config_path, uint8_t tank_instance,
                      tN2kFluidType tank_type, double tank_capacity,
                      tNMEA2000* nmea2000)
      : Configurable{config_path},
        tank_instance_{tank_instance},
        tank_type_{tank_type},
        tank_capacity_{tank_capacity},
        nmea2000_{nmea2000},
        repeat_interval_{2500},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{10000}           // In ms. When the inputs expire.
  {
    tank_level_ = ExpiringValue<double>(N2kDoubleNA, expiry_, N2kDoubleNA);
  }

  virtual void start() override {
    ReactESP::app->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      uint8_t instance = this->tank_instance_;
      double level = this->tank_level_.get();
      // At the moment, the PGN is sent regardless of whether all the values
      // are invalid or not.
      SetN2kFluidLevel(N2kMsg, instance, this->tank_type_, level,
                       this->tank_capacity_);
      this->nmea2000_->SendMsg(N2kMsg);
    });
  }

  LambdaConsumer<double> tank_level_consumer_{[this](double value) {
    // Internal tank level is a ratio, NMEA 2000 wants a percentage.
    this->tank_level_.update(100. * value);
  }};

  virtual String get_config_schema() override {
    return R"###({
      "type": "object",
      "properties": {
        "tank_instance": { "title": "Tank instance", "type": "integer", "description": "Tank NMEA 2000 instance number (0-13)" },
        "tank_type": { "title": "Tank type", "type": "integer", "description": "Tank type (0-13)" },
        "tank_capacity": { "title": "Tank capacity", "type": "number", "description": "Tank capacity (liters)" }
      }
    })###";
  };

  virtual bool set_configuration(const JsonObject& config) override {
    String expected[] = {"tank_instance", "tank_type", "tank_capacity"};
    for (auto str : expected) {
      if (!config.containsKey(str)) {
        debugE("N2kFluidLevelSender: Missing configuration key %s",
               str.c_str());
        return false;
      }
    }
    tank_instance_ = config["tank_instance"];
    tank_type_ = config["tank_type"];
    tank_capacity_ = config["tank_capacity"];
    return true;
  }

  virtual void get_configuration(JsonObject& config) override {
    config["tank_instance"] = tank_instance_;
    config["tank_type"] = tank_type_;
    config["tank_capacity"] = tank_capacity_;
  }

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  uint8_t tank_instance_;
  tN2kFluidType tank_type_;
  double tank_capacity_;              // in liters
  ExpiringValue<double> tank_level_;  // in percent
};

}  // namespace sensesp

#endif  // HALMET_SRC_N2K_SENDERS_H_
