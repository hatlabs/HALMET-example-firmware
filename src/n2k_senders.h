#ifndef HALMET_SRC_N2K_SENDERS_H_
#define HALMET_SRC_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include "expiring_value.h"
#include "sensesp/system/configurable.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/startable.h"
#include "sensesp_app.h"

using namespace sensesp;

class N2kEngineParameterRapidSender : public Configurable, public Startable {
 public:
  N2kEngineParameterRapidSender(String config_path, uint8_t engine_instance,
                                tNMEA2000* nmea2000)
      : Configurable{config_path},
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

 private:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  uint8_t engine_instance_;
  ExpiringValue<double> engine_speed_;
  ExpiringValue<double> engine_boost_pressure_;
  ExpiringValue<int8_t> engine_tilt_trim_;
};

#endif  // HALMET_SRC_N2K_SENDERS_H_
