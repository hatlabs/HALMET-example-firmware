#ifndef HALMET_SRC_N2K_SENDERS_H_
#define HALMET_SRC_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include "expiring_value.h"
#include "sensesp/system/configurable.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/startable.h"

namespace sensesp {

/**
 * @brief Transmit NMEA 2000 PGN 127488: Engine Parameters, Rapid Update
 *
 */
class N2kEngineParameterRapidSender : public Startable, public Configurable {
 public:
  N2kEngineParameterRapidSender(String config_path, uint8_t engine_instance,
                                tNMEA2000* nmea2000)
      : Configurable{config_path},
        Startable(),
        engine_instance_{engine_instance},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{1000},          // In ms. When the inputs expire.
        engine_speed_{N2kDoubleNA, expiry_, N2kDoubleNA},
        engine_boost_pressure_{N2kDoubleNA, expiry_, N2kDoubleNA},
        engine_tilt_trim_{N2kInt8NA, expiry_, N2kInt8NA} {}

  virtual void start() override {
    ReactESP::app->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      // At the moment, the PGN is sent regardless of whether all the values
      // are invalid or not.
      SetN2kEngineParamRapid(
          N2kMsg, this->engine_instance_, this->engine_speed_.get(),
          this->engine_boost_pressure_.get(), this->engine_tilt_trim_.get());
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

/**
 * @brief Transmit NMEA 2000 PGN 127489: Engine Parameters, Dynamic
 *
 */
class N2kEngineParameterDynamicSender : public Startable, public Configurable {
 public:
  N2kEngineParameterDynamicSender(String config_path, uint8_t engine_instance,
                                  tNMEA2000* nmea2000)
      : Configurable{config_path},
        Startable(),
        engine_instance_{engine_instance},
        nmea2000_{nmea2000},
        repeat_interval_{500},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{5000},          // In ms. When the inputs expire.
        oil_pressure_{N2kDoubleNA, expiry_, N2kDoubleNA},
        oil_temperature_{N2kDoubleNA, expiry_, N2kDoubleNA},
        temperature_{N2kDoubleNA, expiry_, N2kDoubleNA},
        alternator_potential_{N2kDoubleNA, expiry_, N2kDoubleNA},
        fuel_rate_{N2kDoubleNA, expiry_, N2kDoubleNA},
        total_engine_hours_{N2kUInt32NA, expiry_, N2kUInt32NA},
        coolant_pressure_{N2kDoubleNA, expiry_, N2kDoubleNA},
        fuel_pressure_{N2kDoubleNA, expiry_, N2kDoubleNA},
        engine_load_{N2kInt8NA, expiry_, N2kInt8NA},
        engine_torque_{N2kInt16NA, expiry_, N2kInt16NA},
        check_engine_{false, expiry_, false},
        over_temperature_{false, expiry_, false},
        low_oil_pressure_{false, expiry_, false},
        low_oil_level_{false, expiry_, false},
        low_fuel_pressure_{false, expiry_, false},
        low_system_voltage_{false, expiry_, false},
        low_coolant_level_{false, expiry_, false},
        water_flow_{false, expiry_, false},
        water_in_fuel_{false, expiry_, false},
        charge_indicator_{false, expiry_, false},
        preheat_indicator_{false, expiry_, false},
        high_boost_pressure_{false, expiry_, false},
        rev_limit_exceeded_{false, expiry_, false},
        egr_system_{false, expiry_, false},
        throttle_position_sensor_{false, expiry_, false},
        emergency_stop_{false, expiry_, false},
        warning_level_1_{false, expiry_, false},
        warning_level_2_{false, expiry_, false},
        power_reduction_{false, expiry_, false},
        maintenance_needed_{false, expiry_, false},
        engine_comm_error_{false, expiry_, false},
        sub_or_secondary_throttle_{false, expiry_, false},
        neutral_start_protect_{false, expiry_, false},
        engine_shutting_down_{false, expiry_, false} {}

  virtual void start() override {
    ReactESP::app->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      SetN2kEngineDynamicParam(
          N2kMsg, this->engine_instance_, this->oil_pressure_.get(),
          this->oil_temperature_.get(), this->temperature_.get(),
          this->alternator_potential_.get(), this->fuel_rate_.get(),
          this->total_engine_hours_.get(), this->coolant_pressure_.get(),
          this->fuel_pressure_.get(), this->engine_load_.get(),
          this->engine_torque_.get(), this->get_engine_status_1(),
          this->get_engine_status_2());
      this->nmea2000_->SendMsg(N2kMsg);
    });
  }

// Define a macro for defining the consumer functions for each parameter.
#define DEFINE_CONSUMER(name, type) \
  LambdaConsumer<type> name##_consumer_{[this](type value) { \
    this->name##_.update(value); \
  }};
  DEFINE_CONSUMER(oil_pressure, double)
  DEFINE_CONSUMER(oil_temperature, double)
  DEFINE_CONSUMER(temperature, double)
  DEFINE_CONSUMER(alternator_potential, double)
  DEFINE_CONSUMER(fuel_rate, double)
  DEFINE_CONSUMER(total_engine_hours, uint32_t)
  DEFINE_CONSUMER(coolant_pressure, double)
  DEFINE_CONSUMER(fuel_pressure, double)
  DEFINE_CONSUMER(engine_load, int)
  DEFINE_CONSUMER(engine_torque, int)
  DEFINE_CONSUMER(check_engine, bool)
  DEFINE_CONSUMER(over_temperature, bool)
  DEFINE_CONSUMER(low_oil_pressure, bool)
  DEFINE_CONSUMER(low_oil_level, bool)
  DEFINE_CONSUMER(low_fuel_pressure, bool)
  DEFINE_CONSUMER(low_system_voltage, bool)
  DEFINE_CONSUMER(low_coolant_level, bool)
  DEFINE_CONSUMER(water_flow, bool)
  DEFINE_CONSUMER(water_in_fuel, bool)
  DEFINE_CONSUMER(charge_indicator, bool)
  DEFINE_CONSUMER(preheat_indicator, bool)
  DEFINE_CONSUMER(high_boost_pressure, bool)
  DEFINE_CONSUMER(rev_limit_exceeded, bool)
  DEFINE_CONSUMER(egr_system, bool)
  DEFINE_CONSUMER(throttle_position_sensor, bool)
  DEFINE_CONSUMER(emergency_stop, bool)
  DEFINE_CONSUMER(warning_level_1, bool)
  DEFINE_CONSUMER(warning_level_2, bool)
  DEFINE_CONSUMER(power_reduction, bool)
  DEFINE_CONSUMER(maintenance_needed, bool)
  DEFINE_CONSUMER(engine_comm_error, bool)
  DEFINE_CONSUMER(sub_or_secondary_throttle, bool)
  DEFINE_CONSUMER(neutral_start_protect, bool)
  DEFINE_CONSUMER(engine_shutting_down, bool)
#undef DEFINE_CONSUMER

 protected:
  uint16_t get_engine_status_1() {
    uint16_t status = 0;
    status |= 0x0001 * check_engine_.get();
    status |= 0x0002 * over_temperature_.get();
    status |= 0x0004 * low_oil_pressure_.get();
    status |= 0x0008 * low_oil_level_.get();
    status |= 0x0010 * low_fuel_pressure_.get();
    status |= 0x0020 * low_system_voltage_.get();
    status |= 0x0040 * low_coolant_level_.get();
    status |= 0x0080 * water_flow_.get();
    status |= 0x0100 * water_in_fuel_.get();
    status |= 0x0200 * charge_indicator_.get();
    status |= 0x0400 * preheat_indicator_.get();
    status |= 0x0800 * high_boost_pressure_.get();
    status |= 0x1000 * rev_limit_exceeded_.get();
    status |= 0x2000 * egr_system_.get();
    status |= 0x4000 * throttle_position_sensor_.get();
    status |= 0x8000 * emergency_stop_.get();

    return status;
  }

  uint16_t get_engine_status_2() {
    uint16_t status = 0;
    status |= 0x0001 * warning_level_1_.get();
    status |= 0x0002 * warning_level_2_.get();
    status |= 0x0004 * power_reduction_.get();
    status |= 0x0008 * maintenance_needed_.get();
    status |= 0x0010 * engine_comm_error_.get();
    status |= 0x0020 * sub_or_secondary_throttle_.get();
    status |= 0x0040 * neutral_start_protect_.get();
    status |= 0x0080 * engine_shutting_down_.get();
    return status;
  }

  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  uint8_t engine_instance_;
  // Data to be transmitted
  ExpiringValue<double> oil_pressure_;
  ExpiringValue<double> oil_temperature_;
  ExpiringValue<double> temperature_;
  ExpiringValue<double> alternator_potential_;
  ExpiringValue<double> fuel_rate_;
  ExpiringValue<uint32_t> total_engine_hours_;
  ExpiringValue<double> coolant_pressure_;
  ExpiringValue<double> fuel_pressure_;
  ExpiringValue<int> engine_load_;
  ExpiringValue<int> engine_torque_;
  // Engine status 1 fields
  ExpiringValue<bool> check_engine_;
  ExpiringValue<bool> over_temperature_;
  ExpiringValue<bool> low_oil_pressure_;
  ExpiringValue<bool> low_oil_level_;
  ExpiringValue<bool> low_fuel_pressure_;
  ExpiringValue<bool> low_system_voltage_;
  ExpiringValue<bool> low_coolant_level_;
  ExpiringValue<bool> water_flow_;
  ExpiringValue<bool> water_in_fuel_;
  ExpiringValue<bool> charge_indicator_;
  ExpiringValue<bool> preheat_indicator_;
  ExpiringValue<bool> high_boost_pressure_;
  ExpiringValue<bool> rev_limit_exceeded_;
  ExpiringValue<bool> egr_system_;
  ExpiringValue<bool> throttle_position_sensor_;
  ExpiringValue<bool> emergency_stop_;
  // Engine status 2 fields
  ExpiringValue<bool> warning_level_1_;
  ExpiringValue<bool> warning_level_2_;
  ExpiringValue<bool> power_reduction_;
  ExpiringValue<bool> maintenance_needed_;
  ExpiringValue<bool> engine_comm_error_;
  ExpiringValue<bool> sub_or_secondary_throttle_;
  ExpiringValue<bool> neutral_start_protect_;
  ExpiringValue<bool> engine_shutting_down_;
};

/**
 * @brief Transmit NMEA 2000 PGN 127505: Fluid Level
 *
 */
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
      // At the moment, the PGN is sent regardless of whether all the values
      // are invalid or not.
      SetN2kFluidLevel(N2kMsg, this->tank_instance_, this->tank_type_,
                       this->tank_level_.get(), this->tank_capacity_);
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
