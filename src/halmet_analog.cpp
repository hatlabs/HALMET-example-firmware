#include "halmet_analog.h"

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"

// HALMET constant measurement current (A)
const float kMeasurementCurrent = 0.01;

// Default fuel tank size, in m3
const float kTankDefaultSize = 120. / 1000;

FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115, int channel,
                                 String name, bool enable_signalk_output) {
  const uint ads_read_delay = 500;  // ms

  // Configure the sender resistance sensor

  auto sender_resistance =
      new RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kVoltageDividerScale * adc_output_volts / kMeasurementCurrent;
      });

  if (enable_signalk_output) {
    char resistance_sk_config_path[80];
    snprintf(resistance_sk_config_path, sizeof(resistance_sk_config_path),
             "/Tank %s/Resistance SK Path", name.c_str());
    char resistance_sk_path[80];
    snprintf(resistance_sk_path, sizeof(resistance_sk_path),
             "tanks.fuel.%s.senderResistance", name.c_str());
    char resistance_meta_display_name[80];
    snprintf(resistance_meta_display_name, sizeof(resistance_meta_display_name),
             "Resistance %s", name.c_str());
    char resistance_meta_description[80];
    snprintf(resistance_meta_description, sizeof(resistance_meta_description),
             "Measured tank %s sender resistance", name.c_str());

    auto sender_resistance_sk_output =
        new SKOutputFloat(resistance_sk_path, resistance_sk_config_path,
                          new SKMetadata("ohm", resistance_meta_display_name,
                                         resistance_meta_description));
    sender_resistance->connect_to(sender_resistance_sk_output);
  }

  // Configure the piecewise linear interpolator for the tank level (ratio)

  char curve_config_path[80];
  snprintf(curve_config_path, sizeof(curve_config_path), "/Tank %s/Level Curve",
           name.c_str());

  auto tank_level = (new CurveInterpolator(nullptr, curve_config_path))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Fuel Level (ratio)");

  if (tank_level->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    tank_level->clear_samples();
    tank_level->add_sample(CurveInterpolator::Sample(0, 0));
    tank_level->add_sample(CurveInterpolator::Sample(180., 1));
    tank_level->add_sample(CurveInterpolator::Sample(1000., 1));
  }

  sender_resistance->connect_to(tank_level);

  if (enable_signalk_output) {
    char level_config_path[80];
    snprintf(level_config_path, sizeof(level_config_path),
             "/Tank %s/Current Level SK Path", name.c_str());
    char level_sk_path[80];
    snprintf(level_sk_path, sizeof(level_sk_path), "tanks.%s.currentLevel",
             name.c_str());
    char level_meta_display_name[80];
    snprintf(level_meta_display_name, sizeof(level_meta_display_name),
             "Tank %s level", name.c_str());
    char level_meta_description[80];
    snprintf(level_meta_description, sizeof(level_meta_description),
             "Tank %s level", name.c_str());

    auto tank_level_sk_output =
        new SKOutputFloat(level_sk_path, level_config_path,
                          new SKMetadata("ratio", level_meta_display_name,
                                         level_meta_description));
    tank_level->connect_to(tank_level_sk_output);
  }

  // Configure the linear transform for the tank volume

  char volume_config_path[80];
  snprintf(volume_config_path, sizeof(volume_config_path),
           "/Tank %s/Total Volume", name.c_str());
  auto tank_volume = new Linear(kTankDefaultSize, 0, volume_config_path);
  tank_level->connect_to(tank_volume);

  if (enable_signalk_output) {
    char volume_sk_config_path[80];
    snprintf(volume_sk_config_path, sizeof(volume_sk_config_path),
             "/Tank %s/Current Volume SK Path", name.c_str());
    char volume_sk_path[80];
    snprintf(volume_sk_path, sizeof(volume_sk_path), "tanks.%s.currentVolume",
             name.c_str());
    char volume_meta_display_name[80];
    snprintf(volume_meta_display_name, sizeof(volume_meta_display_name),
             "Tank %s volume", name.c_str());
    char volume_meta_description[80];
    snprintf(volume_meta_description, sizeof(volume_meta_description),
             "Calculated tank %s remaining volume", name.c_str());

    auto tank_volume_sk_output =
        new SKOutputFloat(volume_sk_path, volume_sk_config_path,
                          new SKMetadata("m3", volume_meta_display_name,
                                         volume_meta_description));
    tank_volume->connect_to(tank_volume_sk_output);
  }

  return tank_level;
}
