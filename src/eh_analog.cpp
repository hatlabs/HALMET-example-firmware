#include "eh_analog.h"

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"

// Interpolator to convert the input voltage to a fuel level ratio
class FuelLevelInterpolator : public CurveInterpolator {
 public:
  FuelLevelInterpolator(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // If saved configuration hasn't been loaded, use the default values
    if (samples.empty()) {
      // Populate a lookup table to translate the ohm values returned by
      // the tank sender to fill level
      clear_samples();

      // This curve corresponds to a European standard (0-180 ohms) tank sender
      // addSample(CurveInterpolator::Sample(knownOhmValue, knownFuelLevel));
      add_sample(CurveInterpolator::Sample(0, 0));
      add_sample(CurveInterpolator::Sample(180., 1));
      add_sample(CurveInterpolator::Sample(300., 1));
    }
  }
};

// ADS1115 input hardware scale factor
const float kAnalogInputScale = 29. / 2.048;

// Default fuel tank size, in m3
const float kTankDefaultSize = 120. / 1000;

FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115, int channel,
                                 String name) {
  const uint ads_read_delay = 500;  // ms

  char config_path[80];
  char sk_path[80];
  char meta_display_name[80];
  char meta_description[80];

  snprintf(config_path, sizeof(config_path), "/analog/input_%s/voltage",
           name.c_str());
  auto input_voltage = new RepeatSensor<float>(1000, [ads1115, channel]() {
    int16_t adc_output = ads1115->readADC_SingleEnded(channel);
    float adc_output_volts = ads1115->computeVolts(adc_output);
    return kAnalogInputScale * adc_output_volts;
  });

  snprintf(config_path, sizeof(config_path), "/analog/%s/inputVoltage/skPath",
           name.c_str());
  snprintf(sk_path, sizeof(sk_path), "tanks.fuel.%s.senderVoltage",
           name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Input voltage %s",
           name.c_str());
  snprintf(meta_description, sizeof(meta_description),
           "Calculated voltage at input %s", name.c_str());
  auto input_voltage_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("V", meta_display_name, meta_description));

  snprintf(config_path, sizeof(config_path),
           "/analog/%s/fuelTank/currentLevel/interpolator", name.c_str());
  auto tank_level = new FuelLevelInterpolator(config_path);

  snprintf(config_path, sizeof(config_path),
           "/analog/%s/fuelTank/currentLevel/skPath", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "tanks.fuel.%s.currentLevel",
           name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Tank %s level",
           name.c_str());
  snprintf(meta_description, sizeof(meta_description),
           "Calculated tank %s level", name.c_str());
  auto tank_level_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("ratio", meta_display_name, meta_description));

  snprintf(config_path, sizeof(config_path),
           "/analog/%s/fuelTank/currentVolume/linear", name.c_str());
  auto tank_volume = new Linear(kTankDefaultSize, 0, config_path);

  snprintf(config_path, sizeof(config_path),
           "/analog/%s/fuelTank/currentVolume/skPath", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "tanks.fuel.%s.currentVolume",
           name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Tank %s volume",
           name.c_str());
  snprintf(meta_description, sizeof(meta_description),
           "Calculated tank %s remaining volume", name.c_str());
  auto tank_volume_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("m3", meta_display_name, meta_description));

  input_voltage->connect_to(input_voltage_sk_output);

  input_voltage->connect_to(tank_level)->connect_to(tank_level_sk_output);

  tank_level->connect_to(tank_volume)->connect_to(tank_volume_sk_output);

  return tank_level;
}
