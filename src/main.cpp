// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

// Comment out this line to disable NMEA 2000 output.
#define ENABLE_NMEA2000_OUTPUT

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef ENABLE_NMEA2000_OUTPUT
#include <NMEA2000_esp32.h>
#endif

#include "halmet_analog.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "n2k_senders.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

// I2C pins on HALMET
const int kSDAPin = 21;
const int kSCLPin = 22;

// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// CAN bus (NMEA 2000) pins on HALMET
const gpio_num_t kCANRxPin = GPIO_NUM_18;
const gpio_num_t kCANTxPin = GPIO_NUM_19;

// HALMET digital input pins
const int kDigitalInputPin1 = GPIO_NUM_23;
const int kDigitalInputPin2 = GPIO_NUM_25;
const int kDigitalInputPin3 = GPIO_NUM_27;
const int kDigitalInputPin4 = GPIO_NUM_26;

TwoWire* i2c;
Adafruit_SSD1306* display;

reactesp::ReactESP app;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
tNMEA2000* nmea2000;
N2kEngineParameterRapidSender* engine_d1_sender;
N2kFluidLevelSender* tank_a1_sender;
#endif

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcSetup(0, kTestOutputFrequency, 13);
  // Attach the channel to the GPIO pin to be controlled
  ledcAttachPin(kTestOutputPin, 0);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(0, 4096);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
  // Initialize NMEA 2000

  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() { nmea2000->ParseMessages(); });
#endif  // ENABLE_NMEA2000_OUTPUT

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("halmet")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(&app, sensesp_app, &display, i2c);

  // Connect the tank senders
  auto tank_a1_volume = ConnectTankSender(ads1115, 0, "A1");
  // auto tank_a2_volume = ConnectTankSender(ads1115, 1, "B");
  // auto tank_a3_volume = ConnectTankSender(ads1115, 2, "C");
  // auto tank_a4_volume = ConnectTankSender(ads1115, 3, "D");

  // Connect the tacho senders. Engine name is "main".
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "main");

  // Connect the alarm inputs
  auto alarm_d2_input = ConnectAlarmSender(kDigitalInputPin2, "D2");
  // auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "3");
  // auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "4");

  // Update the alarm states based on the input value changes
  alarm_d2_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // alarm_d3_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  // alarm_d4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

#ifdef ENABLE_NMEA2000_OUTPUT
  // Connect outputs to the N2k senders
  engine_d1_sender = new N2kEngineParameterRapidSender(
      "/NMEA 2000/Engine 1", 0, nmea2000);  // Engine 1, instance 0
  tacho_d1_frequency->connect_to(&(engine_d1_sender->engine_speed_consumer_));

  // Tank 1, instance 0. Capacity 200 liters.
  tank_a1_sender =
      new N2kFluidLevelSender("/NMEA 2000/Tank 1", 0, N2kft_Fuel, 200, nmea2000);
  tank_a1_volume->connect_to(&(tank_a1_sender->tank_level_consumer_));
#endif

  // Connect the outputs to the display
  if (display_present) {
    app.onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Add display updaters for tank and RPM values
    tank_a1_volume->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 2, "Tank A1", 100 * value); }));

    tacho_d1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM D1", 60 * value); }));

    // Create a poor man's "christmas tree" display for the alarms
    app.onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
  }

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
