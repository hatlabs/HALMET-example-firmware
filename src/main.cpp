// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "eh_analog.h"
#include "eh_digital.h"
#include "eh_display.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

// I2C pins on SH-ESP32
const int kSDAPin = 16;
const int kSCLPin = 17;

// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// CAN bus (NMEA 2000) pins on SH-ESP32
const int kCANRxPin = GPIO_NUM_34;
const int kCANTxPin = GPIO_NUM_32;

// Engine hat digital input pins
const int kDigitalInputPin1 = GPIO_NUM_15;
const int kDigitalInputPin2 = GPIO_NUM_13;
const int kDigitalInputPin3 = GPIO_NUM_14;
const int kDigitalInputPin4 = GPIO_NUM_12;

// Test output pin configuration
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_18;
// repetition interval in ms; corresponds to 1000/(2*5)=100 Hz
const int kTestOutputInterval = 5;
#endif

TwoWire* i2c;
Adafruit_SSD1306* display;

reactesp::ReactESP app;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Convenience function to print the addresses found on the I2C bus
void ScanI2C(TwoWire* i2c) {
  uint8_t error, address;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    i2c->beginTransmission(address);
    error = i2c->endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("done");
}

void ToggleTestOutputPin(void * parameter) {
  while (true) {
    digitalWrite(kTestOutputPin, !digitalRead(kTestOutputPin));
    delay(kTestOutputInterval);
  }
}

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  ScanI2C(i2c);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  xTaskCreate(ToggleTestOutputPin, "toggler", 2048, NULL, 1, NULL);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("engine-hat")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();


  // Initialize the OLED display
  bool display_present = InitializeSSD1306(&app, sensesp_app, &display, i2c);


  // Connect the tank senders
  auto tank_a_volume = ConnectTankSender(ads1115, 0, "A");

  // Connect the tacho senders
  auto tacho_1_frequency = ConnectTachoSender(kDigitalInputPin1, "1");

  // Connect the alarm inputs
  auto alarm_2_input = ConnectAlarmSender(kDigitalInputPin2, "2");
  auto alarm_3_input = ConnectAlarmSender(kDigitalInputPin3, "3");
  auto alarm_4_input = ConnectAlarmSender(kDigitalInputPin4, "4");

  alarm_2_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  alarm_3_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  alarm_4_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

  // Connect the outputs to the display
  if (display_present) {
    app.onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Add display updaters for temperature values
    tank_a_volume->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 2, "Tank A", 100 * value); }));

    tacho_1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM 1", 60 * value); }));

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
