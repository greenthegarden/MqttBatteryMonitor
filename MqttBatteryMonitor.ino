/*
 * Defaultly disabled. More details: https://docs.particle.io/reference/firmware/photon/#system-thread
 */
//SYSTEM_THREAD(ENABLED);

/*
 * Defaultly disabled. If BLE setup is enabled, when the Duo is in the Listening Mode, it will de-initialize and re-initialize the BT stack.
 * Then it broadcasts as a BLE peripheral, which enables you to set up the Duo via BLE using the RedBear Duo App or customized
 * App by following the BLE setup protocol: https://github.com/redbear/Duo/blob/master/docs/listening_mode_setup_protocol.md#ble-peripheral
 *
 * NOTE: If enabled and upon/after the Duo enters/leaves the Listening Mode, the BLE functionality in your application will not work properly.
 */
//BLE_SETUP(ENABLED);

/*
 * SYSTEM_MODE:
 *     - AUTOMATIC: Automatically try to connect to Wi-Fi and the Particle Cloud and handle the cloud messages.
 *     - SEMI_AUTOMATIC: Manually connect to Wi-Fi and the Particle Cloud, but automatically handle the cloud messages.
 *     - MANUAL: Manually connect to Wi-Fi and the Particle Cloud and handle the cloud messages.
 *
 * SYSTEM_MODE(AUTOMATIC) does not need to be called, because it is the default state.
 * However the user can invoke this method to make the mode explicit.
 * Learn more about system modes: https://docs.particle.io/reference/firmware/photon/#system-modes .
 */
#if defined(ARDUINO)
SYSTEM_MODE(SEMI_AUTOMATIC);
#endif

// Simple test of using sleep(SLEEP_MODE_DEEP, 30). This halts execution of your code, and when it
// wakes up again, it goes through setup() again with all variables cleared.

// Running this test without a battery connected to VBAT is interesting, however, because retained
// variables ARE preserved in deep sleep, even without a battery.

// IMPORTANT NOTE: If using retained variables to preserve values across deep sleep when NOT using
// a battery, be sure to tie VBAT to ground. This somewhat counter-intuitive step is necessary
// because otherwise when you first power up the device, the retained memory will not be initialized,
// so it will contain random values, which will probably confuse your code!

// Source: https://community.particle.io/t/sleep-mode-explained-using-code-samples/21173
// STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// Using JSON is a pain with standard library
// Information at https://docs.particle.io/reference/device-os/firmware/#json


/*
 *************** Configuration details ***************
 */

// for debugging.
#define DEBUG_CODE_SERIAL_OUTPUT true
#if defined(DEBUG_CODE_SERIAL_OUTPUT)
#define code_debug_print(fmt, ...) Serial.printf("[DEBUG] CODE " fmt, ##__VA_ARGS__)
#else /* !DEBUG_CODE_SERIAL_OUTPUT */
#define code_debug_print(fmt, ...) ((void)0)
#endif /* DEBUG_CODE_SERIAL_OUTPUT */

#include "secrets.h"

// set USE_SLEEP to false to run a delay rather than sleep duo
#ifndef USE_SLEEP
#define USE_SLEEP false
#endif

// set to true to run test
//    will not sleep
//    uses a different HA_MQTT_PREFIX to prevent publishing to homeassistant
//    uses a different SAMPLE_INTERVAL
#ifndef TEST_MODE
#define TEST_MODE true
#endif

#if TEST_MODE
#define DEBUG_MQTT_SERIAL_OUTPUT true
const unsigned long SAMPLE_INTERVAL = 15UL * 1000UL; // 15 seconds
#else
const unsigned long SAMPLE_INTERVAL = 1UL * 60UL * 1000UL; // 1 minutes
#endif


/*
 *************** Configure Duo ***************
 */

const int DUO_BLUE_LED = D7;
const float DUO_REF_VOLTAGE = 3.3;
const unsigned int DUO_ADC_RANGE = 1024;

/*
 *************** Configure MQTT ***************
 */

#include "MQTT.h"

// buffer for payload
const unsigned int PAYLOAD_LENGTH = 1023;
const unsigned int KEEP_ALIVE = 60;

char payload[PAYLOAD_LENGTH];

String client_id;

// This is called when a message is received. However, we do not use this feature in
// this project so it will be left empty
void callback(char* topic, byte* payload, unsigned int length)
{}

/**
 * if want to use IP address,
 * const uint8_t server[] = { XXX,XXX,XXX,XXX };
 * MQTT client(server, 1883, callback);
 * want to use domain name,
 * exp) iot.eclipse.org is Eclipse Open MQTT Broker: https://iot.eclipse.org/getting-started
 * MQTT client("mqtt.eclipse.org", 1883, callback);
 **/
// MQTT(const char *domain, uint16_t port, int maxpacketsize, int keepalive, void (*callback)(char *, uint8_t *, unsigned int), bool thread = false);
MQTT client(BROKER_IP, BROKER_PORT, PAYLOAD_LENGTH, KEEP_ALIVE, callback);

/*
 *************** Configure Home Assistant Integration ***************
 */

#include "HAMqttDevice.h"

#if TEST_MODE
char HA_MQTT_PREFIX[] = "ha";
# else
char HA_MQTT_PREFIX[] = "homeassistant";
#endif

HAMqttDevice duo_solar_monitor_state("Duo Solar Monitor State", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice battery_thumper_80ah_voltage("Thumper 80ah Voltage", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice solarpanel_350watt_current("Solar 350watt Current", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));

/*
 *************** Configure Device Message ***************
 */

// if formatted will return in form ab:cd:ef:01
// else will return in form abcded01
String macAddressToString(bool formatted=true)
{
  // the MAC address of your Wifi
  byte mac_buffer[6];

  // print your MAC address:
  WiFi.macAddress(mac_buffer);
  String mac_address = "";
  for (byte octet = 0; octet < 6; octet++)
  {
    mac_address += String(mac_buffer[octet], HEX);
    if (octet < 5 && formatted)
    {
      mac_address += ':';
    }
  }
  return mac_address;
}

void deviceConfig()
{
  duo_solar_monitor_state.enableStateTopic();
  duo_solar_monitor_state.enableAttributesTopic();
  duo_solar_monitor_state
      .addConfigVar("stat_cla", "measurement")
      .addConfigVar("val_tpl", "{{ value | int(0) }}")
      .addConfigVar("unit_of_meas", "bytes")
      .addConfigVar("dev", "{\"ids\": \"duo_solar_monitor\", \"name\": \"Duo Solar Monitor\", \"mdl\": \"Duo\", \"sa\": \"back_veranda\", \"mf\": \"Redbear\"}");
  duo_solar_monitor_state
      .addAttribute("sample_interval", String(SAMPLE_INTERVAL / 1000))
      .addAttribute("ssid", String(WiFi.SSID()))
      .addAttribute("ip_address", String(WiFi.localIP()))
      .addAttribute("mac_address", macAddressToString())
      .addAttribute("client_id", client_id);
}

void devicePublish()
{
  int memory = System.freeMemory();
  client.publish(duo_solar_monitor_state.getStateTopic(), String(memory));
}

/*
 *************** Configure Voltage Sensor ***************
 */

// Based on https://how2electronics.com/interfacing-0-25v-dc-voltage-sensor-with-arduino/
// voltage sensor configuration
const unsigned int VOLTAGE_SENSOR_PIN = A0;

// values for ADC voltage & input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;

// Resistor values in voltage divider (Ohms)
const float R1 = 30000.0;
const float R2 = 7500.0;

// ADC value
int adc_value = 0;

void voltageSensorConfig() {
  // set data pins as imputs
  pinMode(VOLTAGE_SENSOR_PIN, INPUT);

  battery_thumper_80ah_voltage.enableStateTopic();
  battery_thumper_80ah_voltage
      .addConfigVar("dev_cla", "voltage")
      .addConfigVar("stat_cla", "measurement")
      .addConfigVar("unit_of_meas", "V")
      .addConfigVar("val_tpl", "{{ value | float(0.0) }}")
      .addConfigVar("dev", "{\"ids\": \"battery_voltage\", \"name\": \"25V Voltage Divider\", \"mdl\": \"25V\", \"sa\": \"outside_kitchen\", \"mf\": \"MH-Electronic\"}");
  battery_thumper_80ah_voltage.enableAttributesTopic();
  battery_thumper_80ah_voltage
      .addAttribute("pin_data", String(VOLTAGE_SENSOR_PIN));
}

void voltageMeasurement()
{
  // Read the analog input`
  adc_value = analogRead(VOLTAGE_SENSOR_PIN);

  // Determine voltage at ADC input
  adc_voltage = (adc_value * DUO_REF_VOLTAGE) / float(DUO_ADC_RANGE);

  // Calculate voltage at divider input
  in_voltage = adc_voltage / (R2/(R1+R2));

  // publish reading
  String topic;
  topic = battery_thumper_80ah_voltage.getStateTopic();
  client.publish(topic, String(in_voltage));
}

/*
 *************** Configure current sensor ***************
 */

#include <ACS712.h>

// voltage sensor configuration
const unsigned int CURRENT_SENSOR_PIN = A4;

// Configure ACS712 Sensor
const unsigned long ACS712_PUBLISH_INTERVAL = 300000UL;
unsigned long acs712PreviousMillis = 0UL;

// Arduino UNO has 5.0 volt with a max ADC value of 1023 steps
// ACS712 5A  uses 185 mV per A
// ACS712 20A uses 100 mV per A
// ACS712 30A uses  66 mV per A
ACS712 ACS(CURRENT_SENSOR_PIN, DUO_REF_VOLTAGE, DUO_ADC_RANGE, 66.0);

void acs712Config() {
  // set data pins as imputs
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  // ACS.autoMidPoint();

  solarpanel_350watt_current.enableStateTopic();
  solarpanel_350watt_current
      .addConfigVar("dev_cla", "current")
      .addConfigVar("stat_cla", "measurement")
      .addConfigVar("unit_of_meas", "A")
      .addConfigVar("val_tpl", "{{ value | float(0.0) }}")
      .addConfigVar("dev", "{\"ids\": \"solar_current\", \"name\": \"ACS712 Current\", \"mdl\": \"30A\", \"sa\": \"outside_kitchen\", \"mf\": \"duinotech\"}");
  solarpanel_350watt_current.enableAttributesTopic();
  solarpanel_350watt_current
      .addAttribute("pin_data", String(CURRENT_SENSOR_PIN));
}

void acs712Measurement()
{
  // read value from the sensor
  float acs712_current = (float)ACS.mA_DC() / 1000.0;

  // publish reading
  String topic;
  topic = solarpanel_350watt_current.getStateTopic();
  client.publish(topic, String(acs712_current));
}

/*
 *************** Arduino Methods ***************
 */

// put your setup code here, to run once:
void setup() {

  code_debug_print(" Setup ...\n");

  pinMode(DUO_BLUE_LED, OUTPUT);

  // voltage sensor configuration
  voltageSensorConfig();
  // ACS712 sensor configuration
  acs712Config();

  // Connect to network
  WiFi.on();
  WiFi.setCredentials(SSID, PASSWORD);
  WiFi.connect();

  // wait for Wifi connection
  // while (WiFi.connecting())
  // {
  // }

  // Wait for wifi to be ready
  while (!WiFi.ready()) {}
  //   code_debug_print(" Did not connect to %s\n", SSID);

  // wait for wifi connection to be established
  delay(3000);

  // connect to broker with unique client ID based on MAC address
  client_id = String(CLIENTID) + "_" + macAddressToString(false);
  String willTopic = String("duo") + String("/") + macAddressToString(false) + String("/status");

  // client.connect(client_id);
  // return connect(id, NULL, NULL, 0, QOS0, 0, 0, true);
  // connect(id, user, pass, 0, QOS0, 0, 0, true);
  // bool connect(const char *id, const char *user, const char *pass, const char* willTopic, EMQTT_QOS willQos, uint8_t willRetain, const char* willMessage, bool cleanSession, MQTT_VERSION version = MQTT_V311);
  client.connect(client_id, NULL, NULL, willTopic, MQTT::EMQTT_QOS::QOS0, 0, "offline", true);

  if (client.isConnected())
  {
    client.publish(willTopic, "online", true);

    code_debug_print("Connected to broker");

    // device memory configuration
    // needs to occur after connection to network
    deviceConfig();

    // Publish config payloads
    client.publish(duo_solar_monitor_state.getConfigTopic(), duo_solar_monitor_state.getConfigPayload());
    client.publish(battery_thumper_80ah_voltage.getConfigTopic(), battery_thumper_80ah_voltage.getConfigPayload());
    client.publish(solarpanel_350watt_current.getConfigTopic(), solarpanel_350watt_current.getConfigPayload());

    // Publish attributes payloads
    client.publish(duo_solar_monitor_state.getAttributesTopic(), duo_solar_monitor_state.getAttributesPayload());
    client.publish(battery_thumper_80ah_voltage.getAttributesTopic(), battery_thumper_80ah_voltage.getAttributesPayload());
    client.publish(solarpanel_350watt_current.getAttributesTopic(), solarpanel_350watt_current.getAttributesPayload());
  }
}

// put your main code here, to run repeatedly:
void loop() {

  if (client.isConnected())
  {
    // client.loop();

    // Publish measurements
    devicePublish();
    voltageMeasurement();
    acs712Measurement();
  }

  delay(SAMPLE_INTERVAL);

}
