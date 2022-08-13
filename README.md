# MQTT Solar and Battery Monitor

Project to use a [Redbear Duo](https://github.com/redbear/Duo) microprocessing board to gather and publish solar current and battery voltage via MQTT. The structure of the topics are compatible with [Home Assistant MQTT Discovery](https://www.home-assistant.io/docs/mqtt/discovery/).

## Hardware

### Redbear Duo

The [Redbear Duo](https://github.com/redbear/Duo) was acquired via a [Kickstarter campaign](https://www.kickstarter.com/projects/redbearinc/redbear-duo-a-small-and-powerful-wi-fi-ble-iot-boa). The board is compatible with Particle boards. Information about using MQTT with Particle Boards is provided at https://www.digikey.jp/ja/maker/projects/how-to-build-a-photon-mqtt-logger/876ce49a8f914f0799a0f8b94519acc1.

LED Status Indicators

| RGB LED State | Duo State |
| ------------- | --------- |
| flashing blue | listening mode |
| flashing green | attempting connection to access point |
| pulsing green | 
| solid green | listening mode |
| solid yellow | |

### Hardware Layout

Redbear Duo is installed on a 

### Voltage Sensor

[0-25V Voltage Sensor Module](https://how2electronics.com/interfacing-0-25v-dc-voltage-sensor-with-arduino/) is utilised to monitor the battery voltage which has the following thresholds, which are within the 16 volt limit when using he sensor with a 3.3 volt microprocessor, as in this case.

| Charge Level | Voltage |
| ------------ | ------- |
| Full (100%)  | 12.7    |
| 80%          | 12.5    |
| 50%          | 12.2    |
| Flat         | 11.8    |

### Current Sensor

A [ACS712: Hall-Effect-Based Linear Current Sensor](https://www.allegromicro.com/en/Products/Sense/Current-Sensor-ICs/Zero-To-Fifty-Amp-Integrated-Conductor-Sensor-ICs/ACS712) is utilised to measure the solar panel current. A 350Watt panel is utilised which limits current to 30Amps. The [ACS712](https://github.com/RobTillaart/ACS712) library is used to get the measurements from the sensor.

## Software

The [Arduino IDE](https://www.arduino.cc/en/software) is required to be used to support the Redbear Duo board. I was not able to add the board definition to the [PlatformIO](https://platformio.org/) my preferred Arduino development platform. Support the the Redbear Duo board is by following the [installation guide](<https://github.com/redbear/Duo/blob/master/docs/arduino_board_package_installation_guide.md).

The source code for the project is hosted at https://github.com/greenthegarden/RedbearDuoMqttMoisturePublisher.

Before compiling the code, add a file named `secrets.h` to the root project directory with the following format

```cpp
//SSID (network name)
char SSID[] = "";
// Network password
char PASSWORD[] = "";
// MQTT Broker details
char BROKER_IP[] = "";
const uint16_t BROKER_PORT = 1883;
char CLIENTID[] = "duo_moisture";
// Device ID
char DEVICE_ID[] = "3a001d000d47353033323637";
```

## Libraries

The project uses modified versions of the following libraries. Due to the modifications made the library source code has been added directly to this project. A script, `libraries/get_libraries.sh` had been provided to get updated copies of the libraries.

### HAMqttDevice

The [HAMqttDevice library](https://www.arduino.cc/reference/en/libraries/hamqttdevice/) is used to provide support for [Home Assistant](https://www.home-assistant.io/) [MQTT Discovery](https://www.home-assistant.io/docs/mqtt/discovery/). It has been modified to support the Particle version of the Vector library utilised by the Redbear Duo.

### MQTT

THe [MQTT library](https://github.com/hirotakaster/MQTT) was found to be the most compatible MQTT library for the Redbear Duo.

## MQTT Messages

Five Home Assistant devices are defined using

```cpp
HAMqttDevice duo_solar_monitor_state("Duo Solar Monitor State", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice battery_thumper_80ah_voltage("Thumper 80ah Voltage", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice solarpanel_350watt_current("Solar 350watt Current", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
```

### Device

Config Topic: `homeassistant/sensor/duo_solar_monitor_state/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/duo_solar_monitor_state",
  "name": "Duo Solar Monitor State",
  "unique_id": "duo_solar_monitor_state",
  "stat_t": "~/state",
  "json_attr_t": "~/attr",
  "stat_cla": "measurement",
  "unit_of_meas": "bytes",
  "dev": {
    "ids": "duo_solar_monitor",
    "name": "Solar Monitor",
    "mdl": "Duo",
    "sa": "back_veranda",
    "mf": "Redbear"
  }
}
```

Attribute Topic: `homeassistant/sensor/duo_solar_monitor_state/attr`

Attribute Message structure: 

```json
{
  "sample_interval": "60",
  "ssid": "videoAtHome-2.4g",
  "ip_address": "192.168.1.165",
  "mac_address": "94:a1:a2:fd:71:f5",
  "client_id": "duo_moisture_94a1a2fd71f5"
}
```

State Topic: `homeassistant/sensor/soil_device_memory/state`

State Message structure: int

### Soil Temperature

Config Topic: `homeassistant/sensor/soil_sht10_temperature/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_sht10_temperature",
  "name": "Soil SHT10 Temperature",
  "unique_id": "soil_sht10_temperature",
  "dev_cla": "temperature",
  "stat_cla": "measurement",
  "unit_of_meas": "°C",
  "stat_t": "duo/sensor/soil_sht10",
  "val_tpl": "{{ value_json.temperature | int(0) }}",
  "dev":{
    "ids": "temp_hum_sensor",
    "name": "Moisture Sensor",
    "mdl": "SHT10",
    "sa": "garden",
    "mf": "Seeed"
  }
}
```

State Topic: `duo/sensor/soil_sht10`

State Message structure: 

```json
{
  "temperature": 24.75,
  "humidity": 68.517
}
```

### Soil Humidity

Config Topic: `homeassistant/sensor/soil_sht10_humidity/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_sht10_humidity",
  "name": "Soil SHT10 Humidity",
  "unique_id": "soil_sht10_humidity",
  "dev_cla": "humidity",
  "stat_cla": "measurement",
  "unit_of_meas": "%",
  "stat_t": "duo/sensor/soil_sht10",
  "val_tpl": "{{ value_json.humidity | int(0) }}",
  "dev": {
    "ids": "temp_hum_sensor",
    "name": "Moisture Sensor",
    "mdl": "SHT10",
    "sa": "garden",
    "mf": "Seeed"
  }
}
```

State Topic: `duo/sensor/soil_sht10`

State Message structure:

```json
{
  "temperature": 24.75,
  "humidity": 68.517
}
```

### Soil Moisture Sensor 1

Config Topic: `homeassistant/sensor/soil_moisture_1/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_moisture_1",
  "name": "Soil Moisture 1",
  "unique_id": "soil_moisture_1",
  "stat_t": "~/state",
  "stat_cla": "measurement",
  "val_tpl": "{{ value | int(0) }}",
  "dev": {
    "ids": "moisture_sensor_1",
    "name": "Capacitive Soil Moisture Sensor",
    "mdl": "V1.2",
    "sa": "garden",
    "mf": "DIYMORE.CC"
  },
  "json_attr_t": "~/attr"
}
```

Attribute Topic: `homeassistant/sensor/soil_moisture_1/attr`

Attribute Message structure:

```json
{
  "pin_data": "11",
  "power_pin": "1"
}
```

State Topic: `homeassistant/sensor/soil_moisture_1/state`

State Message structure: int

### Soil Moisture Sensor 2

Config Topic: `homeassistant/sensor/soil_moisture_2/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_moisture_2",
  "name": "Soil Moisture 2",
  "unique_id": "soil_moisture_2",
  "stat_t": "~/state",
  "stat_cla": "measurement",
  "val_tpl": "{{ value | int(0) }}",
  "dev": {
    "ids": "moisture_sensor_2",
    "name": "Capacitive Soil Moisture Sensor",
    "mdl": "V1.2",
    "sa": "garden",
    "mf": "DIYMORE.CC"
  }
}
```

Attribute Topic: `homeassistant/sensor/soil_moisture_2/attr`

Attribute Message structure:

```json
{
  "pin_data": "13",
  "power_pin": "2"
}
```

State Topic: `homeassistant/sensor/soil_moisture_2/state`

State Message structure: int