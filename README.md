# Redbear Duo MQTT Solar and Battery Monitor

Project to use a [Redbear Duo](https://github.com/redbear/Duo) microprocessing board to gather and publish solar current and battery voltage via MQTT. The structure of the topics are compatible with [Home Assistant MQTT Discovery](https://www.home-assistant.io/docs/mqtt/discovery/).

## Hardware

### Redbear Duo

The [Redbear Duo](https://github.com/redbear/Duo) was acquired via a [Kickstarter campaign](https://www.kickstarter.com/projects/redbearinc/redbear-duo-a-small-and-powerful-wi-fi-ble-iot-boa). The board is compatible with Particle boards so utilised [information about using MQTT with Particle Boards](https://www.digikey.jp/ja/maker/projects/how-to-build-a-photon-mqtt-logger/876ce49a8f914f0799a0f8b94519acc1).

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

[0-25V Voltage Sensor Module](https://how2electronics.com/interfacing-0-25v-dc-voltage-sensor-with-arduino/) is utilised to monitor the battery voltage which has the following thresholds, which are within the 16 volt limit when using the sensor with a 3.3 volt microprocessor, as in this case.

| Charge Level | Voltage |
| ------------ | ------- |
| Full (100%)  | 12.7    |
| 80%          | 12.5    |
| 50%          | 12.2    |
| Flat         | 10.5    |

### Current Sensor

A [ACS712: Hall-Effect-Based Linear Current Sensor](https://www.allegromicro.com/en/Products/Sense/Current-Sensor-ICs/Zero-To-Fifty-Amp-Integrated-Conductor-Sensor-ICs/ACS712) is utilised to measure the solar panel current. A 350Watt panel is utilised which limits current to 30Amps. The [ACS712](https://github.com/RobTillaart/ACS712) library is used to get the measurements from the sensor. More information is available from [Arduino](https://create.arduino.cc/projecthub/instrumentation-system/acs712-current-sensor-87b4a6).

## Software

The [Arduino IDE](https://www.arduino.cc/en/software) is required to be used to support the Redbear Duo board. I was not able to add the board definition to the [PlatformIO](https://platformio.org/) my preferred Arduino development platform. Support the the Redbear Duo board is by following the [installation guide](https://github.com/redbear/Duo/blob/master/docs/arduino_board_package_installation_guide.md).

The source code for the project is hosted on [Github](https://github.com/greenthegarden/RedbearDuoMqttMoisturePublisher).

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

### Scheduler

Scheduler project hosted on [Github](https://github.com/arduino-libraries/Scheduler.git).

### Home Assistant Discovery

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

Config Topic: `ha/sensor/duo_solar_monitor_state/config`

Config Message structure:

```json
{
  "~":"ha/sensor/duo_solar_monitor_state",
  "name":"Duo Solar Monitor State",
  "unique_id":"duo_solar_monitor_state",
  "stat_t":"~/state",
  "json_attr_t":"~/attr",
  "stat_cla":"measurement",
  "val_tpl":"{{ value | int(0) }}",
  "unit_of_meas":"bytes",
  "dev":{
    "ids": "duo_solar_monitor",
    "name": "Duo Solar Monitor",
    "mdl": "Duo",
    "sa": "outside_kitchen",
    "mf": "Redbear"
  }
}
```

Attribute Topic: `homeassistant/sensor/duo_solar_monitor_state/attr`

Attribute Message structure:

```json
{
  "sample_interval":"15",
  "ssid":"videoAtHome-2.4g",
  "ip_address":"192.168.1.145",
  "mac_address":"94:a1:a2:fd:73:67",
  "client_id":"duo_battery_94a1a2fd7367"
}
```

State Topic: `ha/sensor/duo_solar_monitor_state/state`

State Message structure: int

### Battery Voltage

Config Topic: `ha/sensor/thumper_80ah_voltage/config`

Config Message structure:

```json
{
  "~":"ha/sensor/thumper_80ah_voltage",
  "name":"Thumper 80ah Voltage",
  "unique_id":"thumper_80ah_voltage",
  "stat_t":"~/state",
  "dev_cla":"voltage",
  "stat_cla":"measurement",
  "unit_of_meas":"V",
  "val_tpl":"{{ value | float(0.0) }}",
  "dev":{
    "ids": "battery_voltage",
    "name": "25V Voltage Divider",
    "mdl": "25V",
    "sa": "outside_kitchen",
    "mf": "MH-Electronic"
  },
  "json_attr_t":"~/attr"
}
```

Attribute Topic: `ha/sensor/thumper_80ah_voltage/attr`

Attribute Topic structure:

```json
{
  "pin_data":"10"
}
```

State Topic: `ha/sensor/thumper_80ah_voltage/state`

State Message structure: float

### Solar Current

Config Topic: `ha/sensor/solar_350watt_current/config`

Config Message structure:

```json
{
  "~":"ha/sensor/solar_350watt_current",
  "name":"Solar 350watt Current",
  "unique_id":"solar_350watt_current",
  "stat_t":"~/state",
  "dev_cla":"current",
  "stat_cla":"measurement",
  "unit_of_meas":"A",
  "val_tpl":"{{ value | float(0.0) }}",
  "dev": {
    "ids": "solar_current",
    "name": "ACS712 Current",
    "mdl": "30A",
    "sa": "outside_kitchen",
    "mf": "duinotech"},
  "json_attr_t":"~/attr"
}
```

Attribute Topic: `ha/sensor/solar_350watt_current/attr`

Attribute Message structure:

```json
{
  "pin_data":"14"
}
```

State Topic: `ha/sensor/solar_350watt_current/state`

State Message structure: float
