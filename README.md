# smart-thermo
Arduino code for the SmarThermostat, a set of environmental sensors and relays coupled to an Adafruit HUZZAH ESP8266.

The SmarThermostat gets temperature, pressure, and humidity information from a BME80 sensor and light level information from a
TSL2561 sensor, which it reports back to the home automation server via MQTT (using the topic _environment/<roomname>/data_ ). It
receives commands - and, on request, reports current settings - to change the settings of the relays controlling the fan, furnace, and
air conditioning unit over the topics _environment/central/command_ and _environment/central/status_ , respectively, and actions them
accordingly.

A secondary routine flashes LED 0 on the Feather flash periodically to indicate normal function.

The code for the SmarThermostat is freely available under the MIT License, should you happen to want it.
