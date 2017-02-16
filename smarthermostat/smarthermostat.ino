/*
 *  SmarThermostat - operating code for the smart thermostat;
 *  i.e., for the environmental sensor and a relay board in one case.
 *  
 *  Copyright Alistair Young, 2017. All rights reserved.
 */

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <PubSubClient.h>

// Expose Espressif SDK functionality - wrapped in ifdef so that it still
// compiles on other platforms
#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

// Network -- WiFi and MQTT configuration -----------------

// WiFi setup

const char* ssid = "Arkane Systems" ;
const char* password = "NOTMYWIFIPASSWORD" ;

// MQTT server

const char* mqttserver = "ariadne.arkane-systems.lan";
const char* subTopic = "environment/central/command";
const char* pubStatusTopic = "environment/central/status";
const char* pubDataTopic = "environment/livingroom/data";

// Create ESP8266 WiFiClient class to connect to the MQTT server
WiFiClient wifiClient;

PubSubClient client (wifiClient);

// Pins -- pin definitions --------------------------------

#define statusPin 0
#define relay4pin 12
#define relay3pin 13
#define relay2pin 14
#define relay1pin 15

#define fanPin relay4pin
#define furnacePin relay3pin
#define acPin relay2pin

#define ON LOW
#define OFF HIGH

// Tickers -- periodic calls to scanning routines ---------

Ticker statusBlink;
Ticker envScan;

// Sensors -- environmental sensor declarations -----------

Adafruit_BME280 bme; // I2C
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// Setup -- set up the smart thermostat -------------------

void setup()
{
  // Setup the system.
  Serial.begin (9600);
  Serial.println ("SMARTHERMOSTAT: Setup");

  // Setup the LED pin.
  pinMode (statusPin, OUTPUT);

  // Setup the relay pins; start all off.
  digitalWrite (relay4pin, OFF);
  digitalWrite (relay3pin, OFF);
  digitalWrite (relay2pin, OFF);
  digitalWrite (relay1pin, OFF);
  
  pinMode (relay4pin, OUTPUT);
  pinMode (relay3pin, OUTPUT);
  pinMode (relay2pin, OUTPUT);
  pinMode (relay1pin, OUTPUT);

  // Set up the WiFi.
  wifi_station_set_hostname ("iot-smarthermostat");
  WiFi.begin (ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP: ");
  Serial.println(WiFi.localIP());

  // Setup MQTT client.
  client.setServer (mqttserver, 1883);
  client.setCallback (MQTT_callback);

  // Start sensors.
  // BME280
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // TSL2561
  if(!tsl.begin())
  {
    Serial.println("Could not find a valid TSL2561 sensor, check wiring!");
    while(1);
  }

  configureLightSensor();

  // Commence status blinking.
  statusBlink.attach (1.0, blink); 

  // Enable environmental monitor.
  envScan.attach (5.0, environmentalScan);
}

// Monitor -- query sensors and return data --------------

float light;
bool takeLightReading = true;

void environmentalScan ()
{
  float temperature = bme.readTemperature(); // C
  float pressure = bme.readPressure() / 100.0F; // hPa
  float humidity = bme.readHumidity(); // %

//  sensors_event_t event;
//  tsl.getEvent(&event);
//
//  /*
//   * The TSL2561 uses a memory caching scheme where the next measurement is triggered by reading the value 
//   * currently in memory. That works smoothly if you take continuous readings, but when you wait long periods 
//   * between readings the data in the output buffer goes stale.
//   * 
//   * After any long delay, you basically need to do one read to clear out the old data, then take the next 
//   * reading as a fresh measurement.
//   */
//
//  tsl.getEvent(&event);

  // Format as JSON.
  // Allocate memory pool for the object tree.
  StaticJsonBuffer<256> jsonBuffer;

  // Create the root of the object tree.
  JsonObject& root = jsonBuffer.createObject();

  // Add values to the object.
  root["light-level"] = /* event. */ light;
  root["temperature"] = temperature;
  root["pressure"] = pressure;
  root["humidity"] = humidity;

  char buffer[256];
  root.printTo(buffer, sizeof(buffer));

  Serial.println (buffer);

  int result = client.publish (pubDataTopic, buffer);

  // light hack
  takeLightReading = true;
}

// Config -- sensor configuration ------------------------

void configureLightSensor()
{
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);

  // display details
  sensor_t sensor;
  tsl.getSensor(&sensor);

  Serial.print  ("TSL Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("TSL Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("TSL Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");
  Serial.println ();
}

// LED -- LED management ---------------------------------

void blink()
{
  int state = digitalRead(statusPin);
  digitalWrite(statusPin, !state);
}

// Relays -- relay state management ----------------------

char status = 'O';

// All off.
void rOff ()
{
  // turn off all relays
  // do the blower last
  digitalWrite (furnacePin, OFF);
  digitalWrite (acPin, OFF);

  delay (250);

  digitalWrite (fanPin, OFF);
}

// Blower only.
void rBlower ()
{
  // make sure the others are off, then enable the blower
  digitalWrite (furnacePin, OFF);
  digitalWrite (acPin, OFF);

  digitalWrite (fanPin, ON);
}

// Heating.
void rHeating ()
{
  // disable AC, enable blower, then enable furnace
  digitalWrite (acPin, OFF);

  digitalWrite (fanPin, ON);
  digitalWrite (furnacePin, ON);
}

// Cooling.
void rCooling ()
{
  // disable furnace, enable blower, then enable ac
  digitalWrite (furnacePin, OFF);

  digitalWrite (fanPin, ON);
  digitalWrite (acPin, ON);
}


// Loop -- run the MQTT loop forever ---------------------

void loop()
{
  // MQTT server connect and loop
  if (!client.connected())
  {
    MQTT_connect();
  }

  // light hack
  if (takeLightReading)
  {
    sensors_event_t event;
    tsl.getEvent(&event);

    light = event.light;

    takeLightReading = false;
  }

  client.loop();
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("SmarThermostat"))
    {
      Serial.println("connected");
      
      // ... and subscribe to topic
      client.subscribe(subTopic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Process incoming MQTT messages.
void MQTT_callback (char* topic, byte* payload, unsigned int length)
{
  Serial.print("Received MQTT topic=");
  Serial.println(topic);
  
  // check topic
  if (strcmp (topic, subTopic) != 0)
    return;

  // check length
  if (length != 1)
    return;

  // if we are asked for a status report, give it
  if ((char)payload[0] == 'S')
  {
    char str[2] = "\0";
    str[0] = status;
    client.publish (pubStatusTopic, str);
    return;
  }

  status = (char)payload[0];
  switch (status)
  {
    case 'O':
      rOff();
      return;

    case 'F':
      rBlower ();
      return;

    case 'H':
      rHeating ();
      return;

    case 'C':
      rCooling ();
      return;
  }
  
  Serial.println("unknown command, noop");
}

