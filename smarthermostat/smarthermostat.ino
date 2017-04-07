// **************************************************************************
// * SMARTHERMOSTAT - OPERATING CODE FOR THE SMART THERMOSTAT               *
// *                                                                        *
// * INCLUDES:                                                              *
// *                                                                        *
// *   ENVIRONMENTAL SENSORS                                                *
// *   RELAY CONTROL                                                        *
// *                                                                        *
// * COPYRIGHT ALISTAIR YOUNG, 2017. ALL RIGHTS RESERVED.                   *
// *                                                                        *
// **************************************************************************

// TASK SCHEDULER PARAMETERS ************************************************

// Task scheduling is done using Anatoli Arkhipenko's cooperative multitasking
// library, found at:
//
// https://github.com/arkhipenko/TaskScheduler
//
// Since all non-template functionality added to this sketch is intended to be
// implemented using Tasks, I highly recommend careful study of the documentation
// at https://github.com/arkhipenko/TaskScheduler/blob/master/extras/TaskScheduler.pdf
// before implementing sketches based on this template.

// Go into idle sleep mode (until next 1 ms timer tick) when task chain empty.
#define _TASK_SLEEP_ON_IDLE_RUN

// Support Status Requests for tasks.
#define _TASK_STATUS_REQUEST

// Support task IDs and control points.
#define _TASK_WDT_IDS

// Support local task storage.
#define _TASK_LTS_POINTER

#include <TaskScheduler.h>

// **************************************************************************
// * USER SETTINGS SECTION - CONFIGURE AS DESIRED                           *
// **************************************************************************

// Device name (also functions as hostname for WiFi purposes).
#define DEV_NAME          "iot-smarthermostat"

// Debugging mode enabled; causes libraries to output debug information to
// the serial monitor.
#define DEBUG_ENABLED     1

// Enables support for the OLED FeatherWing ( https://www.adafruit.com/products/2900 ).
#define OLED_ENABLED      0

// This sketch template does not, by default, specifically support the three
// optional buttons on the OLED FeatherWing. Button C (on pin 2) is available
// for use as standard. Button B (on pin 16) functions as an "early-wake"
// button if the deep sleep functionality is used and therefore pin 16 is
// connected to RST; if this functionality is not used and this connection
// is not made, button B can be used as standard.

// Making use of button A (on pin 0) required removing the status blink
// functionality entirely from this template, since it makes use of pin 0
// as an output.

// Wireless network SSID and passphrase.
#define WLAN_SSID         "YOUR-SSID-HERE"
#define WLAN_PASS         "YOUR-PASSPHRASE-HERE"

// MQTT server details.
#define MQTT_SERVER       "YOUR-MQTT-SERVER-HERE"
#define MQTT_SERVERPORT   1883
#define MQTT_CLIENTID     DEV_NAME
#define MQTT_USERNAME     ""
#define MQTT_PASSWORD     ""

// Watchdog timer configuration (MQTT topic and interval (s).
#define WATCHDOG_FEED     "watchdog/devices"
#define WATCHDOG_INTERVAL 60

// Battery monitoring enabled.
#define VBAT_ENABLED      0

// Low battery reports posted to this MQTT topic.
#define BATTERY_FEED      "status/battery"

// Battery monitoring assumes that the VBat pin of the Feather HUZZAH is
// connected to the analog input pin via a 1 M/200K voltage divider, as per
// the implementation comment below.

// Over-the-Air update settings.
#define OTA_PORT          8266
#define OTA_PASSWORD      "YOUR-OTA-PASSWORD-HERE"

// **************************************************************************
// * STANDARD TEMPLATE CODE - DO NOT EDIT                                   *
// **************************************************************************

// INCLUDES *****************************************************************

// OLED support.
#if OLED_ENABLED == 1
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <Adafruit_FeatherOLED_WiFi.h>
#endif

// WiFi support.
#include <ESP8266WiFi.h>

// Expose Espressif SDK functionality
extern "C"
{
#include "user_interface.h"
}

// MQTT support.
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <ArduinoJson.h>

// OTA update support.
#include <ArduinoOTA.h>

// SYSTEM DEFINITIONS *******************************************************

// Board pins.
//
// Available for GPIO: 4, 5, 12, 13, 14, 15 (see caveats)
//
// If I2C is used, pins 4 and 5 are reserved.
//
// If SPI is used, pins 12, 13, and 14 are reserved.
//
// Pin 15 MUST NOT be pulled HIGH on startup.
//
// 16 is used by the deep sleep functionality and/or optional button B; 0 by
// the status blink functionality and/or optional button A; 2 may be used for
// GPIO if neither the blue LED nor button C is used.

#define BOARD_RED_LED_PIN         0
#define BOARD_BLUE_LED_PIN        2
#define BOARD_WAKE_PIN            16

#define LED_OFF                   HIGH
#define LED_ON                    LOW

// TASK DECLARATIONS ********************************************************

void _wifiMonitor ();
bool _wifiStartup ();

// Task for the WiFi monitoring function.
Task _tWifi (1UL * TASK_SECOND, TASK_FOREVER, &_wifiMonitor, NULL, false, &_wifiStartup, NULL);

void _mqttMonitor ();
bool _mqttStartup ();
void _mqttShutdown ();

// Task for the MQTT monitoring/keepalive function.
Task _tMqtt (30UL * TASK_SECOND, TASK_FOREVER, &_mqttMonitor, NULL, false, &_mqttStartup, &_mqttShutdown);

void _bark ();

// Task for the Watchdog Timer function.
Task _tWatchdog (WATCHDOG_INTERVAL * TASK_SECOND, TASK_FOREVER, &_bark, NULL, false, NULL, NULL);

#if VBAT_ENABLED == 1
void _batteryUpdate () ;

// Task for the battery update function.
Task _tBattery (5UL * TASK_SECOND, TASK_FOREVER, &_batteryUpdate, NULL, false, NULL, NULL);
#endif

void _blinkCallback ();
bool _blinkEnable ();
void _blinkDisable ();

// Task for the status blink function.
Task _tBlink(1UL * TASK_SECOND, TASK_FOREVER, &_blinkCallback, NULL, false, &_blinkEnable, &_blinkDisable);

void _handleOTA ();
bool _setupOTA ();

// Task for the OTA update function.
Task _tOTA(0, TASK_FOREVER, &_handleOTA, NULL, false, &_setupOTA, NULL);

// OLED SUPPORT *************************************************************

#if OLED_ENABLED == 1

// OLED FeatherWing object
Adafruit_FeatherOLED_WiFi _oled = Adafruit_FeatherOLED_WiFi();

// Perform initial setup of the OLED display.
void _oled_initialize()
{
  // Set up the OLED display.
  _oled.init();
  _oled.clearDisplay();

#if VBAT_ENABLED == 1
  // Enable battery display; and
  // Get a VBAT reading before refreshing (if available)
  _oled.setBatteryVisible(true);
  _batteryUpdate();
#else
  // Disable battery display.
  _oled.setBatteryVisible(false);
#endif

  // Enable the WiFi displays.
  _oled.setConnectedVisible (true);
  _oled.setRSSIVisible (true);
  _oled.setRSSIIcon (true);
  _oled.setRSSIAsPercentage (true);
  _oled.setIPAddressVisible (true);

  // Refresh the screen.
  _oled.refreshIcons();
  _oled.clearMsgArea();
  _oled.println (DEV_NAME);
  _oled.println ("booting...");
  _oled.display();
}

// Returns the `Adafruit_FeatherOLED_WiFi` object used by the template to access
// the OLED display. This class is that from the FeatherOLED library, which see
// ( https://learn.adafruit.com/adafruit-oled-featherwing/featheroled-library#featheroled-library )
// for details, available to be used when more complex operations are required
// than the simple helper functions below permit. Note that using this directly,
// it is easy to interfere with the use of the status areas above and below
// that the template code uses; _caveat_ developer!
Adafruit_FeatherOLED_WiFi getOled ()
{
  return _oled;
}

// Clears the middle 128x15 pixels message area on the OLED display.
void oledClear ()
{
  _oled.clearMsgArea();
}

// Prints a string to one line of the message area of the OLED display.
void oledPrintln (char * line)
{
  _oled.println (line);
}

// Causes the OLED display message area to update.
void oledDisplay ()
{
  _oled.display ();
}

#endif

// WIFI SUPPORT *************************************************************

// Initialize the WiFi connection.
bool _wifiStartup ()
{
#if OLED_ENABLED == 1
  _oled.setConnected(false);
  _oled.refreshIcons();
  _oled.clearMsgArea();
  _oled.println ("Connecting to WLAN...");
  _oled.println (WLAN_SSID);
  _oled.display();
#endif

  Serial.print ("Connecting to WLAN:"); Serial.println(WLAN_SSID);

  // WiFi to station mode only.
  WiFi.mode (WIFI_STA);

  // Set hostname to device name.
  wifi_station_set_hostname (DEV_NAME);

  // Configure no auto-connect on power on (since we do this manually here); but
  // auto-reconnect if connection fails.
  WiFi.setAutoConnect (false);
  WiFi.setAutoReconnect (true);

  // Connect to WiFi network.
  WiFi.begin (WLAN_SSID, WLAN_PASS);

  return true;
}

// Saved status from last monitor pass.
wl_status_t _lastKnownStatus = WL_IDLE_STATUS;

// Monitor the WiFi connection and display status.
void _wifiMonitor ()
{
  wl_status_t currentStatus = WiFi.status();

  if (currentStatus == WL_CONNECTED)
  {
    // We are connected.
    if (_lastKnownStatus != WL_CONNECTED)
    {
      // Newly connected.

#if OLED_ENABLED == 1
      // Clear display (of connecting message or reconnecting message).
      _oled.clearMsgArea ();

      // Set as connected; set IP address.
      _oled.setConnected (true);
      _oled.setIPAddress (WiFi.localIP());
#endif

      Serial.println ("Connected.");
      Serial.print ("IP address: "); Serial.println (WiFi.localIP());

      // Enable MQTT task, since WiFi now available.
      _tMqtt.enable();
    }

    // Fallthrough, or continue to be connected.

#if OLED_ENABLED == 1
    // Update RSSI.
    _oled.setRSSI (WiFi.RSSI());
#endif
    
  }
  else if (currentStatus != WL_IDLE_STATUS && currentStatus != WL_DISCONNECTED)
  {
    // Not connected.
    char * message;

    switch (currentStatus)
    {
      case WL_NO_SSID_AVAIL:
        message = "Could not find network: ";
        break;

      case WL_CONNECT_FAILED:
        message = "Incorrect passphrase for: ";
        break;

      default:
        message = "Unknown WiFi error on: ";
        break;
    }

#if OLED_ENABLED == 1
    _oled.clearMsgArea ();
    _oled.println (message);
    _oled.println (WLAN_SSID);
    _oled.display ();
#endif

    Serial.print (message); Serial.println (WLAN_SSID);

  // Disable MQTT task until reconnected.
    _tMqtt.disable ();

  // Reconnecting is done automatically.
  }

  _lastKnownStatus = currentStatus;

#if OLED_ENABLED == 1
  // Refresh icons.
  _oled.refreshIcons ();
#endif
}

// MQTT SUPPORT *************************************************************

// WiFi client for use by MQTT
WiFiClient _mqttWiFiClient;

// MQTT client class; this instance is available to application code.
Adafruit_MQTT_Client mqttClient (&_mqttWiFiClient, MQTT_SERVER, MQTT_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

// Helper function for connecting.
bool _mqttConnect ()
{
  int8_t ret = mqttClient.connect();

  if (ret != 0)
  {
    // Connection failed.
    mqttClient.disconnect ();

    char * error;

    switch (ret)
    {
      case 1:
        error = "Wrong protocol";
        break;

      case 2:
        error = "ID rejected";
        break;

      case 3:
        error = "Server unavailable";
        break;

      case 4:
        error = "Bad username/password";
        break;

      case 5:
        error = "Not authenticated";
        break;

      case 6:
        error = "Failed to subscribe";
        break;

      default:
        error = "Unknown";
        break;
    }

#if OLED_ENABLED == 1
    _oled.clearMsgArea ();
    _oled.println ("MQTT error:");
    _oled.println (error);
    _oled.display ();
#endif

    Serial.print ("MQTT error: "); Serial.println(error);

    return false;
  }

  return true;
}

// Initial connection to the MQTT server.
bool _mqttStartup ()
{
#if OLED_ENABLED == 1
  _oled.clearMsgArea ();
  _oled.println ("Connecting to MQTT");
  _oled.println (MQTT_SERVER);
  _oled.display ();
#endif

  Serial.print ("Connecting to MQTT: "); Serial.println(MQTT_SERVER);

  // Try to connect to the MQTT server.
  if (_mqttConnect ())
  {
    // Connected successfully. Clear connecting message.

#if OLED_ENABLED == 1
    _oled.clearMsgArea();
#endif

    Serial.println ("MQTT connected");

    // Start the watchdog task.
    _tWatchdog.enable();
  }

  // If we don't connect successfully, then the error is already displayed;
  // and we let _mqttMonitor handle it in 30s as if it were a reconnection.
}

// Monitor the status of the MQTT server and reconnect if necessary.
void _mqttMonitor ()
{
  // Ping the MQTT server to be surte we remain connected (keepalive).
  if (! mqttClient.ping())
  {
    // Ping failed. So...
    if (! mqttClient.connected ())
    {
      // Not connected. Disable watchdog task until reconnected.
      _tWatchdog.disable();

      // Confirm disconnect.
      mqttClient.disconnect ();

#if OLED_ENABLED == 1
      _oled.clearMsgArea ();
      _oled.println ("MQTT disconnected");
      _oled.println ("Reconnecting");
      _oled.display ();
#endif

      Serial.println ("MQTT disconnected; reconnecting.");

      // Attempt reconnect
      if (_mqttConnect())
      {
        // Reconnected successfully. Clear connecting message.

#if OLED_ENABLED == 1
        _oled.clearMsgArea();
#endif

        Serial.println ("MQTT reconnected");

        // Start the watchdog task.
        _tWatchdog.enable();
      }

      // Otherwise, we pick it up again next time through.
    }
  }
}

// Shut down the connection to the MQTT server.
void _mqttShutdown ()
{
  mqttClient.disconnect ();
}

// WATCHDOG SUPPORT *********************************************************

// MQTT topic for watchdog timer reports.
Adafruit_MQTT_Publish _watchdog = Adafruit_MQTT_Publish (&mqttClient, WATCHDOG_FEED);

// Send a watchdog timer message ("bark").
void _bark ()
{
  // Format as JSON
  // Allocate memory pool for the object tree.
  StaticJsonBuffer<128> jsonBuffer;

  // Create the root of the object tree.
  JsonObject& root = jsonBuffer.createObject();

  // Add values to the object.
  root["device"] = DEV_NAME ;
  root["notify"] = "bark" ;
  root["uptime"] = millis();

  char buffer[128];
  root.printTo (buffer, sizeof(buffer));

  if (!_watchdog.publish(buffer))
  {
    Serial.println ("bark failed");
  }
}

// BATTERY MONITORING *******************************************************

#if VBAT_ENABLED == 1

// The analog input pin.
#define VBAT_PIN                  A0

// MQTT topic for battery level reports.
Adafruit_MQTT_Publish _battlevel = Adafruit_MQTT_Publish (&mqttClient, BATTERY_FEED);

// Update battery status, including OLED display if enabled and sending
// status MQTT message at appropriate interval.
void _batteryUpdate ()
{
  float mvPerLev = 5.426357F;
  
  // read the battery level from the ESP8266 analog in pin.
  // analog read level is 10 bit 0-1023 (0V-1V).
  // our 1M & 220K voltage divider takes the max
  // lipo value of 4.2V and drops it to 0.758V max.
  // this means our min analog read value should be 580 (3.14V)
  // and the max analog read value should be 774 (4.2V).
  int level = analogRead(A0);

  // convert battery level to voltage in mV
  float voltage = ((float)level * mvPerLev);

#if OLED_ENABLED == 1
  // set display
  _oled.setBattery (voltage / 1000);
  _oled.refreshIcons ();
#endif

  // send battery level to MQTT server, but only once per two minutes.
  if (((_tBattery.getRunCounter() % 24) == 0) && (!_tBattery.isFirstIteration()))
  {
    // Convert battery level to percentage.
    int percent = (level - 574) / 2;

    // Format as JSON
    // Allocate memory pool for the object tree.
    StaticJsonBuffer<128> jsonBuffer;

    // Create the root of the object tree.
    JsonObject& root = jsonBuffer.createObject();

    // Add values to the object.
    root["device"] = DEV_NAME ;
    root["level"] = percent;

    char buffer[128];
    root.printTo (buffer, sizeof(buffer));

    if (!_battlevel.publish(buffer))
    {
      Serial.println ("level report failed");
    }
  }
}

#endif

// STATUS BLINKING **********************************************************

// Enable method for the status blink task; this configures the RED LED
// pin for output.
bool _blinkEnable()
{
  pinMode (BOARD_RED_LED_PIN, OUTPUT);
  digitalWrite (BOARD_RED_LED_PIN, LED_OFF);

  return true;
}

// Callback method for the status blink task; this toggles the RED LED
// pin, causing this LED to flash periodically to indicate operational status.
void _blinkCallback()
{
  int state = digitalRead(BOARD_RED_LED_PIN);
  digitalWrite(BOARD_RED_LED_PIN, !state);
}

// Disable method for the status blink task; this ensures that the RED LED
// is turned off when the task is disabled.
void _blinkDisable()
{
  digitalWrite(BOARD_RED_LED_PIN, LED_OFF);
}

// SLEEP MODE SUPPORT *******************************************************

// Send an MQTT notification of impending sleep ("yawn") on the watchdog channel.
void _longSleepNotify (int msec)
{
  // Format as JSON
  // Allocate memory pool for the object tree.
  StaticJsonBuffer<128> jsonBuffer;

  // Create the root of the object tree.
  JsonObject& root = jsonBuffer.createObject();

  // Add values to the object.
  root["device"] = DEV_NAME ;
  root["notify"] = "yawn";
  root["msec"] = msec;

  char buffer[128];
  root.printTo (buffer, sizeof(buffer));

  if (!_watchdog.publish(buffer))
  {
    Serial.println ("yawn failed");
  }
}

// Go into deep sleep mode for the specified number of milliseconds. If this length of
// time exceeds half the configured WATCHDOG_INTERVAL, an impending-sleep notification
// ("yawn") will be sent on the watchdog MQTT topic. If the OLED display is enabled,
// a notification message ("yawn...") is displayed to indicate that the Feather is 
// asleep locally, since status blinking ceases with the red LED off while in deep sleep.
void deepSleep (int msec)
{
  if (msec > (WATCHDOG_INTERVAL * 1000) / 2)
  {
    // Long sleep (send notification)
    _longSleepNotify (msec);

#if OLED_ENABLED == 1
      _oled.clearMsgArea ();
      _oled.println ("yawn...");
      _oled.display ();
#endif

      Serial.println ("yawn...");
  }

  Serial.println ("Sleeping.");

  ESP.deepSleep (msec * 1000);
}

// OTA UPDATE SUPPORT *******************************************************

// Set up over-the-air update support.
bool _setupOTA ()
{
  // Set up OTA parameters.
  ArduinoOTA.setHostname(DEV_NAME);
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setPassword((const char *)OTA_PASSWORD);

  // OTA update reporting functions.
  ArduinoOTA.onStart([]()
  {
#if OLED_ENABLED == 1
    _oled.clearMsgArea ();
    _oled.println ("OTA Update:");
    _oled.println ("starting");
    _oled.display ();
#endif

    Serial.println ("OTA Update: starting.");

    application_otasafe ();
  });
  
  ArduinoOTA.onEnd([]()
  {
#if OLED_ENABLED == 1
    _oled.clearMsgArea ();
    _oled.println ("OTA Update:");
    _oled.println ("done");
    _oled.display ();
#endif

    Serial.println ("OTA Update: done.");

    ESP.restart();
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    unsigned int percent = (progress / (total / 100));
    
#if OLED_ENABLED == 1
    _oled.clearMsgArea ();
    _oled.println ("OTA Update:");
    _oled.print ("progress (%): "); _oled.println(percent);
    _oled.display ();
#endif

      Serial.print ("OTA Update: progress (%): "); Serial.println (percent);
  });
  
  ArduinoOTA.onError([](ota_error_t error)
  {
    char * errmsg ;

    switch (error)
    {
      case OTA_AUTH_ERROR:
        errmsg = "auth failed";
        break;

      case OTA_BEGIN_ERROR:
        errmsg = "begin failed";
        break;

      case OTA_CONNECT_ERROR:
        errmsg = "connect failed";
        break;

      case OTA_RECEIVE_ERROR:
        errmsg = "receive error";
        break;

      case OTA_END_ERROR:
        errmsg = "end failed";
        break;

      default:
        errmsg = "unknown error";
        break;
    }
    
#if OLED_ENABLED == 1
    _oled.clearMsgArea ();
    _oled.println ("OTA Error:");
    _oled.println (errmsg);
    _oled.display ();
#endif

    Serial.print ("OTA Error:"); Serial.println (errmsg);
  });
  
  ArduinoOTA.begin();

  return true;
}

// Handle OTA requests.
void _handleOTA ()
{
  ArduinoOTA.handle();
}

// FAIL *********************************************************************

void fail (char * reason)
{
  // Display reason.
#if OLED_ENABLED == 1
  _oled.clearMsgArea ();
  _oled.println ("Fail:");
  _oled.println (reason);
  _oled.display ();
#endif

  Serial.print ("Fail:"); Serial.println (reason);

  // Stop application activities.
  application_fail ();

  // Stop system activites - except OTA update and WiFi, which we leave running
  // to allow repair of the fail.
  _tBlink.disable ();

#if VBAT_ENABLED == 1
  _tBattery.disable ();
#endif

  _tWatchdog.disable ();
  _tMqtt.disable ();

  // Turn the status LED to continuous on to indicate the failure.
  digitalWrite(BOARD_RED_LED_PIN, LED_ON);
}

// TASK SCHEDULER ***********************************************************

Scheduler taskManager;

// ENTRYPOINTS **************************************************************

bool woken = false;

// Called once when the board is reset (i.e., initializes or awakens from
// [deep] sleep).
void setup()
{
  // Set serial monitor baud rate.
  Serial.begin(57600);

  // Where did we come from?
  const rst_info * resetInfo = system_get_rst_info();
  woken = resetInfo->reason == 5;

#if DEBUG_ENABLED == 1
  // Enable debug output.
  Serial.setDebugOutput(true);
#endif

#if OLED_ENABLED == 1
  // Initialize the display.
  _oled_initialize();
#endif

  Serial.println (DEV_NAME);
  Serial.println ("booting...");
  
	// Initialize the task manager.
	taskManager.init();

	// Add the system tasks to the task manager and enable them.

  taskManager.addTask(_tWifi);
  _tWifi.enable();

  taskManager.addTask (_tMqtt);
  // MQTT task is not enabled by setup; it's enabled by the WiFi task once it inits successfully and connects.

  taskManager.addTask (_tWatchdog);
  // Watchdog task is not enabled by setup; it's enabled by the MQTT task once it inits successfully and connects.

#if VBAT_ENABLED == 1
  taskManager.addTask(_tBattery);
  _tBattery.enable();
#endif

  taskManager.addTask(_tBlink);
  _tBlink.enable();

  taskManager.addTask(_tOTA);
  _tOTA.enable();

	// Call application setup code.
	application_setup();
}

// Loops endlessly after setup() finishes and for so long as the board is
// powered and awake.
void loop()
{
	taskManager.execute();
}

// **************************************************************************
// * APPLICATION CODE - INSERT USER CODE BELOW                              *
// **************************************************************************

// Includes - sensor libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>

// Pins -- smart thermostat pin definitions

#define relay4pin 12
#define relay3pin 13
#define relay2pin 14
#define relay1pin 15

#define fanPin relay4pin
#define furnacePin relay3pin
#define acPin relay2pin

#define RELAY_ON LOW
#define RELAY_OFF HIGH

// MQTT topics
#define DATA_FEED     "environment/livingroom/data"
#define STATUS_FEED   "environment/central/status"
#define COMMAND_FEED  "environment/central/command"

// Declare tasks for sensor monitoring, subscription checking.

void environmentScan ();
bool environmentSetup ();

Task tEnvironmentScan (5UL * TASK_SECOND, TASK_FOREVER, &environmentScan, NULL, false, &environmentSetup, NULL);

void subscriptionCheck ();
bool subscriptionSetup ();

Task tSubscription (1UL * TASK_SECOND, TASK_FOREVER, &subscriptionCheck, NULL, false, &subscriptionSetup, NULL);

// Sensors -- environmental sensor declarations

Adafruit_BME280 bme; // I2C
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// MQTT topic for environmental scan reports.
Adafruit_MQTT_Publish envFeed = Adafruit_MQTT_Publish (&mqttClient, DATA_FEED);

// Specific configuration for the TSL2561.
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

// Set up the environment scanning task/ sensors.
bool environmentSetup ()
{
  // Start BME280 temperature/humidity/pressure sensor.
  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    return false;
  }

  // Start TSL2561 luminosity sensor.
  if(!tsl.begin())
  {
    Serial.println("Could not find a valid TSL2561 sensor, check wiring!");
    return false;
  }

  configureLightSensor ();

  return true;
}

void environmentScan ()
{
  // Query sensors and return data.
  float temperature = bme.readTemperature (); // C
  float pressure = bme.readPressure () / 100.0F; // hPa
  float humidity = bme.readHumidity (); // %

  sensors_event_t event;
  tsl.getEvent (&event);

  // Format as JSON.
  // Allocate memory pool for the object tree.
  StaticJsonBuffer<256> jsonBuffer;

  // Create the root of the object tree.
  JsonObject& root = jsonBuffer.createObject();

  // Add values to the object.
  root["light-level"] = event.light;
  root["temperature"] = temperature;
  root["pressure"] = pressure;
  root["humidity"] = humidity;

  char buffer[256];
  root.printTo(buffer, sizeof(buffer));
  Serial.println (buffer);

  if (!envFeed.publish(buffer))
  {
    Serial.println ("envFeed publish failed");
  }
}

// Relays -- relay state management

char relayStatus = 'O';

// All off.
void relayOff ()
{
  Serial.println ("MODE SET: Off");
  
  // turn off all relays
  // do the blower last
  digitalWrite (furnacePin, RELAY_OFF);
  digitalWrite (acPin, RELAY_OFF);

  delay (250);
  
  digitalWrite (fanPin, RELAY_OFF);
}

// Blower only.
void relayBlower ()
{
  Serial.println ("MODE SET: Fan");
  
  // make sure the others are off, then enable the blower
  digitalWrite (furnacePin, RELAY_OFF);
  digitalWrite (acPin, RELAY_OFF);

  digitalWrite (fanPin, RELAY_ON);
}

// Heating.
void relayHeating ()
{
  Serial.println ("MODE SET: Heating");
  
  // disable AC, enable blower, then enable furnace
  digitalWrite (acPin, RELAY_OFF);

  digitalWrite (fanPin, RELAY_ON);
  digitalWrite (furnacePin, RELAY_ON);
}

// Cooling.
void relayCooling ()
{
  Serial.println ("MODE SET: Cooling");
  
  // disable furnace, enable blower, then enable ac
  digitalWrite (furnacePin, RELAY_OFF);

  digitalWrite (fanPin, RELAY_ON);
  digitalWrite (acPin, RELAY_ON);
}

// MQTT subscription for relay management.
Adafruit_MQTT_Subscribe cmdFeed = Adafruit_MQTT_Subscribe (&mqttClient, COMMAND_FEED);
Adafruit_MQTT_Publish statusFeed = Adafruit_MQTT_Publish (&mqttClient, STATUS_FEED);

// Callback function for processing commands.
void cmdCallback (char * command, uint16_t length)
{
  // Data received on command feed.
  // Check length.
  if (length != 1)
    return;

  // If we are asked for a status report, give it.
  if ((char)command[0] == 'S')
  {
    char str[2] = "\0";
    str[0] = relayStatus;

    if (!statusFeed.publish(str))
    {
      Serial.println ("statusFeed publish failed");
    }

    return;
  }

  // Set mode per command.
  relayStatus = (char)command[0];
  switch (relayStatus)
  {
    case 'O':
      relayOff ();
      return;

    case 'F':
      relayBlower ();
      return;

    case 'H':
      relayHeating ();
      return;

    case 'C':
      relayCooling ();
      return;
  }

  Serial.println ("unknown command received, noop");
}

// Set up relay management function.
bool subscriptionSetup ()
{
  cmdFeed.setCallback (cmdCallback);
  mqttClient.subscribe (&cmdFeed);
  
  return true;
}

// Relay management loop.
void subscriptionCheck ()
{
  // Check incoming subscriptions.
  mqttClient.processPackets (100);
}

// Entrypoints -- setup and fail-safe

// Initially set up application tasks (i.e., add them and enable them in the
// task manager) and perform other initialization functions.
void application_setup()
{
  // Set up the relay pins; start all off.
  digitalWrite (relay4pin, RELAY_OFF);
  digitalWrite (relay3pin, RELAY_OFF);
  digitalWrite (relay2pin, RELAY_OFF);
  digitalWrite (relay1pin, RELAY_OFF);
 
  pinMode (relay4pin, OUTPUT);
  pinMode (relay3pin, OUTPUT);
  pinMode (relay2pin, OUTPUT);
  pinMode (relay1pin, OUTPUT);
  
  // Set up the enviroment scanning task.
  taskManager.addTask (tEnvironmentScan);
  tEnvironmentScan.enable();

  // Set up the subscription management task.
  taskManager.addTask (tSubscription);
  tSubscription.enable ();
}

// This function is a callback from `fail ()` (see below) invoked on fatal errors
// to shut down application activities (disabling tasks, etc.) before shutting
// down system activities and otherwise handling failure.
void application_fail ()
{
  // Disable tasks.
  tEnvironmentScan.disable ();
  tSubscription.disable ();

  // Safe relays.
  relayOff ();
}

// This function is invoked immediately before an over-the-air update is installed.
// It should do whatever is necessary to ensure that any devices controlled by this
// Feather are in a safe and shut-down state, such that time taken for, or errors
// during, the update do not pose any risk of harm. Be aware that the OTA update will
// proceed _immediately_ upon return from `application_otasafe()`, and so this
// function cannot rely on any outside tasks; no other code will execute between
// this function and the reset following the update.
void application_otasafe()
{
  // Safe relays.
  relayOff ();
}

