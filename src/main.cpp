/**
   Weather Station
   Original idea https://www.instructables.com/Solar-Powered-WiFi-Weather-Station-V30/
   Author dleo.lopez@gmail.com
*/
#include <Arduino.h>


#define VERSION 0.4
/**
   Libraries
*/
#include "weather.h"
#include "secrets.h"
#include "faov.h"
#include "OneWire.h"
#include "Wire.h"
#include "WiFi.h"
#include "esp_deep_sleep.h"
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include <EnvironmentCalculations.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <TimeLib.h>



/**
 * Objects for 
 */
WiFiUDP ntpUDP;
WiFiClient client;
WiFiClientSecure net;
PubSubClient mqtt(net);
TaskHandle_t TransmitTask;
TaskHandle_t ReadSensorTask;
TaskHandle_t BlinkTask;

/**
 * Variables and constants
 */

// Variables used in calculating the battery voltage
float batteryVolt;
float Vout = 0.00;
float Vin = 0.00;
float R1 = 27000.00; // resistance of R1 (27K) // You can also use 33K
float R2 = 100;


bool led = true;                                         // Init state of led
struct tm timeinfo;
RTC_DATA_ATTR struct historicalData rainfall;
RTC_DATA_ATTR volatile int rainTicks = 0;
RTC_DATA_ATTR int lastHour = 0;
RTC_DATA_ATTR int bootCount = 0;
bool published = false;
long initialMillis = 0;
bool lowBattery = false;
const long  gmtOffset_sec = -4 * 3600;
const int   daylightOffset_sec = 3600;
const char* ntpServer = "pool.ntp.org";

OneWire oneWire(TEMP_PIN);
DallasTemperature temperatureSensor(&oneWire);
Adafruit_SI1145 uv = Adafruit_SI1145();
BH1750 lightMeter;
BME280I2C bme;

/**
   Setup function
*/
void setup() {
  lowBattery = false;                                 // Knows if baterry is low
  published = false;                                  // Knows if published successfull
  initialMillis = 0;                                  // Initial millis on init
  long UpdateIntervalModified = 0;
  // Setup watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  // Setup Pin Mode
  pinMode(LED_BUILTIN, OUTPUT);                           // Led alive
  pinMode(WIND_DIR_PIN, INPUT);                           // Wind dir sensor
  pinMode(WIND_SPD_PIN, INPUT);                           // Wind speed sensor
  pinMode(RAIN_PIN, INPUT);                               // Rain sensor
  
  Serial.begin(115200);
  debug("Weather station powered on.");
  published = false;
  initialMillis = millis();
  bootCount++;

  // Begin setup sensors
  Wire.begin();
  temperatureSensor.begin();
  while(!bme.begin())
  {
    debug("Could not find BME280 sensor!");
  }
  uv.begin(0x60);                                         // 0x60 is the address of the GY1145 module*/
  lightMeter.begin();

  // Power off WiFi for saving power
  wifiOff();
  // Identified reason for wakeup
  wakeupReason();
  // Check if we need publish
  // Use parallel for led blink and know if it's alive
  /*
  xTaskCreatePinnedToCore(
                    switchLed,                // Task function.
                    "blinkTask",              // name of task
                    10000,                    // Stack size of task
                    NULL,                     // parameter of the task
                    1,                        // priority of the task 
                    &BlinkTask,               // Task handle to keep track of created task
                    1                         // pin task to core 1
  );
  */
  UpdateIntervalModified = nextUpdate - mktime(&timeinfo);
  
  if (UpdateIntervalModified <= 0)
  {
    UpdateIntervalModified = 60 * SEC;    // Seconds 
  }
  //pet the dog
  esp_task_wdt_reset();                       // Pet the dog!
  sleep(UpdateIntervalModified);
}


/**
//===========================================================
// wakeupReason: action based on WAKE reason
// 1. Power up
// 2. WAKE on EXT0 - increment rain tip gauge count and sleep
// 3. WAKE on TIMER - send sensor data to IOT target
//===========================================================
//check for WAKE reason and respond accordingly
 */
void wakeupReason() {
  digitalWrite(LED_BUILTIN, HIGH);
  esp_sleep_wakeup_cause_t wakeupReason;

  wakeupReason = esp_sleep_get_wakeup_cause();
  debug("Wakeup reason: %d\n", wakeupReason);
  switch (wakeupReason)
  {
    //Rain Tip Gauge
    case ESP_SLEEP_WAKEUP_EXT0 :
      debug("Wakeup caused by external signal using RTC_IO\n");
      published = true;
      rainTicks++;
      printHourlyArray();
      break;

    //Timer
    case ESP_SLEEP_WAKEUP_TIMER :
      debug("Wakeup caused by timer\n");
      published = false;
      initCoreTasks();
      break;
    //Initial boot or other default reason
    default :
      debug("Wakeup was not caused by deep sleep: %d\n", wakeupReason);
      published = false;
      initCoreTasks();
      break;
  }
}

/**
 * Init core tasks
 */
void initCoreTasks() {
      //Rainfall interrupt pin set up
    delay(100); //possible settling time on pin to charge
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainTick, FALLING);
    attachInterrupt(digitalPinToInterrupt(WIND_SPD_PIN), windTick, RISING);
    processSensorUpdates();
    // We create a thread for read data from sensors
    /*
    xTaskCreatePinnedToCore(
                      processSensorUpdates,           // Task function.
                      "readSensorsData",              // name of task
                      10000,                          // Stack size of task
                      NULL,                           // parameter of the task
                      1,                              // priority of the task 
                      &ReadSensorTask,                // Task handle to keep track of created task
                      1                               // pin task to core 1
    );
    */
    xTaskCreatePinnedToCore(
                      debugSensor,                    // Task function.
                      "readSensorsData",              // name of task
                      10000,                          // Stack size of task
                      NULL,                           // parameter of the task
                      1,                              // priority of the task 
                      &ReadSensorTask,                // Task handle to keep track of created task
                      1                               // pin task to core 1
    );
}


/**
 * Loop function
 */
void loop() {
  //Nothing here...
}

/**
 * Sleep
 */
void sleep(long sleepMilis) {
  digitalWrite(LED_BUILTIN, LOW);
  wifiOff();
  // ESP32 Deep SLeep Mode
  esp_deep_sleep_enable_timer_wakeup(sleepMilis);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 0);
  debug("Going to sleep now...");
  esp_deep_sleep_start();
}

/**
 * Change Led Status
 */
void switchLed(void *paramsValue) {
  
  while(true) {
    if ((millis() % 5000) == 0) {
      int signal = led ? LOW : HIGH;
      digitalWrite(LED_BUILTIN, signal);
      led = !led;
    }
  }
}

/**
 * Update clock and read sensors
 */
void processSensorUpdates()
{
  struct sensorData environment;
  if (wifiOn()) {
    //Calibrate Clock - My ESP RTC is noticibly fast
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
    printTimeNextWake();
    //Get Sensor data
    readSensorsData(&environment);
    // Print data
    printData(&environment);
    //move rainTicks into hourly containers
    debug("Current Hour: %i\n\n", timeinfo.tm_hour);
    addTipsToHour(rainTicks);
    clearRainfallHour(timeinfo.tm_hour + 1);
    rainTicks = 0;
    // Send data to mqtt
    sendData(&environment);
  } 
  wifiOff();
}

/**
 * Debug sensor info
 */
void debugSensor(void *paramsValue) {
  while(true) {
    if ((millis() % 5000) == 0) {
      processSensorUpdates();
    }
  }
}

/**
 * Print data
 */
void printData(struct sensorData *enviroment)
{
  debug("Altitude: %i\n", enviroment->altitude);
  debug("Air temperature [°C]: %6.2f\n", enviroment->temperature);
  debug("Temperature [°C]: %6.2f\n", enviroment->outTemperature);
  debug("Humidity [%]: %6.2f\n", enviroment->humidity);
  debug("Barometric pressure [hPa]: %6.2f\n", enviroment->pressure);
  debug("UV: %6.2f\n", enviroment->UVIndex);
  debug("Light: %6.2f\n", enviroment->lux);
  debug("Wind Dir: %s\n", enviroment->windCardinalDirection);
  debug("Windspeed: %6.2f km/h\n", enviroment->windSpeed);
  /** TODO
  debug("Wind Gust: %s\n", enviroment->windCardinalDirection);                
  debug("Wind Gust Dir: %6.2f km/h\n", enviroment->windSpeed);
  */
  debug("Rainfall last hour: %6.2f \n", rainfall.hourlyRainfall[timeinfo.tm_hour-1]);
  last24();                                                                                     //Rain Last 24 hours
  debug("Battery Level: %6.2f km/h\n", enviroment->batteryVoltage);
  debug("Temperature in C: %6.2f \n", enviroment->outTemperature);
  debug("Solar Radiation W/M2: %6.2f \n", enviroment->irradiation);
  debug("Dew Point: %6.2f C\n", enviroment->dewPoint);
  debug("Heat Index: %6.2f C\n", enviroment->heatIndex);
  debug("ETo: %6.2f \n", enviroment->eto);
  debug("Moon Phase: %6.2f \n", enviroment->moonPhaseString);
  debug("Batery: %6.2f \n", enviroment->batteryVoltage);
}
