/**
   Weather Station
   Original idea https://www.instructables.com/Solar-Powered-WiFi-Weather-Station-V30/
   Author dleo.lopez@gmail.com
*/
#include <Arduino.h>


#define VERSION "0.4.0"
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
RTC_DATA_ATTR int currentHour = 0;
RTC_DATA_ATTR int currentDay = 0;
float currentRain = 0;
RTC_DATA_ATTR int bootCount = 0;
bool published = false;
long initialMillis = 0;
bool lowBattery = false;
const long  gmtOffset_sec = -4 * 3600;
const int   daylightOffset_sec = 3600;
const char* ntpServer = "pool.ntp.org";
int dayOfYear = 1;
bool isNewDay = false;
bool isNewHour = false;

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
  float currentRain = 0;                              // At least we should know if is rainying
  // Setup watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               //add current thread to WDT watch

  // Setup Pin Mode
  pinMode(LED_BUILTIN, OUTPUT);                           // Led alive
  pinMode(WIND_DIR_PIN, INPUT);                           // Wind dir sensor
  pinMode(WIND_SPD_PIN, INPUT);                           // Wind speed sensor
  pinMode(RAIN_PIN, INPUT);                               // Rain sensor
  pinMode(SOLAR_RADIATION, INPUT);                        // Solar Radiation sensor
  
  Serial.begin(115200);
  debug("Weather station powered on.\n");
  published = false;
  initialMillis = millis();
  debug("Boot #%d.\n", bootCount);
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
  //pet the dog
  esp_task_wdt_reset();                       // Pet the dog!
}

/**
 * @brief 
 * 
 */
void goToSleep(){
  long UpdateIntervalModified = 0;
  UpdateIntervalModified = nextUpdate - mktime(&timeinfo);
  sleep(UpdateIntervalModified);                            // Go to sleep
  if (UpdateIntervalModified <= 0)
  {
    UpdateIntervalModified = 60 * SEC;    // Seconds 
  }
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
      goToSleep();
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
    //processSensorUpdates();
    // We create a thread for read data from sensors
    xTaskCreatePinnedToCore(
                      handle,                         // Task function.
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
void processSensorUpdates(struct sensorData *environment, boolean proccessRain = true)
{
  wifiOn();
  if (WiFi.status() == WL_CONNECTED) {
    //Calibrate Clock - My ESP RTC is noticibly fast
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    getLocalTime(&timeinfo);
    dayOfYear = calculateDayOfYear(day(), month(), year());
    printLocalTime();
    printTimeNextWake();
    // Check if is the first boot, we init curretn hour and day
    if (bootCount <= 1) {
      currentHour = timeinfo.tm_hour;
      currentDay = timeinfo.tm_mday;
    }
    // Check if we're on new hour
    if (currentHour != timeinfo.tm_hour) {
      isNewHour = true; 
      currentHour = timeinfo.tm_hour;
    }
    // Check if we are on new day, we must update
    if (currentDay != timeinfo.tm_mday) {
      isNewDay = true;
      currentDay = timeinfo.tm_mday;
    }
    // Rain process must be with time available
    if (proccessRain) {
      //move rainTicks into hourly containers
      debug("Moving rain ticks...Current Hour: %i\n\n", timeinfo.tm_hour);
      if (isNewHour) {
        currentRain = rainTicks;
      }
      addTipsToHour(rainTicks);
      rainTicks = 0;
      // Check if we are on new day, we must update
      if (isNewDay) {
        clearRainfall();
      }
    }
  }
  //Get Sensor data
  readSensorsData(environment);
  // Print data
  printData(environment);
    
}

/**
 * Debug sensor info
 */
void handle(void *paramsValue) {
  struct sensorData environment;
  while(true) {
    esp_task_wdt_reset();                                   //Pet the dog       
    if ((millis() % 1000) == 0) {
      processSensorUpdates(&environment);
    }
    // Try to sent data
    if ((millis() - initialMillis) >= (10 * msFactor)) {
      if (sendData(&environment)) {
        sleep(updateWake());
      }
    }
    // We will check if has been more than 60 seconds up
    if ((millis() - initialMillis) >= (60 * msFactor))
    {
      sleep(updateWake());
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
  debug("Rainfall last hour: %6.2f \n", currentRain);
  last24();                                                                                     //Rain Last 24 hours
  debug("Solar Radiation W/M2: %6.2f \n", enviroment->irradiation);
  debug("Dew Point: %6.2f C\n", enviroment->dewPoint);
  debug("Heat Index: %6.2f C\n", enviroment->heatIndex);
  debug("ETo: %6.2f \n", enviroment->eto);
  debug("Moon Phase: %6.2f \n", enviroment->moonPhaseString);
  debug("Batery: %6.2f \n", enviroment->batteryVoltage);
  debug("Rx Signa: %d \n", enviroment->rxSignal);
}
