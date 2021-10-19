/**
   Weather Station
   Original idea https://www.instructables.com/Solar-Powered-WiFi-Weather-Station-V30/
   Author dleo.lopez@gmail.com
*/
#define VERSION 0.4
/**
   Libraries
*/
#include "weather.h"
#include "secrets.h"
#include "faov.h"
#include <BME280I2C.h>
#include "Adafruit_SI1145.h"
#include <BH1750.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "Wire.h"
#include <WiFi.h>
#include "esp_deep_sleep.h"
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <EnvironmentCalculations.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>
#include <TimeLib.h>


/**
 * Pin definitions
 */
#define WIND_SPD_PIN 14
#define RAIN_PIN     25
#define WIND_DIR_PIN 35
#define VOLT_PIN     33
#define PV_PIN       32 // ACS712
#define TEMP_PIN     4  // DS18B20 hooked up to GPIO pin 4
#define LED_BUILTIN  2
#define WDT_TIMEOUT 60
#define SEC 1E6          //Multiplier for uS based math

// Variables and constants used in tracking rainfall
#define S_IN_DAY   86400
#define S_IN_HR     3600
#define NO_RAIN_SAMPLES 2000
#define SEALEVELPRESSURE_HPA (1013.25)
#define LATITUDE (7.810944)                           //Latitude for Granja Rosal
#define SEA_LEVEL 1500


/**
 * Objects for 
 */
WiFiUDP ntpUDP;
WiFiClient client;
BME280I2C bme;
Adafruit_SI1145 uv = Adafruit_SI1145();
BH1750 lightMeter;
WiFiClientSecure net;
PubSubClient mqtt(net);
TaskHandle_t TransmitTask;
TaskHandle_t ReadSensorTask;
TaskHandle_t BlinkTask;

/**
 * Externs
 */
extern DallasTemperature temperatureSensor;
extern struct tm timeinfo;
extern const long  gmtOffset_sec;
extern const int   daylightOffset_sec;
extern const char* ntpServer;



/**
 * Variables and constants
 */
struct sensorData
{
  float eto;
  float temperature;
  float pressure;
  float altitude;
  float windSpeed;
  float windGust;
  int windGustDir;
  float dewPoint = 0;
  float heatIndex = 0;
  float outTemperature = 0;
  float windDirection;
  char windCardinalDirection[5];
  float humidity;
  float UVIndex;
  float lux;
  int photoresistor;
  float batteryVoltage;
  int baterry;
  float irradiation;
  int moonPhase;
  char * moonPhaseString;
  int timestamp;
  float rain;
  float rxSignal;
};

//rainfall is stored here for historical data uses RTC
struct historicalData
{
  unsigned int hourlyRainfall[24];
  unsigned int current60MinRainfall[12];
};

// Variables used with RTC Memory storage
RTC_DATA_ATTR volatile int rainTicks = 0;
RTC_DATA_ATTR int lastHour = 0;
RTC_DATA_ATTR time_t nextUpdate;
RTC_DATA_ATTR struct historicalData rainfall;
RTC_DATA_ATTR int bootCount = 0;

// Variables used in calculating the battery voltage
float batteryVolt;
float Vout = 0.00;
float Vin = 0.00;
float R1 = 27000.00; // resistance of R1 (27K) // You can also use 33K
float R2 = 100;
// Variables for solar radiation
float solarRadiation;
float R3 = 100;

//Variables for Solar Radiation
float mVperAmpValue = 185;                  // If using ACS712 current module : for 5A module key in 185, for 20A module key in 100, for 30A module key in 66
float moduleMiddleVoltage = 1650;           // key in middle voltage value in mV. For 5V power supply key in 2500, for 3.3V power supply, key in 1650 mV
float moduleSupplyVoltage = 3300;           // supply voltage to current sensor module in mV, default 5000mV, may use 3300mV
float currentSampleRead  = 0;               // to read the value of a sample
float currentLastSample  = 0;               // to count time for each sample. Technically 1 milli second 1 sample is taken
float currentSampleSum   = 0;               // accumulation of sample readings
float currentSampleCount = 0;               // to count number of sample
float currentMean ;                         // to calculate the average value from all samples
float finalCurrent ;                        // the final current reading without taking offset value
float finalCurrent2 ;                       // the final current reading
float ShortCircuitCurrentSTC = 0.057;       // Key in the Short Circuit Current (At STC condition) of your Solar Panel or Solar Cell. Value 9 showing 9.0A Isc Panel.
/* 1.1 - Offset DC Current */
int   OffsetRead = 0;                   // To switch between functions for auto callibation purpose
float currentOffset =0.00;              // to Offset deviation and accuracy. Offset any fake current when no current operates. 
                                        // the offset will automatically done when you press the <SELECT> button on the LCD display module.
                                        // you may manually set offset here if you do not have LCD shield
float offsetLastSample = 0;             // to count time for each sample. Technically 1 milli second 1 sample is taken
float offsetSampleCount = 0;            // to count number of sample.

/*
  1.2 - Average Accumulate Irradiation
  We need change this for deep sleep  
*/
               
float accumulateIrradiation = 0;                          // Amount of accumulate irradiation
unsigned long startMillisIrradiation;                     // start counting time for irradiation energy
unsigned long currentMillisIrradiation;                   // current counting time for irradiation energy
const unsigned long periodIrradiation = 1000;             // refresh every X seconds (in seconds) Default 1000 = 1 second 
float FinalAccumulateIrradiationValue = 0;                // shows the final accumulate irradiation reading


bool led = true;                                         // Init state of led
bool published = false;                                  // Knows if published successfull
bool lowBattery = false;
long initialMillis = 0;                                  // Initial millis on init


/**
   Setup function
*/
void setup() {
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
    UpdateIntervalModified = 5 * msFactor;    // Seconds 
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
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);
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

    //Start sensor housekeeping
    addTipsToHour(rainTicks);
    clearRainfallHour(timeinfo.tm_hour + 1);
    rainTicks = 0;

    sendData(&environment);
  } 
  wifiOff();
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
  debug("Rainfall last hour: %6.2f \n", rainfall.hourlyRainfall[timeinfo.tm_hour]);
  last24();                                                                                     //Rain Last 24 hours
  debug("Battery Level: %6.2f km/h\n", enviroment->batteryVoltage);
  debug("Temperature in C: %6.2f \n", enviroment->outTemperature);
  debug("Solar Radiation W/M2: %6.2f \n", enviroment->irradiation);
  debug("Dew Point: %6.2f C\n", enviroment->dewPoint);
  debug("Heat Index: %6.2f C\n", enviroment->heatIndex);
  debug("ETo: %6.2f \n", enviroment->eto);
  debug("Moon Phase: %6.2f \n", enviroment->moonPhaseString);
}
