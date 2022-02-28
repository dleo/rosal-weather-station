#ifndef DEFINES_H
#define DEFINES_H
#include "Arduino.h"
#include "DallasTemperature.h"
#include "PubSubClient.h"
#include "Adafruit_SI1145.h"
#include "BME280I2C.h"
#include "BH1750.h"
#include <WiFiClientSecure.h>
/**
 * Pin definitions
 */
#define WIND_SPD_PIN 13   //For version 38 pins, 13. Other 14
#define RAIN_PIN     25   
#define WIND_DIR_PIN 35   
#define VOLT_PIN     33
#define TEMP_PIN     4    // DS18B20 hooked up to GPIO pin 4
#define LED_BUILTIN  2
#define WDT_TIMEOUT 60
#define SEC 1E6           //Multiplier for uS based math
#define S_IN_DAY   86400
#define S_IN_HR     3600
#define NO_RAIN_SAMPLES 2000
#define SEALEVELPRESSURE_HPA (1013.25)
#define LATITUDE (7.810944)                           //Latitude for Granja Rosal
#define SEA_LEVEL 1500
#define SOLAR_RADIATION 36  
#define REF_3V3 39
#define RAIN_TICK (0.011)


/**
 * Struct definitions
 */
struct sensorData
{
  float eto;
  float temperature;
  float pressure;
  float altitude;
  float windSpeed;                          //km/h
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
  float rainLastHour;
  float rainLastDay;
  float rxSignal;
  float etoDaily;
};
//rainfall is stored here for historical data uses RTC
struct historicalData
{
  unsigned int hourlyRainfall[24];
  unsigned int current60MinRainfall[12];
};

// Variables used with RTC Memory storages
extern RTC_DATA_ATTR volatile int rainTicks;
extern RTC_DATA_ATTR int currentHour;
extern RTC_DATA_ATTR int currentDay;
extern RTC_DATA_ATTR time_t nextUpdate;
extern RTC_DATA_ATTR struct historicalData rainfall;
extern RTC_DATA_ATTR int bootCount;
extern DallasTemperature temperatureSensor;
extern bool published;
extern bool lowBattery;
extern long initialMillis;
extern float currentRain;
extern struct tm timeinfo;
extern PubSubClient mqtt;
extern Adafruit_SI1145 uv;
extern BME280I2C bme;
extern BH1750 lightMeter;
extern WiFiClientSecure net;
extern int dayOfYear;
extern bool isNewDay;
extern bool isNewHour;


/**
 * Function declaration
 */
void wakeupReason();
void sleep(long sleepMilis);
void initCoreTasks();
void switchLed(void *paramsValue);
void processSensorUpdates();
void handle(void *paramsValue);
void printData(struct sensorData *enviroment);
bool wifiOn();
void wifiOff();
void IRAM_ATTR rainTick(void);
void debug( const char* format, ... );
void IRAM_ATTR windTick(void);
void printLocalTime();
void printTimeNextWake();
void readSensorsData(struct sensorData *environment);
void readMoonPhase(struct sensorData *environment);
void readEto(struct sensorData *environment);
void readBattery (struct sensorData *environment);
void readUvIndex(struct sensorData *environment);
void readBME(struct sensorData *environment);
void readLux(struct sensorData *environment);
void readTemperature (struct sensorData *environment);
void readIrradiation(struct sensorData *environment);
void readRain(struct sensorData *environment);
void sendData(struct sensorData enviroment);
int moonPhases(unsigned long time);
char * moonPhaseToString(int phase);
void connectToMqtt();
void reconnect();
void pubSubErr(int8_t MQTTErr);
boolean sendData(struct sensorData *enviroment);
void connectAWS();
void callback(char* topic, byte* payload, unsigned int length);
void clearRainfall(void);
void clearRainfallHour(int hourPtr);
void addTipsToHour(int count);
void printHourlyArray (void);
int getRainByHour(int hour);
int last24(void);
void readSensorsData(struct sensorData *environment);
void readMoonPhase(struct sensorData *environment);
void readEto(struct sensorData *environment);
void readBattery (struct sensorData *environment);
void readUvIndex(struct sensorData *environment);
void readBME(struct sensorData *enviroment);
void readLux(struct sensorData *environment);
void readTemperature(struct sensorData *environment);
void readIrradiation(struct sensorData *environment);
void readRxSignal(struct sensorData *environment);
void printLocalTime();
void printTimeNextWake();
long updateWake (void);
double properAng(double big);
double julianDate(unsigned long time);
int moonPhases(unsigned long time);
char * moonPhaseToString(int phase);
void calculateEto(struct sensorData *environment, boolean hourly = false);
void debug( const char* format, ... );
void readWindSpeed(struct sensorData *environment );
void readWindDirection(struct sensorData *environment);
void IRAM_ATTR windTick(void);
int calculateDayOfYear(int day, int month, int year);

#endif




