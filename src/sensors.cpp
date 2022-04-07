#include "weather.h"
#include "faov.h"
#include "Arduino.h"
#include "WiFi.h"
#include "TimeLib.h"
#include "secrets.h"
#include "BME280I2C.h"
#include "EnvironmentCalculations.h"

/**
 * Read sensors
 */
void readSensorsData(struct sensorData *environment)
{
  readWindSpeed(environment);
  readWindDirection(environment);
  readTemperature(environment);
  readLux(environment);
  readBME(environment);
  readUvIndex(environment);
  readBattery(environment);
  readIrradiation(environment);
  readEto(environment);
  readMoonPhase(environment);
  readRxSignal(environment);
  readRain(environment);
}

/**
 * Read moonphase
 * @author David Lopez <dleo.lopez@gmail.com>
 */
void readMoonPhase(struct sensorData *environment)
{
  // Calculate moon phase
  if (WiFi.status() == WL_CONNECTED)
  {
    environment->moonPhase = moonPhases(now());
    environment->moonPhaseString = moonPhaseToString(environment->moonPhase);
  }
}

/**
 * @brief Read eto
 * 
 * 
 */
void readEto(struct sensorData *environment)
{
  // If is new hour, we calculate hour eto
  calculateEto(environment, isNewHour);
}

/**
 * Read analog volatage divider value
 */
void readBattery(struct sensorData *environment)
{
  // Reading Battery Level in %
  float val = analogRead(VOLT_PIN); // reads the analog input
  debug("Voltaje Batery: %6.2f \n", val);
  float Vout = val * batteryCalFactor;                       // formula for calculating voltage out
  environment->batteryVoltage = ((Vout / 4.2) * 100) * 0.95; // formula for calculating voltage including error
}

/**
 * Read from GY1145 sensor read
 */
void readUvIndex(struct sensorData *environment)
{
  // Reading GY1145 UV sensor
  environment->UVIndex = uv.readUV() / 100;
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
}

/**
 * Read from BME sensor read
 */
void readBME(struct sensorData *enviroment)
{
  bme.read(enviroment->pressure, enviroment->temperature, enviroment->humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
  EnvironmentCalculations::AltitudeUnit envAltUnit = EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit envTempUnit = EnvironmentCalculations::TempUnit_Celsius;
  enviroment->altitude = EnvironmentCalculations::Altitude(enviroment->pressure, envAltUnit, SEALEVELPRESSURE_HPA, envTempUnit);
  enviroment->dewPoint = EnvironmentCalculations::DewPoint(enviroment->temperature, enviroment->humidity, envTempUnit);
  enviroment->heatIndex = EnvironmentCalculations::HeatIndex(enviroment->temperature, enviroment->humidity, envTempUnit);
}

/**
 * Read from LUX sensor read
 */
void readLux(struct sensorData *environment)
{
  environment->lux = lightMeter.readLightLevel();
}

/**
 * Read from 1W DS1820B
 */
void readTemperature(struct sensorData *environment)
{
  temperatureSensor.requestTemperatures();
  environment->outTemperature = temperatureSensor.getTempCByIndex(0);

  // Check if reading was successful
  if (environment->outTemperature == DEVICE_DISCONNECTED_C)
  {
    environment->outTemperature = -40;
  }
}

/**
 * @brief Read from solar radiation sensor
 */
void readIrradiation(struct sensorData *environment)
{
  int radSolar = analogRead(SOLAR_RADIATION);
  int ref = analogRead(REF_3V3);
  debug("Solar radiation read %d \n", radSolar);
  debug("Ref radiation read %d \n", ref);
  float mV = (radSolar / ref) * 3300;
  environment->irradiation = (int)mV / 1.67;
}

/**
 * Read Rx WiFi Signal
 */
void readRxSignal(struct sensorData *environment)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    environment->rxSignal = WiFi.RSSI();
  }
}

/**
 * @brief Read the rain
 *
 * @param environment
 */
void readRain(struct sensorData *environment)
{
  environment->rain = getRainByHour(currentHour);
  int diff = diffHour(currentHour, 1);
  environment->rainLastHour = getRainByHour(diff);
  environment->rainLastDay = last24() * RAIN_TICK;
}