#include "weather.h"

OneWire oneWire(TEMP_PIN);
DallasTemperature temperatureSensor(&oneWire);

/**
 * Read sensors
 */
void readSensorsData(struct sensorData *environment) {
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
}

/**
 * Read moonphase
 */
void readMoonPhase(struct sensorData *environment) {
    // Calculate moon phase
    if (WiFi.status() == WL_CONNECTED) {
      environment->moonPhase = moonPhases(now());
      environment->moonPhaseString = moonPhaseToString(environment->moonPhase);
    }
}

/**
 * Read eto
 */
void readEto(struct sensorData *environment) {
    environment->eto = calculateEto(environment);
}

/**
 * Read analog volatage divider value
 */
void readBattery (struct sensorData *environment) {
    // Reading Battery Level in %
    float val = analogRead(VOLT_PIN);                    //reads the analog input
    float Vout = (val * 3.3 ) / 4095.0;                  // formula for calculating voltage out
    environment->batteryVoltage = Vout * ( R2 + R1) / R2 ;  // formula for calculating voltage in
}

/**
 * Read from GY1145 sensor read
 */
void readUvIndex(struct sensorData *environment) {
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
    EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
    EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;
    enviroment->altitude = EnvironmentCalculations::Altitude(enviroment->pressure, envAltUnit, SEALEVELPRESSURE_HPA, envTempUnit);
    enviroment->dewPoint = EnvironmentCalculations::DewPoint(enviroment->temperature, enviroment->humidity, envTempUnit);
    enviroment->heatIndex = EnvironmentCalculations::HeatIndex(enviroment->temperature, enviroment->humidity, envTempUnit);
}

/**
 * Read from LUX sensor read
 */
void readLux(struct sensorData *environment) {
  environment->lux = lightMeter.readLightLevel();
}

/**
 * Read from 1W DS1820B
 */
void readTemperature(struct sensorData *environment) {
  temperatureSensor.requestTemperatures();
  environment->outTemperature = temperatureSensor.getTempCByIndex(0);

  // Check if reading was successful
  if (environment->outTemperature == DEVICE_DISCONNECTED_C)
  {
    environment->outTemperature = -40;
  }
}

/**
 * Calculate solar irradiation from ACS712
 */
void readIrradiation(struct sensorData *environment) {
    
  /* 1- DC Current & Irradiation */
  
  if(millis() >= currentLastSample + 1 )                                                                          /* every 100 milli second taking 1 reading */
  {
    currentSampleRead = analogRead(PV_PIN)-((moduleMiddleVoltage/moduleSupplyVoltage)*1024);                      /* read the sample value */ 
    currentSampleSum += currentSampleRead ;                                                                       /* accumulate value with older sample readings*/  
    currentSampleCount++;                                                                                         /* to move on to the next following count */
    currentLastSample = millis();                                                                                 /* to reset the time again so that next cycle can start again*/ 
  }

  if(currentSampleCount == 10)                                                                                     /* after 10 count or 1000 milli seconds (1 second), do the calculation and display value*/
  {
    currentMean = currentSampleSum/currentSampleCount;                                                            /* calculate average value of all sample readings taken*/
    finalCurrent = (((currentMean /1024)*moduleSupplyVoltage)/mVperAmpValue);                                     /* calculate the final current (without offset)*/
    finalCurrent2 = finalCurrent + currentOffset;                                                                 /* The final current */
    environment->irradiation = (finalCurrent2/(ShortCircuitCurrentSTC*10));                                        /* Save the current irradiation */
    currentSampleSum = 0;                                                                                         /* to reset accumulate sample values for the next cycle */
    currentSampleCount= 0;                                                                                        /* to reset number of sample for the next cycle */
  }    

  /* 1.2 - Average Accumulate Irradiation */

  currentMillisIrradiation = millis();                                                                 /* Count the time for current */

  if (currentMillisIrradiation - startMillisIrradiation >= periodIrradiation)
  {
    accumulateIrradiation = environment->irradiation/3600*(periodIrradiation/1000);                                /* for smoothing calculation*/
    FinalAccumulateIrradiationValue =  FinalAccumulateIrradiationValue + accumulateIrradiation ;
    startMillisIrradiation = currentMillisIrradiation ;                                               /* Set the starting point again for next counting time */
  }
}

/**
 * Read Rx WiFi Signal
 */
void readRxSignal(struct sensorData *environment) {
  if (WiFi.status() == WL_CONNECTED) {
    environment->rxSignal = WiFi.RSSI();
  }
}

