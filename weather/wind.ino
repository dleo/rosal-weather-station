#include "weather.h";

volatile unsigned long timeSinceLastTick = 0;
volatile unsigned long lastTick = 0;
volatile unsigned long tickTime[20] = {0};
volatile int count = 0;

/**
 * Read wind speed
 */
void readWindSpeed(struct sensorData *environment )
{
  float windSpeed = 0;
  int position;
  long msTotal = 0;
  int samples = 0;

  //intentionally ignore the zeroth element
  //look at up to 3 (or 6) revolutions to get wind speed
  //Again, I see 2 ticks on anemometer
  if (count)
  {
    for (position = 1; position < 7; position++)
    {
      if (tickTime[position])
      {
        msTotal += tickTime[position];
        samples ++;
      }
    }
  }
  //Average samples
  if (msTotal > 0 && samples > 0)
  {
    windSpeed = 1.49 * 1000 / (msTotal / samples);      // Get speed on mph
  }
  else
  {
    debug("No Wind data");
    windSpeed = 0;
  }
  // TODO Logic for metric or US
  windSpeed =  windSpeed * 1.60934;
  environment->windSpeed = windSpeed;
}

/**
 * Read ADC to find wind direction
 */
void readWindDirection(struct sensorData *environment)
{
  int windPosition;
  //Initial direction
  //Prove it is not this direction
  String windDirection = "0";
  String windCardinalDirection = "N";
  int analogCompare[15] = {150, 300, 450, 600, 830, 1100, 1500, 1700, 2250, 2350, 2700, 3000, 3200, 3400, 3900};
  String windDirText[15] = {"157.5", "180", "247.5", "202.5", "225", "270", "292.5", "112.5", "135", "337.5", "315", "67.5", "90", "22.5", "45"};
  String windDirCardinalText[15] = {"SSE", "S", "WSW", "SSW", "SW", "W", "WNW", "ESE", "SE", "NNW", "NW", "ENE", "E", "NNE", "NE"};
  char buffer[10];
  int vin = analogRead(WIND_DIR_PIN);

  for (windPosition = 0; windPosition < 15; windPosition++)
  {
    if (vin < analogCompare[windPosition])
    {
      windDirection = windDirText[windPosition];
      windCardinalDirection = windDirCardinalText[windPosition];
      break;
    }
  }
  windDirection.toCharArray(buffer, 5);
  environment->windDirection = atof(buffer);
  strcpy(environment->windCardinalDirection, windCardinalDirection.c_str());
}

//=======================================================
//  windTick: ISR to capture wind speed relay closure
//=======================================================
void IRAM_ATTR windTick(void)
{
  timeSinceLastTick = millis() - lastTick;
  //software debounce attempt
  //record up to 10 ticks from anemometer
  if (timeSinceLastTick > 10 && count < 10)
  {
    lastTick = millis();
    tickTime[count] = timeSinceLastTick;
    count++;
  }
}
