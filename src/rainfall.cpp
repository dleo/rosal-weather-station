#include "weather.h"

// Variables used in software delay to supress spurious counts on rain_tip
volatile unsigned long timeSinceLastTip = 0;
volatile unsigned long lastTip = 0;
volatile unsigned long tickRainTime[20] = {0};
volatile int countRain = 0;

/**
 * Zero out rainfall counter structure
 */
void clearRainfall(void)
{
  memset(&rainfall, 0x00, sizeof(rainfall));
}


/*
 * Increment current hour tip count
 */
void addTipsToHour(int count)
{
  int hourPtr = timeinfo.tm_hour;
  rainfall.hourlyRainfall[hourPtr] = rainfall.hourlyRainfall[hourPtr] + count;
}

/**
 * @brief Get the rain by specify hour
 * 
 * @param hour 
 * @return float 
 */
float getRainByHour(int hour)
{
  float rain = rainfall.hourlyRainfall[hour % 24] * RAIN_TICK;
  debug("The rain for hour %d is %i", hour, rain);
  return rain;
}

/**
 * Diagnostic routine to print hourly rainfall array to terminal
 */
void printHourlyArray (void)
{
  int hourCount = 0;
  for (hourCount = 0; hourCount < 24; hourCount++)
  {
    debug("Hour %i: %u\n", (hourCount+1), rainfall.hourlyRainfall[hourCount]);
  }
}

/**
 * Return tip counter for last 24h (technically 23h)
 */
int last24(void)
{
  int hour;
  int totalRainfall = 0;
  for (hour = 0; hour < 24; hour++)
  {
    totalRainfall += rainfall.hourlyRainfall[hour];
  }
  debug("Total rainfall: %i\n", totalRainfall);
  return totalRainfall;
}

/**
 * ISR for rain tip gauge count
 */
void IRAM_ATTR rainTick(void)
{
  timeSinceLastTip = millis() - lastTip;
  //software debounce attempt
  if (timeSinceLastTip > 400)
  {
    debug("Rain dump\n");
    rainTicks++;
    lastTip = millis();
    //Here we can calculate rain rate
    //Even must be some calculations for know if a storm is present
    tickRainTime[countRain] = timeSinceLastTip;
    countRain++;
    if (countRain >= 10) {
      countRain = 0;
    }
  }
}