#include "weather.h"

RTC_DATA_ATTR time_t nextUpdate;

/**
 *
 */
void printLocalTime()
{
  if (!getLocalTime(&timeinfo)) {
    debug("Failed to obtain time");
    return;
  }
  Serial.printf("Date:%02i %02i %i Time: %02i:%02i:%02i\n", timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

/**
 * Diagnostic routine to print next wake time
 */
void printTimeNextWake()
{
  getLocalTime(&timeinfo);
  Serial.printf("Time to next wake: %i seconds\n", nextUpdate - mktime(&timeinfo) );
}

/**
 *
 */
long updateWake (void)
{
  long UpdateIntervalModified = 0;
  // TODO: Adjust time based on battery
  int muliplierBatterySave = 1;
  if (lowBattery)
  {
    muliplierBatterySave = 4;
  }
  getLocalTime(&timeinfo);
  int minutesToQuater = (14 - (timeinfo.tm_min %  15));                   // We used 14 for wake up 1 minute before
  if (minutesToQuater<=0) {
    minutesToQuater = 15;                                                  // Wake up in the next minute by default
  }

  nextUpdate = mktime(&timeinfo) + (minutesToQuater * 60);
  /*
  // Intentional offset for data aquire before display unit updates
  // guarantees fresh data
  if (nextUpdate > 120)
  {
    nextUpdate -= 60;
  }
  */

  UpdateIntervalModified = nextUpdate - mktime(&timeinfo);
  if (UpdateIntervalModified <= 0)
  {
    UpdateIntervalModified = 15;
  }
  printTimeNextWake();

  return UpdateIntervalModified * SEC;
}
