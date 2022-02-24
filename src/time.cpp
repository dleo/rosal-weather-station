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

/**
 * @brief Calculate day of year
 * 
 * @link https://gist.github.com/jrleeman/3b7c10712112e49d8607
 * @param day 
 * @param month 
 * @param year 
 * @return int 
 */
int calculateDayOfYear(int day, int month, int year) {
  
  // Given a day, month, and year (4 digit), returns 
  // the day of year. Errors return -1.
  
  int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  
  // Verify we got a 4-digit year
  if (year < 1000) {
    return -1;
  }
  
  // Check if it is a leap year, this is confusing business
  // See: https://support.microsoft.com/en-us/kb/214019
  if (year%4  == 0) {
    if (year%100 != 0) {
      daysInMonth[1] = 29;
    }
    else {
      if (year%400 == 0) {
        daysInMonth[1] = 29;
      }
    }
   }

  // Make sure we are on a valid day of the month
  if (day < 1) 
  {
    return -1;
  } else if (day > daysInMonth[month-1]) {
    return -1;
  }
  
  int doy = 0;
  for (int i = 0; i < month - 1; i++) {
    doy += daysInMonth[i];
  }
  
  doy += day;
  return doy;
}
