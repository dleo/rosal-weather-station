#include "weather.h"
/**
 * Internal function calculate proper angle
 */
double properAng(double big)
{
  double tmp = 0;
  if (big > 0)
  {
    tmp = big / 360.0;
    tmp = (tmp - floor(tmp)) * 360.0;
  }
  else
  {
    tmp = ceil(abs(big / 360.0));
    tmp = big + tmp * 360.0;
  }

  return tmp;
}

/**
 * Calculate julian date
 */
double julianDate(unsigned long time) 
{
  return ( time / 86400.0 ) + 2440587.5;
}

/**
 * Calculate month phase
 * 0 -> Full moon
 * 1 -> Waning Gibbous
 * 2 -> Last Quarter
 * 3 -> Old Crescent
 * 4 -> New Moon
 * 5 -> New Crescent
 * 6 -> First Quarter
 * 7 -> Waxing Gibbous
 */
int moonPhases(unsigned long time)
{
  // calculates the age of the moon phase(0 to 7)
  // there are eight stages, 0 is full moon and 4 is a new moon
  double jd = 0;                // Julian Date
  double ed = 0;                //days elapsed since start of full moon
  int b= 0;
  jd = julianDate(time);
  jd = int(jd - 2244116.75);    // start at Jan 1 1972
  jd /= 29.53;                  // divide by the moon cycle
  b = jd;
  jd -= b;                      // leaves the fractional part of jd
  ed = jd * 29.53;              // days elapsed this month
  b = jd*8 +0.5;
  b = b & 7;
  return b;
}

/**
 * Moon phase on string
 */
char * moonPhaseToString(int phase)
{
  switch (phase) {
    case 0:
      return "Full moon";
      break;
    case 1:
      return "Waning Gibbous";
      break;
    case 2:
      return "Last Quarter";
      break;
    case 3:
      return "Old Crescent";
      break;
    case 4:
      return "New Moon";
      break;
    case 5:
      return "New Crescent";
      break;
    case 6:
      return "First Quarter";
      break;
    case 7:
      return "Waxing Gibbous";
      break;
  }
}

/**
 * Calculate eto
 */
float calculateEto(struct sensorData *environment) {
  float svp = svp_from_t(environment->humidity);
  float avp = avp_from_tdew(environment->dewPoint);
  float dsvp = delta_svp(environment->temperature);
  float psy = psy_const(environment->pressure);
  return fao56_penman_monteith(environment->irradiation, environment->temperature, environment->windSpeed * 2.4, svp, environment->humidity, avp, dsvp, psy);
}

/**
 * Diagnostic printf to terminal
 */ 
void debug( const char* format, ... ) {
  char buffer[200];
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end( args );
#ifdef SerialMonitor
  Serial.printf("%s", buffer);
#endif
}
