#include "weather.h"
#include "faov.h"
#include "TimeLib.h"
#include "secrets.h"
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
  return (time / 86400.0) + 2440587.5;
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
  double jd = 0;              // Julian Date
  float phase;
  jd = julianDate(time);
  // Calculate illumination (synodic) phase.
  // From number of days since new moon on Julian date MOON_SYNODIC_OFFSET
  // (1815UTC January 6, 2000), determine remainder of incomplete cycle.
  phase = (jd - MOON_SYNODIC_OFFSET) / MOON_SYNODIC_PERIOD;
  phase -= floor(phase);
  
  return (int)(phase * 8 + 0.5) % 8;
}

/**
 * @brief Moon phase on string
 * 
 * @param phase 
 * @return char* 
 * 
 */
char *moonPhaseToString(int phase)
{
  char *phaseNames[] = {"New Moon", "Old Crescent", "First Quarter",
				   "Waxing Gibbous", "Full moon", "Waning Gibbous",
				   "Last Quarter", "Morning Crescent"};

  return phaseNames[phase];
}

/**
 * Calculate eto
 */
void calculateEto(struct sensorData *environment, boolean hourly)
{  
  environment->eto = 0;
  environment->etoDaily = 0;
  float pressure_kpa = environment->pressure / 10;
  float dsvp = delta_svp(environment->temperature);       // kPa Pendiente de la curva de presión de saturación de vapor (e. 13)
  float psy = psy_const(pressure_kpa);                    // kPa
  float svp = svp_from_t(environment->temperature);       // kPa Presión media de vapor de la saturación (e. 11)
  float avp = avp_from_rhmax(svp, environment->humidity);
  float temp_k = environment->temperature + 273.15;       // Convertion C -> K
  float irradiation = environment->irradiation * 0.0864;  // Convertion to MJ m-2 día-1
  float sol_declination = sol_dec(dayOfYear);
  float sha = sunset_hour_angle(LATITUDE, sol_declination);
  float ird = inv_rel_dist_earth_sun(dayOfYear);
  float et_radiation = et_rad(LATITUDE, sol_declination, sha, ird);
  float cs_radiation = cs_rad(environment->altitude, et_radiation);
  float net_radiation = net_rad(
      net_in_sol_rad(irradiation),
      net_out_lw_rad(environment->temperature, environment->temperature, irradiation, cs_radiation, avp));
  float wind_speed_ms = (int)environment->windSpeed / 3.6; // Convertion km/h -> m/s
  float eto_daily = fao56_penman_monteith(net_radiation, temp_k, wind_speed_ms, svp, avp, dsvp, psy);
  environment->etoDaily = eto_daily;
  if (hourly)
  {
    float g = hourly_soil_heat_flux(net_radiation, isDay(now()));
    environment->eto = fao56_eto_hr(net_radiation, environment->temperature, psy, wind_speed_ms, dsvp, avp, svp, g);
  }
  /*

   Without solar radiation sensor
   int dayOfYear = 1; // TODO Calculated
   float ird = inv_rel_dist_earth_sun(dayOfYear);
   float solDec = sol_dec(dayOfYear);
   float sha = sunset_hour_angle(LAT, solDec);
   int daylightHours = daylight_hours(sha);
   float etRad = et_rad(LAT, solDec, sha, ird);
   float solRad = sol_rad_from_sun_hours(daylightHours, SUNSHINEHOURS, etRad);
   float niSwRad = net_in_sol_rad(solRad);
   float csRad = cs_rad(LAT, etRad);
   float avp = avp_from_tdew(environment->dewPoint);
   // TODO Calculations for tmin a tmax
   float noLwRad = net_out_lw_rad(18 + CTOK, 28 + CTOK, solRad, csRad, avp);
   float netRad = net_rad(niSwRad, noLwRad);
   float svp = svp_from_t(environment->humidity);
   float dsvp = delta_svp(environment->temperature);
   float psy = psy_const(environment->pressure);
   return fao56_penman_monteith(netRad, environment->temperature + CTOK, environment->windSpeed * 2.4, svp, avp, dsvp, psy);
   */
}

/**
 * Diagnostic printf to terminal
 */
void debug(const char *format, ...)
{
  char buffer[200];
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);
#ifdef SerialMonitor
  Serial.printf("%s", buffer);
#endif
}
