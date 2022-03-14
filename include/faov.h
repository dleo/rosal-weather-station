#ifndef DEFINES_FAOV_H
#define DEFINES_FAOV_H
// Solar constant [ MJ m-2 min-1]
#define SOLAR_CONSTANT 0.0820

// Stefan Boltzmann constant [MJ K-4 m-2 day-1]
#define STEFAN_BOLTZMANN_CONSTANT 0.000000004903

#define PI 3.1415926535897932384626433832795

float deg2rad(float degrees);

const float _MINLAT_RADIANS = deg2rad(-90.0);
const float _MAXLAT_RADIANS = deg2rad(90.0);
// Solar declination
const float _MINSOLDEC_RADIANS = deg2rad(-23.5);
const float _MAXSOLDEC_RADIANS = deg2rad(23.5);

// Sunset hour angle
const float _MINSHA_RADIANS = 0.0;
const float _MAXSHA_RADIANS = deg2rad(180);



bool check_day_hours(int hours, char arg_name);
bool check_doy(int doy);
bool check_latitude_rad(float latitude);
bool check_sol_dec_rad(float sd);
bool check_sunset_hour_angle_rad(float sha);
float svp_from_t(float t);
float atm_pressure(float altitude);
float avp_from_tmin(float tmin);
float avp_from_rhmin_rhmax(float svp_tmin, float svp_tmax, float rh_max, float rh_min);
float avp_from_rhmax(float svp_tmin, float rh_max);
float avp_from_rhmean(float svp_tmin, float svp_tmax, float rh_mean);
float avp_from_tdew(float tdew);
float avp_from_twet_tdry(float twet, float tdry, float svp_twet, float psy_const);
float cs_rad(float altitude, float et_rad);
float daily_mean_t(float tmin, float tmax);
float daylight_hours(float sha);
float delta_svp(float t);
float energy2evap(float energy);
float et_rad(float latitude, float sol_dec, float sha, float ird);
float fao56_penman_monteith(float net_rad, float t, float ws, float svp, float avp, float delta_svp, float psy, double shf = 0.0);
float hargreaves(float tmin, float tmax, float tmean, float et_rad);
float inv_rel_dist_earth_sun(int day_of_year);
float mean_svp(float tmin, float tmax);
float monthly_soil_heat_flux(float t_month_prev, float t_month_next);
float monthly_soil_heat_flux2(float t_month_prev, float t_month_cur);
float net_in_sol_rad(float sol_rad, double albedo = 0.23);
float net_out_lw_rad(float tmin, float tmax, float sol_rad, float cs_rad, float avp);
float net_rad(float ni_sw_rad, float no_lw_rad);
float psy_const(float atmos_pres);
float psy_const_of_psychrometer(float psychrometer, float atmos_pres);
float rh_from_avp_svp(float avp, float svp);
float sol_dec(float day_of_year);
float sol_rad_from_sun_hours(int daylight_hours, int sunshine_hours, float et_rad);
float sol_rad_from_t(float et_rad, float cs_rad, float tmin, float tmax, bool coastal);
float sol_rad_island(float et_rad);
float sunset_hour_angle(float latitude, float sol_dec);
float wind_speed_2m(float ws, float z);
float fao56_eto_hr(float net_rad, float t, float psy, float ws, float delta_svp, float avp, float svp, double g = 0.0);
float hourly_soil_heat_flux(float net_radiation, bool isDay);
#endif