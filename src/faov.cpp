#include "faov.h"
#include "math.h"
#include "Arduino.h"
/**
Library of functions for estimating reference evapotransporation (ETo) for
a grass reference crop using the FAO-56 Penman-Monteith and Hargreaves
equations. The library includes numerous functions for estimating missing
meteorological data.

(c) 2015 by Mark Richards.
@author David Lopez <dlopez@hsd.cl>
*/

/**
    Convert angular degrees to radians
    :param degrees: Value in degrees to be converted.
    :return: Value in radians
    :rtype: float
*/
float deg2rad(float degrees) {
    return degrees * (PI / 180.0);
}
    

/**
    Check that *hours* is in the range 1 to 24.
*/
bool check_day_hours(int hours, char* arg_name) {
    if (!((0 <= hours) && (hours <= 24))) {
        return false;
    }
    return true;
} 

/**
    Check day of the year is valid.
*/
bool check_doy(int doy) {
    if (!((1 <= doy) && (doy <= 366))) {
        return false;
    }

    return true;
}

/**
 * Check latitude radians is valid
 */
bool check_latitude_rad(float latitude) {
    if (!_MINLAT_RADIANS <= latitude <= _MAXLAT_RADIANS) {
        return false;
    }
    return true;
} 

/**
    Solar declination can vary between -23.5 and +23.5 degrees.
    See http://mypages.iit.edu/~maslanka/SolarGeo.pdf
*/
bool check_sol_dec_rad(float sd) {
    if (!_MINSOLDEC_RADIANS <= sd <= _MAXSOLDEC_RADIANS) {
        return false;
    }
    return true;
}

/**
    Sunset hour angle has the range 0 to 180 degrees.
    See http://mypages.iit.edu/~maslanka/SolarGeo.pdf
*/
bool check_sunset_hour_angle_rad(float sha) {
    if (!_MINSHA_RADIANS <= sha <= _MAXSHA_RADIANS)
        return false;
    return true;
}

/**
    Estimate saturation vapour pressure (*es*) from air temperature.

    Based on equations 11 and 12 in Allen et al (1998).

    :param t: Temperature [deg C]
    :return: Saturation vapour pressure [kPa]
    :rtype: float
*/
float svp_from_t(float t) {
    return 0.6108 * pow(M_E, ((17.27 * t) / (t + 237.3)));
}

/**
    Estimate atmospheric pressure from altitude.

    Calculated using a simplification of the ideal gas law, assuming 20 degrees
    Celsius for a standard atmosphere. Based on equation 7, page 62 in Allen
    et al (1998).

    :param altitude: Elevation/altitude above sea level [m]
    :return: atmospheric pressure [kPa]
    :rtype: float
*/
float atm_pressure(float altitude) {
    float tmp = (293.0 - (0.0065 * altitude)) / 293.0;
    return pow(tmp, 5.26) * 101.3;
}

/**
    Estimate actual vapour pressure (*ea*) from minimum temperature.

    This method is to be used where humidity data are lacking or are of
    questionable quality. The method assumes that the dewpoint temperature
    is approximately equal to the minimum temperature (*tmin*), i.e. the
    air is saturated with water vapour at *tmin*.

    **Note**: This assumption may not hold in arid/semi-arid areas.
    In these areas it may be better to subtract 2 deg C from the
    minimum temperature (see Annex 6 in FAO paper).

    Based on equation 48 in Allen et al (1998).
*/
float avp_from_tmin(float tmin) {
    return 0.611 * pow(M_E,((17.27 * tmin) / (tmin + 237.3)));
}

/**
    Estimate actual vapour pressure (*ea*) from saturation vapour pressure and
    relative humidity.

    Based on FAO equation 17 in Allen et al (1998).

    :param svp_tmin: Saturation vapour pressure at daily minimum temperature
        [kPa]. Can be estimated using ``svp_from_t()``.
    :param svp_tmax: Saturation vapour pressure at daily maximum temperature
        [kPa]. Can be estimated using ``svp_from_t()``.
    :param rh_min: Minimum relative humidity [%]
    :param rh_max: Maximum relative humidity [%]
    :return: Actual vapour pressure [kPa]
    :rtype: float

*/
float avp_from_rhmin_rhmax(float svp_tmin, float svp_tmax, float rh_max, float rh_min) {
    float tmp1 = svp_tmin * (rh_max / 100.0);
    float tmp2 = svp_tmax * (rh_min / 100.0);
    return (tmp1 + tmp2) / 2.0;
}

/**
    Estimate actual vapour pressure (*e*a) from saturation vapour pressure at
    daily minimum temperature and maximum relative humidity

    Based on FAO equation 18 in Allen et al (1998).

    :param svp_tmin: Saturation vapour pressure at daily minimum temperature
        [kPa]. Can be estimated using ``svp_from_t()``.
    :param rh_max: Maximum relative humidity [%]
    :return: Actual vapour pressure [kPa]
    :rtype: float
*/
float avp_from_rhmax(float svp_tmin, float rh_max) {
    return svp_tmin * (rh_max / 100.0);
}

/**
    Estimate actual vapour pressure (*ea*) from saturation vapour pressure at
    daily minimum and maximum temperature, and mean relative humidity.

    Based on FAO equation 19 in Allen et al (1998).

    :param svp_tmin: Saturation vapour pressure at daily minimum temperature
        [kPa]. Can be estimated using ``svp_from_t()``.
    :param svp_tmax: Saturation vapour pressure at daily maximum temperature
        [kPa]. Can be estimated using ``svp_from_t()``.
    :param rh_mean: Mean relative humidity [%] (average of RH min and RH max).
    :return: Actual vapour pressure [kPa]
    :rtype: float

*/
float avp_from_rhmean(float svp_tmin, float svp_tmax, float rh_mean) {
    return (rh_mean / 100.0) * ((svp_tmax + svp_tmin) / 2.0);
}

/**
    Estimate actual vapour pressure (*ea*) from dewpoint temperature.

    Based on equation 14 in Allen et al (1998). As the dewpoint temperature is
    the temperature to which air needs to be cooled to make it saturated, the
    actual vapour pressure is the saturation vapour pressure at the dewpoint
    temperature.

    This method is preferable to calculating vapour pressure from
    minimum temperature.

    :param tdew: Dewpoint temperature [deg C]
    :return: Actual vapour pressure [kPa]
    :rtype: float
*/
float avp_from_tdew(float tdew) {
    return 0.6108 * pow(M_E, ((17.27 * tdew) / (tdew + 237.3)));
}

/**
    Estimate actual vapour pressure (*ea*) from wet and dry bulb temperature.

    Based on equation 15 in Allen et al (1998). As the dewpoint temperature
    is the temperature to which air needs to be cooled to make it saturated, the
    actual vapour pressure is the saturation vapour pressure at the dewpoint
    temperature.

    This method is preferable to calculating vapour pressure from
    minimum temperature.

    Values for the psychrometric constant of the psychrometer (*psy_const*)
    can be calculated using ``psyc_const_of_psychrometer()``.

    :param twet: Wet bulb temperature [deg C]
    :param tdry: Dry bulb temperature [deg C]
    :param svp_twet: Saturated vapour pressure at the wet bulb temperature
        [kPa]. Can be estimated using ``svp_from_t()``.
    :param psy_const: Psychrometric constant of the pyschrometer [kPa deg C-1].
        Can be estimated using ``psy_const()`` or
        ``psy_const_of_psychrometer()``.
    :return: Actual vapour pressure [kPa]
    :rtype: float
*/
float avp_from_twet_tdry(float twet, float tdry, float svp_twet, float psy_const) {
    return svp_twet - (psy_const * (tdry - twet));
}

/**
    Estimate clear sky radiation from altitude and extraterrestrial radiation.

    Based on equation 37 in Allen et al (1998) which is recommended when
    calibrated Angstrom values are not available.

    :param altitude: Elevation above sea level [m]
    :param et_rad: Extraterrestrial radiation [MJ m-2 day-1]. Can be
        estimated using ``et_rad()``.
    :return: Clear sky radiation [MJ m-2 day-1]
    :rtype: float
*/
float cs_rad(float altitude, float et_rad)  {
    return (0.00002 * altitude + 0.75) * et_rad;
}

/**
    Estimate mean daily temperature from the daily minimum and maximum
    temperatures.

    :param tmin: Minimum daily temperature [deg C]
    :param tmax: Maximum daily temperature [deg C]
    :return: Mean daily temperature [deg C]
    :rtype: float
*/
float daily_mean_t(float tmin, float tmax) {
    return (tmax + tmin) / 2.0;
}


/**
    Calculate daylight hours from sunset hour angle.

    Based on FAO equation 34 in Allen et al (1998).

    :param sha: Sunset hour angle [rad]. Can be calculated using
        ``sunset_hour_angle()``.
    :return: Daylight hours.
    :rtype: float

*/
float daylight_hours(float sha) {
    check_sunset_hour_angle_rad(sha);
    return (24.0 / PI) * sha;
}

/**
    Estimate the slope of the saturation vapour pressure curve at a given
    temperature.

    Based on equation 13 in Allen et al (1998). If using in the Penman-Monteith
    *t* should be the mean air temperature.

    :param t: Air temperature [deg C]. Use mean air temperature for use in
        Penman-Monteith.
    :return: Saturation vapour pressure [kPa degC-1]
    :rtype: float
*/
float delta_svp(float t) {
    float tmp = 4098 * (0.6108 * pow(M_E, ((17.27 * t) / (t + 237.3))));
    return tmp / pow((t + 237.3), 2);
}

/**
    Convert energy (e.g. radiation energy) in MJ m-2 day-1 to the equivalent
    evaporation, assuming a grass reference crop.

    Energy is converted to equivalent evaporation using a conversion
    factor equal to the inverse of the latent heat of vapourisation
    (1 / lambda = 0.408).

    Based on FAO equation 20 in Allen et al (1998).

    :param energy: Energy e.g. radiation or heat flux [MJ m-2 day-1].
    :return: Equivalent evaporation [mm day-1].
    :rtype: float
**/
float energy2evap(float energy) {
    return 0.408 * energy;
}

/**
    Estimate daily extraterrestrial radiation (*Ra*, 'top of the atmosphere
    radiation').

    Based on equation 21 in Allen et al (1998). If monthly mean radiation is
    required make sure *sol_dec*. *sha* and *irl* have been calculated using
    the day of the year that corresponds to the middle of the month.

    **Note**: From Allen et al (1998): "For the winter months in latitudes
    greater than 55 degrees (N or S), the equations have limited validity.
    Reference should be made to the Smithsonian Tables to assess possible
    deviations."

    :param latitude: Latitude [radians]
    :param sol_dec: Solar declination [radians]. Can be calculated using
        ``sol_dec()``.
    :param sha: Sunset hour angle [radians]. Can be calculated using
        ``sunset_hour_angle()``.
    :param ird: Inverse relative distance earth-sun [dimensionless]. Can be
        calculated using ``inv_rel_dist_earth_sun()``.
    :return: Daily extraterrestrial radiation [MJ m-2 day-1]
    :rtype: float
*/
float et_rad(float latitude, float sol_dec, float sha, float ird) {
    check_latitude_rad(latitude);
    check_sol_dec_rad(sol_dec);
    check_sunset_hour_angle_rad(sha);

    float tmp1 = (24.0 * 60.0) / PI;
    float tmp2 = sha * sin(latitude) * sin(sol_dec);
    float tmp3 = cos(latitude) * cos(sol_dec) * sin(sha);
    return tmp1 * SOLAR_CONSTANT * ird * (tmp2 + tmp3);
}


/**
    Estimate reference evapotranspiration (ETo) from a hypothetical
    short grass reference surface using the FAO-56 Penman-Monteith equation.

    Based on equation 6 in Allen et al (1998).

    :param net_rad: Net radiation at crop surface [MJ m-2 day-1]. If
        necessary this can be estimated using ``net_rad()``.
    :param t: Air temperature at 2 m height [deg Kelvin].
    :param ws: Wind speed at 2 m height [m s-1]. If not measured at 2m,
        convert using ``wind_speed_at_2m()``.
    :param svp: Saturation vapour pressure [kPa]. Can be estimated using
        ``svp_from_t()''.
    :param avp: Actual vapour pressure [kPa]. Can be estimated using a range
        of functions with names beginning with 'avp_from'.
    :param delta_svp: Slope of saturation vapour pressure curve [kPa degC-1].
        Can be estimated using ``delta_svp()``.
    :param psy: Psychrometric constant [kPa deg C]. Can be estimatred using
        ``psy_const_of_psychrometer()`` or ``psy_const()``.
    :param shf: Soil heat flux (G) [MJ m-2 day-1] (default is 0.0, which is
        reasonable for a daily or 10-day time steps). For monthly time steps
        *shf* can be estimated using ``monthly_soil_heat_flux()`` or
        ``monthly_soil_heat_flux2()``.
    :return: Reference evapotranspiration (ETo) from a hypothetical
        grass reference surface [mm day-1].
    :rtype: float
*/
float fao56_penman_monteith(float net_rad, float t, float ws, float svp, float avp, float delta_svp, float psy, double shf) {
    float a1 = (0.408 * (net_rad - shf) * delta_svp /
          (delta_svp + (psy * (1 + 0.34 * ws))));
    float a2 = (900 * ws / t * (svp - avp) * psy /
          (delta_svp + (psy * (1 + 0.34 * ws))));
    return a1 + a2;
}

/**
    Estimate reference evapotranspiration over grass (ETo) using the Hargreaves
    equation.

    Generally, when solar radiation data, relative humidity data
    and/or wind speed data are missing, it is better to estimate them using
    the functions available in this module, and then calculate ETo
    the FAO Penman-Monteith equation. However, as an alternative, ETo can be
    estimated using the Hargreaves ETo equation.

    Based on equation 52 in Allen et al (1998).

    :param tmin: Minimum daily temperature [deg C]
    :param tmax: Maximum daily temperature [deg C]
    :param tmean: Mean daily temperature [deg C]. If emasurements not
        available it can be estimated as (*tmin* + *tmax*) / 2.
    :param et_rad: Extraterrestrial radiation (Ra) [MJ m-2 day-1]. Can be
        estimated using ``et_rad()``.
    :return: Reference evapotranspiration over grass (ETo) [mm day-1]
    :rtype: float
*/
float hargreaves(float tmin, float tmax, float tmean, float et_rad) {
    // Note, multiplied by 0.408 to convert extraterrestrial radiation could
    // be given in MJ m-2 day-1 rather than as equivalent evaporation in
    // mm day-1
    return 0.0023 * (tmean + 17.8) * (tmax - tmin) * 0.5 * 0.408 * et_rad;
}

/**
    Calculate the inverse relative distance between earth and sun from
    day of the year.


    :param day_of_year: Day of the year [1 to 366]
    :return: Inverse relative distance between earth and the sun
    :rtype: float
*/
float inv_rel_dist_earth_sun(int day_of_year) {
    // TODO Look this function
    check_doy(day_of_year);
    return 1 + (0.033 * cos((2.0 * PI / 365.0) * day_of_year));
}

/**
    Estimate mean saturation vapour pressure, *es* [kPa] from minimum and
    maximum temperature.

    Based on equations 11 and 12 in Allen et al (1998).

    Mean saturation vapour pressure is calculated as the mean of the
    saturation vapour pressure at tmax (maximum temperature) and tmin
    (minimum temperature).

    :param tmin: Minimum temperature [deg C]
    :param tmax: Maximum temperature [deg C]
    :return: Mean saturation vapour pressure (*es*) [kPa]
    :rtype: float
*/
float mean_svp(float tmin, float tmax) {
    return (svp_from_t(tmin) + svp_from_t(tmax)) / 2.0;
}

/**
    Estimate monthly soil heat flux (Gmonth) from the mean air temperature of
    the previous and next month, assuming a grass crop.

    Based on equation 43 in Allen et al (1998). If the air temperature of the
    next month is not known use ``monthly_soil_heat_flux2()`` instead. The
    resulting heat flux can be converted to equivalent evaporation [mm day-1]
    using ``energy2evap()``.

    :param t_month_prev: Mean air temperature of the previous month
        [deg Celsius]
    :param t_month2_next: Mean air temperature of the next month [deg Celsius]
    :return: Monthly soil heat flux (Gmonth) [MJ m-2 day-1]
    :rtype: float
*/
float monthly_soil_heat_flux(float t_month_prev, float t_month_next) {
    return 0.07 * (t_month_next - t_month_prev);
}

/**
    Estimate monthly soil heat flux (Gmonth) [MJ m-2 day-1] from the mean
    air temperature of the previous and current month, assuming a grass crop.

    Based on equation 44 in Allen et al (1998). If the air temperature of the
    next month is available, use ``monthly_soil_heat_flux()`` instead. The
    resulting heat flux can be converted to equivalent evaporation [mm day-1]
    using ``energy2evap()``.

    Arguments:
    :param t_month_prev: Mean air temperature of the previous month
        [deg Celsius]
    :param t_month_cur: Mean air temperature of the current month [deg Celsius]
    :return: Monthly soil heat flux (Gmonth) [MJ m-2 day-1]
    :rtype: float
*/
float monthly_soil_heat_flux2(float t_month_prev, float t_month_cur) {
    return 0.14 * (t_month_cur - t_month_prev);
}

/**
    Calculate net incoming solar (or shortwave) radiation from gross
    incoming solar radiation, assuming a grass reference crop.

    Net incoming solar radiation is the net shortwave radiation resulting
    from the balance between incoming and reflected solar radiation. The
    output can be converted to equivalent evaporation [mm day-1] using
    ``energy2evap()``.

    Based on FAO equation 38 in Allen et al (1998).

    :param sol_rad: Gross incoming solar radiation [MJ m-2 day-1]. If
        necessary this can be estimated using functions whose name
        begins with 'sol_rad_from'.
    :param albedo: Albedo of the crop as the proportion of gross incoming solar
        radiation that is reflected by the surface. Default value is 0.23,
        which is the value used by the FAO for a short grass reference crop.
        Albedo can be as high as 0.95 for freshly fallen snow and as low as
        0.05 for wet bare soil. A green vegetation over has an albedo of
        about 0.20-0.25 (Allen et al, 1998).
    :return: Net incoming solar (or shortwave) radiation [MJ m-2 day-1].
    :rtype: float

*/
float net_in_sol_rad(float sol_rad, double albedo) {
    return (1 - albedo) * sol_rad;
}

/**
    Estimate net outgoing longwave radiation.

    This is the net longwave energy (net energy flux) leaving the
    earth's surface. It is proportional to the absolute temperature of
    the surface raised to the fourth power according to the Stefan-Boltzmann
    law. However, water vapour, clouds, carbon dioxide and dust are absorbers
    and emitters of longwave radiation. This function corrects the Stefan-
    Boltzmann law for humidity (using actual vapor pressure) and cloudiness
    (using solar radiation and clear sky radiation). The concentrations of all
    other absorbers are assumed to be constant.

    The output can be converted to equivalent evaporation [mm day-1] using
    ``energy2evap()``.

    Based on FAO equation 39 in Allen et al (1998).

    :param tmin: Absolute daily minimum temperature [degrees Kelvin]
    :param tmax: Absolute daily maximum temperature [degrees Kelvin]
    :param sol_rad: Solar radiation [MJ m-2 day-1]. If necessary this can be
        estimated using ``sol+rad()``.
    :param cs_rad: Clear sky radiation [MJ m-2 day-1]. Can be estimated using
        ``cs_rad()``.
    :param avp: Actual vapour pressure [kPa]. Can be estimated using functions
        with names beginning with 'avp_from'.
    :return: Net outgoing longwave radiation [MJ m-2 day-1]
    :rtype: float
*/
float net_out_lw_rad(float tmin, float tmax, float sol_rad, float cs_rad, float avp) {
    float tmp1 = (STEFAN_BOLTZMANN_CONSTANT * ((pow(tmax, 4) + pow(tmin, 4)) / 2));
    float tmp2 = (0.34 - (0.14 * sqrt(avp)));
    float tmp3 = 1.35 * (sol_rad / cs_rad) - 0.35;
    return tmp1 * tmp2 * tmp3;
}

/**
    Calculate daily net radiation at the crop surface, assuming a grass
    reference crop.

    Net radiation is the difference between the incoming net shortwave (or
    solar) radiation and the outgoing net longwave radiation. Output can be
    converted to equivalent evaporation [mm day-1] using ``energy2evap()``.

    Based on equation 40 in Allen et al (1998).

    :param ni_sw_rad: Net incoming shortwave radiation [MJ m-2 day-1]. Can be
        estimated using ``net_in_sol_rad()``.
    :param no_lw_rad: Net outgoing longwave radiation [MJ m-2 day-1]. Can be
        estimated using ``net_out_lw_rad()``.
    :return: Daily net radiation [MJ m-2 day-1].
    :rtype: float
*/
float net_rad(float ni_sw_rad, float no_lw_rad) {
    return ni_sw_rad - no_lw_rad;
}

/**
    Calculate the psychrometric constant.

    This method assumes that the air is saturated with water vapour at the
    minimum daily temperature. This assumption may not hold in arid areas.

    Based on equation 8, page 95 in Allen et al (1998).

    :param atmos_pres: Atmospheric pressure [kPa]. Can be estimated using
        ``atm_pressure()``.
    :return: Psychrometric constant [kPa degC-1].
    :rtype: float

*/
float psy_const(float atmos_pres) {
    return 0.000665 * atmos_pres;
}


/**
    Calculate the psychrometric constant for different types of
    psychrometer at a given atmospheric pressure.

    Based on FAO equation 16 in Allen et al (1998).

    :param psychrometer: Integer between 1 and 3 which denotes type of
        psychrometer:
        1. ventilated (Asmann or aspirated type) psychrometer with
           an air movement of approximately 5 m/s
        2. natural ventilated psychrometer with an air movement
           of approximately 1 m/s
        3. non ventilated psychrometer installed indoors
    :param atmos_pres: Atmospheric pressure [kPa]. Can be estimated using
        ``atm_pressure()``.
    :return: Psychrometric constant [kPa degC-1].
    :rtype: float

*/
float psy_const_of_psychrometer(float psychrometer, float atmos_pres) {
    float psy_coeff = 0;
    //Select coefficient based on type of ventilation of the wet bulb
    if (psychrometer == 1){
      psy_coeff = 0.000662;
    }
    else if (psychrometer == 2){
      psy_coeff = 0.000800;
    }
    else if (psychrometer == 3){
        psy_coeff = 0.001200;
    }
    else {
        return -1;
    }

    return psy_coeff * atmos_pres;
}

/**
    Calculate relative humidity as the ratio of actual vapour pressure
    to saturation vapour pressure at the same temperature.

    See Allen et al (1998), page 67 for details.

    :param avp: Actual vapour pressure [units do not matter so long as they
        are the same as for *svp*]. Can be estimated using functions whose
        name begins with 'avp_from'.
    :param svp: Saturated vapour pressure [units do not matter so long as they
        are the same as for *avp*]. Can be estimated using ``svp_from_t()``.
    :return: Relative humidity [%].
    :rtype: float
*/
float rh_from_avp_svp(float avp, float svp) {
    return 100.0 * avp / svp;
}


/**
    Calculate solar declination from day of the year.

    Based on FAO equation 24 in Allen et al (1998).

    :param day_of_year: Day of year integer between 1 and 365 or 366).
    :return: solar declination [radians]
    :rtype: float
*/
float sol_dec(float day_of_year) {
    //TODO Look this function
    check_doy(day_of_year);
    return 0.409 * sin(((2.0 * PI / 365.0) * day_of_year - 1.39));
}

/**
    Calculate incoming solar (or shortwave) radiation, *Rs* (radiation hitting
    a horizontal plane after scattering by the atmosphere) from relative
    sunshine duration.

    If measured radiation data are not available this method is preferable
    to calculating solar radiation from temperature. If a monthly mean is
    required then divide the monthly number of sunshine hours by number of
    days in the month and ensure that *et_rad* and *daylight_hours* was
    calculated using the day of the year that corresponds to the middle of
    the month.

    Based on equations 34 and 35 in Allen et al (1998).

    :param dl_hours: Number of daylight hours [hours]. Can be calculated
        using ``daylight_hours()``.
    :param sunshine_hours: Sunshine duration [hours].
    :param et_rad: Extraterrestrial radiation [MJ m-2 day-1]. Can be
        estimated using ``et_rad()``.
    :return: Incoming solar (or shortwave) radiation [MJ m-2 day-1]
    :rtype: float
*/
float sol_rad_from_sun_hours(int daylight_hours, int sunshine_hours, float et_rad) {
    check_day_hours(sunshine_hours, "sun_hours");
    check_day_hours(daylight_hours, "daylight_hours");

    //0.5 and 0.25 are default values of regression constants (Angstrom values)
    //recommended by FAO when calibrated values are unavailable.
    return (0.5 * sunshine_hours / daylight_hours + 0.25) * et_rad;
}

/**
    Estimate incoming solar (or shortwave) radiation, *Rs*, (radiation hitting
    a horizontal plane after scattering by the atmosphere) from min and max
    temperature together with an empirical adjustment coefficient for
    'interior' and 'coastal' regions.

    The formula is based on equation 50 in Allen et al (1998) which is the
    Hargreaves radiation formula (Hargreaves and Samani, 1982, 1985). This
    method should be used only when solar radiation or sunshine hours data are
    not available. It is only recommended for locations where it is not
    possible to use radiation data from a regional station (either because
    climate conditions are heterogeneous or data are lacking).

    **NOTE**: this method is not suitable for island locations due to the
    moderating effects of the surrounding water.

    :param et_rad: Extraterrestrial radiation [MJ m-2 day-1]. Can be
        estimated using ``et_rad()``.
    :param cs_rad: Clear sky radiation [MJ m-2 day-1]. Can be estimated
        using ``cs_rad()``.
    :param tmin: Daily minimum temperature [deg C].
    :param tmax: Daily maximum temperature [deg C].
    :param coastal: ``True`` if site is a coastal location, situated on or
        adjacent to coast of a large land mass and where air masses are
        influenced by a nearby water body, ``False`` if interior location
        where land mass dominates and air masses are not strongly influenced
        by a large water body.
    :return: Incoming solar (or shortwave) radiation (Rs) [MJ m-2 day-1].
    :rtype: float
*/
float sol_rad_from_t(float et_rad, float cs_rad, float tmin, float tmax, bool coastal) {
    //Determine value of adjustment coefficient [deg C-0.5] for
    //coastal/interior locations
    float adj = 0;
    if (coastal)
        adj = 0.19;
    else
        adj = 0.16;

    float sol_rad = adj * sqrt(tmax - tmin) * et_rad;

    return min(sol_rad, cs_rad);
}

/**
    Estimate incoming solar (or shortwave) radiation, *Rs* (radiation hitting
    a horizontal plane after scattering by the atmosphere) for an island
    location.

    An island is defined as a land mass with width perpendicular to the
    coastline <= 20 km. Use this method only if radiation data from
    elsewhere on the island is not available.

    **NOTE**: This method is only applicable for low altitudes (0-100 m)
    and monthly calculations.

    Based on FAO equation 51 in Allen et al (1998).

    :param et_rad: Extraterrestrial radiation [MJ m-2 day-1]. Can be
        estimated using ``et_rad()``.
    :return: Incoming solar (or shortwave) radiation [MJ m-2 day-1].
    :rtype: float
*/
float sol_rad_island(float et_rad) {
    return (0.7 * et_rad) - 4.0;
}


/**
    Calculate sunset hour angle (*Ws*) from latitude and solar
    declination.

    Based on FAO equation 25 in Allen et al (1998).

    :param latitude: Latitude [radians]. Note: *latitude* should be negative
        if it in the southern hemisphere, positive if in the northern
        hemisphere.
    :param sol_dec: Solar declination [radians]. Can be calculated using
        ``sol_dec()``.
    :return: Sunset hour angle [radians].
    :rtype: float
*/
float sunset_hour_angle(float latitude, float sol_dec) {
    // TODO Look this functions
    check_latitude_rad(latitude);
    check_sol_dec_rad(sol_dec);

    float cos_sha = -tan(latitude) * tan(sol_dec);
    // If tmp is >= 1 there is no sunset, i.e. 24 hours of daylight
    // If tmp is <= 1 there is no sunrise, i.e. 24 hours of darkness
    // See http://www.itacanet.org/the-sun-as-a-source-of-energy/
    // part-3-calculating-solar-angles/
    // Domain of acos is -1 <= x <= 1 radians (this is not mentioned in FAO-56!)
    return 1/cos(fmin(fmax(cos_sha, -1.0), 1.0));
}



/**
    Convert wind speed measured at different heights above the soil
    surface to wind speed at 2 m above the surface, assuming a short grass
    surface.

    Based on FAO equation 47 in Allen et al (1998).

    :param ws: Measured wind speed [m s-1]
    :param z: Height of wind measurement above ground surface [m]
    :return: Wind speed at 2 m above the surface [m s-1]
    :rtype: float
*/
float wind_speed_2m(float ws, float z) {
    return ws * (4.87 / log((67.8 * z) - 5.42));
}
