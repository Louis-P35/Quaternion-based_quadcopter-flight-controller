#pragma once


/*
This function read the altutude measured
by the barometric sensor.
Using exponential moving average as a filter.
*/
double readAltitude(MS5611* pBarometer);

double computeAltitude(double accX, double accY, double accZ, double roll_rad, double pitch_rad, double barometerAlt, double dt);