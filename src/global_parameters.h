/*
 * global_parameters.h
 *
 *  Created on: Aug 4, 2017
 *      Author: sahilmalhotra
 *
 *      This file contains global parameters declarations in their own namespace.
 *      TODO: Will take away all these parameters into a config file and write a parser.
 *      That way any change in params does not require recompilation of code.
 */

#ifndef GLOBAL_PARAMETERS_H_
#define GLOBAL_PARAMETERS_H_

#include <string>
#include <math.h>
#include <vector>

using namespace std;
//Namespace for global_parameters
namespace global_params
{
// Map file to be read
const std::string map_filename = "../data/highway_map.csv";

// speed limit
const float speed_limit_mps = 21.5;

// speed limit (transverse)
const float transverse_speed_limit = 1.0;

// Simulation lookahead time
const float look_ahead_time_seconds = 2.25;

// simulation step increment in seconds
const float look_ahead_resolution_seconds = 0.02;

// Prohibitively high lethal cost
const float lethal_cost = 1000.0;

// how many multiples of look ahead time should be tracked
const float tracking_windows =  3.0;

// maximum distance after which s should be wrapped
const double max_distance_to_track_meters = 6945.554;

// min clearance for front and rear. Try to maintain clearance > min
const float min_clearance = 8.0;

// acceleration_limit 10 m/s^2 as per project rubric
const float max_permissible_acceleration = 10;

// debug flag for enabling disabling prints
const bool debug = false;

// lane width
const double lane_width = 4.0;

// constant for conversion
const double mph_mps = 0.447038889;

// minimum window for which tracking should be done even if speed is too low
const double min_track_horizon_rad = 30.0;

// no of points to keep from last path
const int max_points_to_keep = 20;

// approximate radius of vehicle as a point object
const double vehicle_radius = 2.5;
};

namespace global_utils{

// Get the sign of a number
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


// Angle Conversions
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }


// Euclidean Distance
inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


// miles per hour to meters per second converter
inline double convert_mph_to_ms(double v)
{
  return v * global_params::mph_mps;
}

/** convert between lane and d value
 *  return center point of lane
 */
inline double convert_lane_to_d(int lane)
{
  return double(lane) * global_params::lane_width + global_params::lane_width / 2.0;
}

// convert from d to l using banding
inline int convert_d_to_lane(double d)
{
  return trunc(d / global_params::lane_width);
}

// wrap_s around
inline double wrap_s(double s)
{
	double limit = global_params::max_distance_to_track_meters;
	while (s >= limit)
	{
		s -= limit;
	}
	while (s < 0.0)
	{
		s += limit;
	}
	return s;
}

// evaluate polynomial at value
inline double eval_poly(const vector<double>& coeffs, const double x) {
	double result = 0.0;
	for (int i=0; i<coeffs.size(); i++) {
		result += coeffs[i] * pow(x,i);
	}

	return result;
}

// get derivative of poly
inline vector<double> derivative_poly(const vector<double>& coeffs) {
	vector<double> dcoeffs(coeffs.size()-1);

	for (int i=1; i<coeffs.size(); i++)
	{
		dcoeffs[i-1] = coeffs[i] * double(i);
	}

	return dcoeffs;
}
}


#endif /* GLOBAL_PARAMETERS_H_ */
