/*
 * FrameManager.cpp
 *
 *  Created on: Aug 20, 2017
 *      Author: sahilmalhotra
 */

#include "FrameManager.h"

using namespace std;
using namespace global_utils;

namespace frame_management {


/** Constructor : Read map from file
 * @param map_file  map file location
 * read points and interpolate splines
 */

FrameManager::FrameManager(const string& map_file) {

	// load points from file
	ifstream in_map_(map_file.c_str(), ifstream::in);
	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}


	// Interpolate
	waypoint_spline_x.set_points(map_waypoints_s, map_waypoints_x);
	waypoint_spline_y.set_points(map_waypoints_s, map_waypoints_y);
	waypoint_spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
	waypoint_spline_dy.set_points(map_waypoints_s, map_waypoints_dy);


	// Remove Last points for wraparound issues
	map_waypoints_s.pop_back();
	map_waypoints_x.pop_back();
	map_waypoints_y.pop_back();
	map_waypoints_dx.pop_back();
	map_waypoints_dy.pop_back();
}

/** Convert from frenet to world coord
 * @params frenet coordinates
 * returns xy co ordinates
 */
vector<double> FrameManager::frenet_2_xy(const double s, const double d) const {

	// Get xy coordinate on the waypoint spline
	double x = waypoint_spline_x(s);
	double y = waypoint_spline_y(s);

	// Add the offset from d
	x += waypoint_spline_dx(s) * d;
	y += waypoint_spline_dy(s) * d;

	return {x,y};
}


/** Return the index of the closest waypoint to (x,y)
 *  get closest waypoint to x,y position
 *  @params x,y position
 *  returns waypoint index
 */
int FrameManager::ClosestWaypoint(const double x, const double y) const {
	double closestLen = 100000.0;
	int closestWaypoint = 0;

	// iterate all waypoints
	for(int i = 0; i < map_waypoints_x.size(); i++)
	{
		// get map_x, map_y positions
		double map_x = map_waypoints_x[i];
		double map_y = map_waypoints_y[i];

		// get distance between input and map positions
		double dist = distance(x,y,map_x,map_y);

		// if distance is less than current minimum
		if(dist < closestLen)
		{
			//update index
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	// return Index
	return closestWaypoint;
}

/** Return the index of the next waypoint from position (x,y)
 *  get closest waypoint to x,y position
 *  @params x,y position
 *  returns waypoint index
 */
int FrameManager::NextWaypoint(const double x, const double y, const double theta) const {

	// get the closest waypoint index
	int nextWaypoint = ClosestWaypoint(x,y);

	// get xy corresponding to the closest index
	double map_x = map_waypoints_x[nextWaypoint];
	double map_y = map_waypoints_y[nextWaypoint];

	// get heading
	double yaw = atan2( (map_y-y),(map_x-x) );

	// calculate absolute angle difference
	double theta_pos = fmod(theta + (2*pi()),2*pi());
	double heading_pos = fmod(yaw + (2*pi()),2*pi());
	double angle = fabs(theta_pos-heading_pos);
	if (angle > pi())
	{
		angle = (2*pi()) - angle;
	}

	// if the angle > 90 degrees
	if (angle > pi()/2.0)
	{
		// increase index by one
		nextWaypoint++;
		// wraparound (in case the closest waypoint was the last waypoint)
		nextWaypoint %= map_waypoints_x.size();
	}

	// return the index
	return nextWaypoint;
}


/** Transform from Cartesian x,y coordinates to Frenet s,d coordinates
 * @params xy positions
 * returns frenet points on smooth spline
 *
 */
vector<double> FrameManager::xy_2_frenet(const double x, const double y, const double theta) const {

	//get next and previous waypoints
	int next_waypoint = NextWaypoint(x,y,theta);
	int previous_waypoint = next_waypoint-1;

	// reverse wraparound
	if (next_waypoint == 0)
	{
		previous_waypoint  = map_waypoints_x.size()-1;
	}

	// get road dx, dy
	double n_x = map_waypoints_x[next_waypoint]-map_waypoints_x[previous_waypoint];
	double n_y = map_waypoints_y[next_waypoint]-map_waypoints_y[previous_waypoint];

	// get vehicle dx,dy
	double x_x = x - map_waypoints_x[previous_waypoint];
	double x_y = y - map_waypoints_y[previous_waypoint];

	// project vehicle orientation on to road orientation
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);

	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//compare to center point to check for sign
	double center_x = 1000.0 - map_waypoints_x[previous_waypoint];
	double center_y = 2000.0 - map_waypoints_y[previous_waypoint];

	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s
	double frenet_s = map_waypoints_s[previous_waypoint];
	frenet_s += distance(0.0,0.0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

/**
 * Destructor
 */
FrameManager::~FrameManager()
{

}

}
