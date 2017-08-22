/*
 * FrameManager.h
 *
 *  Created on: Aug 20, 2017
 *      Author: sahilmalhotra
 */

#ifndef FRAMEMANAGER_H_
#define FRAMEMANAGER_H_

#include "spline.h"
#include "global_parameters.h"

#include <fstream>
#include <vector>
#include <math.h>
#include <sstream>

using namespace std;

namespace frame_management {

class FrameManager {
public:
	FrameManager(const string& map_file);
	virtual ~FrameManager();



	int ClosestWaypoint(const double x, const double y) const;
	int NextWaypoint(const double x, const double y, const double theta) const;


	vector<double> frenet_2_xy(const double s, const double d) const;
	vector<double> xy_2_frenet(const double x, const double y, const double theta) const;

	// input waypoints read from file
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Splines passing through all waypoints
	tk::spline waypoint_spline_x;
	tk::spline waypoint_spline_y;
	tk::spline waypoint_spline_dx;
	tk::spline waypoint_spline_dy;

};

}

#endif /* FRAMEMANAGER_H_ */
