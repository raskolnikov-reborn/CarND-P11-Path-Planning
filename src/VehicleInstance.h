/*
 * VehicleInstance.h
 *
 *  Created on: Aug 20, 2017
 *      Author: sahilmalhotra
 */

#ifndef VEHICLEINSTANCE_H_
#define VEHICLEINSTANCE_H_

#include "global_parameters.h"
#include <vector>
#include <map>
#include <math.h>

struct trajectory_state {
	double v;
	double v_dot;
	double v_double_dot;
};

using namespace std;

// A namespace to hold object type descriptors such as cars
// To be expanded to include static objects like intersections, traffic signs etc at a later point

namespace objects {

class VehicleInstance {
public:

	// Vehicle State in xy and frenet
	struct state {
		double x;
		double y;
		double dx;
		double dy;
		double s;
		double d;
	};


private:
	// current activation state
	bool active_state;
	// current frenet/xy state
	state current_state;
	//current lane
	int lane;

public:
	// Constructor
	VehicleInstance(): active_state(false) {}

	// destructor
	virtual ~VehicleInstance() {}

	// assign a state
	void update_state(state& new_state);

	// get state at time t
	state projected_state(double t) const;

	// return active status
	bool is_active() const {
		return active_state;
	}

	// Set activation status (by default true)
	void set_activation_status(bool activate=true) {
		active_state=activate;
	}

	// getattr for current state
	const state& get_state() const{
		return current_state;
	}

	// get current lane
	int get_lane() const{
		return lane;
	}

};

/** Update the positions of all vehicles in relevance to ego vehicle
 *  De activate all vehicles outside relevance
 */
inline void update_vehicles(double ego_speed, double ego_s, map<int,VehicleInstance>& vehicles, const vector<vector<double>>& sensor_fusion)
{

	// get front clearance to track to
	double front_relevance = ego_speed * global_params::look_ahead_time_seconds*global_params::tracking_windows;


	// at least track for a minimum value (even if speed is too low)
	if (front_relevance < global_params::min_track_horizon_rad)
	{
		front_relevance = global_params::min_track_horizon_rad;
	}


	// compute rear clearace
	double rear_relevance  = ego_s;

	// apply symmetry
	rear_relevance  -= front_relevance;

	// add clearance
	front_relevance += ego_s;

	// update all vehicles
	for(int i = 0; i < sensor_fusion.size(); i++)
	{
		const vector<double> &vehicle = sensor_fusion[i];

		int Id = vehicle[0];
		vehicles.emplace(Id, VehicleInstance());

		VehicleInstance::state v_state;
		v_state.s  = vehicle[5];

		// wraparound track
		double max_s = global_params::max_distance_to_track_meters;

		if (front_relevance > max_s)
		{
			if (v_state.s < (front_relevance - max_s))
			{
				v_state.s += max_s;
			}
		}
		if (rear_relevance < 0.0)
		{
			if (v_state.s > (rear_relevance + max_s))
			{
				v_state.s -= max_s;
			}
		}

		// If vehicle is in relevant zone update it
		if (v_state.s < front_relevance && v_state.s > rear_relevance)
		{
			v_state.x  = vehicle[1];
			v_state.y  = vehicle[2];
			v_state.dx = vehicle[3];
			v_state.dy = vehicle[4];
			v_state.d  = vehicle[6];
			vehicles[Id].update_state(v_state);
		}
		else
		{
			// else deactivate it
			vehicles[Id].set_activation_status(false);
		}
	}
}

}

#endif /* VEHICLEINSTANCE_H_ */
