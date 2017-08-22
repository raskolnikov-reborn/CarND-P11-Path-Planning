/*
 * lane_controls.h
 *
 *  Created on: Aug 20, 2017
 *      Author: sahilmalhotra
 */

#ifndef LANE_CONTROLS_H_
#define LANE_CONTROLS_H_

#include "VehicleInstance.h"

using namespace std;
using namespace objects;

namespace lane_controls{


struct nearest_neighbor {
	// default is -1,0.0
	nearest_neighbor(): id(-1), distance(0.0) {}
	//
	int id;
	double distance;
};


/** convert between lane and d value
 *  return center point of lane
 */
inline double convert_lane_to_d(int lane)
{
	return (double(lane) * global_params::lane_width + global_params::lane_width*0.5);
}

/** Get nearest neighbor(vehicle) in front
 */
nearest_neighbor get_front_nearest_neighbor(int lane, double pos_s, map<int,VehicleInstance>& vehicles)
{
	// declare neighbour at maximum distance
	nearest_neighbor neighbour;
	double nearest_s = global_params::max_distance_to_track_meters;

	// Define a horizon for wrapping
	double wrap_horizon_s = pos_s + global_params::max_distance_to_track_meters/2;
	wrap_horizon_s -= global_params::max_distance_to_track_meters;

	for (auto& v:vehicles)
	{
		const VehicleInstance& vehicle = v.second;

		if (vehicle.is_active() && vehicle.get_lane() == lane)
		{
			double vehicle_s = vehicle.get_state().s;
			// if in ego lane and ahead of ego vehicle
			if (vehicle_s < wrap_horizon_s)
			{
				vehicle_s += global_params::max_distance_to_track_meters;
			}
			// if closer than current neighbour
			if (vehicle_s > pos_s && vehicle_s < nearest_s)
			{
				//update id and distance
				nearest_s = vehicle_s;
				neighbour.id = v.first;
				neighbour.distance  = vehicle_s - pos_s;
			}
		}
	}

	return neighbour;
}

/** get nearest neighbour behind the car
 *
 */
inline nearest_neighbor get_rear_nearest_neighbor(int lane, double pos_s, map<int,VehicleInstance>& vehicles) {
	// declare neighbour at minimum distance
	nearest_neighbor neighbour;
	double nearest_s = 0.0;

	// set horizon for wrapping
	double wrap_horizon_s = pos_s - global_params::max_distance_to_track_meters/2;
	wrap_horizon_s += global_params::max_distance_to_track_meters/2;

	// Iterate over all vehicles
	for (auto& v: vehicles)
	{
		const VehicleInstance& vehicle = v.second;

		if (vehicle.is_active() && vehicle.get_lane() == lane)
		{
			double vehicle_s = vehicle.get_state().s;

			// reverse wraparound
			if (vehicle_s > wrap_horizon_s)
			{
				vehicle_s -= global_params::max_distance_to_track_meters;
			}
			// if nearer than current neighbour
			if (vehicle_s < pos_s && vehicle_s > nearest_s)
			{
				// update id and distance
				nearest_s      = vehicle_s;
				neighbour.id        = v.first;
				neighbour.distance  = pos_s - vehicle_s;
			}
		}
	}
	return neighbour;
}

/** Cost heuristics for lane navigation
 *
 */
double lane_action_cost(int my_lane, double my_s, double my_d, int delta_lane, map<int,VehicleInstance>& vehicles) {
	int target_lane = my_lane + delta_lane;

	// if leaving road , cost is lethal
	if (target_lane < 0 || target_lane > 2)
	{
		return global_params::lethal_cost;
	}

	// find nearest neighbours
	nearest_neighbor front_target = get_front_nearest_neighbor(target_lane, my_s, vehicles);
	nearest_neighbor back_target  = get_rear_nearest_neighbor(target_lane, my_s, vehicles);


	// keep lane should have lower cost than shift lane
	double cost = 0.0;

	// if there is a vehicle in front
	if (front_target.id != -1)
	{
		// If front distance is not sufficient to attempt lane change
		if (delta_lane != 0 && front_target.distance < global_params::min_clearance)
		{
			return global_params::lethal_cost - 1.0;
		}

		// Add cost proportional to speed
		const VehicleInstance::state& vstate = vehicles[front_target.id].get_state();

		double speed = sqrt(vstate.dx*vstate.dx + vstate.dy*vstate.dy);
		cost += (global_params::speed_limit_mps - speed) / global_params::speed_limit_mps;

		// Add cost inversely proportional to clearance
		cost += exp(-front_target.distance / global_params::min_clearance);

	}

	if (delta_lane != 0 && back_target.id != -1)
	{
		// if rear clearance is not sufficient
		if (back_target.distance < global_params::min_clearance)
		{
			return global_params::lethal_cost - 1.0;
		}
	}

	// add lane state change cost
	double target_d = convert_lane_to_d(target_lane);
	cost += 0.15 * fabs(target_d - my_d) / (1.5 * global_params::lane_width);

	return cost;
}

}


#endif /* LANE_CONTROLS_H_ */
