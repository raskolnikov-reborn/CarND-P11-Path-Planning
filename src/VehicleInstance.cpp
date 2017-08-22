/*
 * VehicleInstance.cpp
 *
 *  Created on: Aug 20, 2017
 *      Author: sahilmalhotra
 */

#include "VehicleInstance.h"

namespace objects {

void VehicleInstance::update_state(state& new_state) {
	current_state = new_state;

	// update current lane based on d
	lane = trunc(current_state.d / global_params::lane_width);

	// set activation status to true
	set_activation_status();
}


// get Projected state at time t
VehicleInstance::state VehicleInstance::projected_state(double t) const
{
	state projected_state = current_state;

	// using x = x0 + dx*dt
	projected_state.x  += current_state.dx * t;
	projected_state.y  += current_state.dy * t;

	return projected_state;
}


}
