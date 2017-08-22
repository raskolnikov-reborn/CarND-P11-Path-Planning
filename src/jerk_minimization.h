/*
 * jerk_minimization.h
 *
 *  Created on: Aug 20, 2017
 *      Author: sahilmalhotra
 */

#ifndef JERK_MINIMIZATION_H_
#define JERK_MINIMIZATION_H_

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


#include "FrameManager.h"
#include "VehicleInstance.h"

using namespace Eigen;
using namespace frame_management;
using namespace global_utils;

namespace jerk_minimization{


/** Check for overspeeding
 *  Declared inline because of duplicate symbol issues
 */
inline bool overspeeding( const vector<double>& jerk_coeffs, const vector<double>& jerk_coeffd, const FrameManager& manager, const map<int,VehicleInstance>& vehicles, double T)
{

	// convert initial frenet pose to xy
	vector<double> initial_position = manager.frenet_2_xy(eval_poly(jerk_coeffs, 0.0), eval_poly(jerk_coeffd, 0.0));

	// get max speed over .01 timestep
	const double time_step = 0.01;

	for (double t = time_step; t <= T ; t += time_step)
	{
		// Get position at that time t
		vector<double> final_position = manager.frenet_2_xy(eval_poly(jerk_coeffs, t), eval_poly(jerk_coeffd, t));

		// calculate speed as ds/dt
		double speed = distance(initial_position[0], initial_position[1], final_position[0], final_position[1]) / time_step;

		// if over limit return true
		if (speed > global_params::speed_limit_mps)
			return true;

		// update initial position
		initial_position = final_position;
	}

	// return false by default
	return false;
}


/** Get nearest neighbor on the path
 * (declared inline due to duplicate symbol issues)
 */
inline double clearance_on_path( const vector<double>& jerk_coeffs, const vector<double>& jerk_coeffd, const FrameManager& manager  , const VehicleInstance& vehicle, double T)
{
	double clearance = 1000000.0;

	// Take 100 samples over path
	for (double t=0.0; t<=T; t+=T/100.0)
	{
		// create ego pose frenet point at time t
		double cur_s = eval_poly(jerk_coeffs, t);
		double cur_d = eval_poly(jerk_coeffd, t);

		// convert to xy
		vector<double> pos_XY = manager.frenet_2_xy(cur_s, cur_d);


		// get position of vehicle (input)
		VehicleInstance::state vehicle_state = vehicle.projected_state(t);

		// get distance between vehicle and ego xy
		double dist = distance(pos_XY[0], pos_XY[1], vehicle_state.x, vehicle_state.y);

		// update closest
		if (dist < clearance)
		{
			clearance=dist;
		}
	}

	// return clearance
	return clearance;
}


/** clearance global minima
 *
 */
inline double clearance_global_minima( const vector<double>& jerk_coeffs, const vector<double>& jerk_coeffd, const FrameManager& manager, const map<int,VehicleInstance>& vehicles, double T)
{
	double clearance = 1000000.0;

	// For all vehicles
	for (auto& v: vehicles)
	{
		// find distance along trajectory
		double dist = clearance_on_path(jerk_coeffs, jerk_coeffd, manager, v.second, T);

		// update clearance
		if (dist < clearance)
			clearance=dist;
	}

	return clearance;
}


/** check for collision
 *  Defined inline due to duplicate symbol issues
 */
inline bool is_collision(  const vector<double>& jerk_coeffs, const vector<double>& jerk_coeffd, const FrameManager& manager, const map<int,VehicleInstance>& vehicles, double T)
{
	// get minimum clearance along the path
	double clearance = clearance_global_minima(jerk_coeffs, jerk_coeffd, manager, vehicles, T);


	// pad for 3.0 times the vehicle radius
	if (clearance < 3.0*global_params::vehicle_radius )
		return true;

	return false;
}


/** Jerk Minimizing Equation coefficients in the frenet space
 *  Defined inline due to duplicate symbol issues
 */
inline vector<double> get_jerk_coeff(trajectory_state start, trajectory_state end, double T)
{

	// Create 3x3 matrix
	MatrixXd A = MatrixXd(3, 3);

	// Row 1 t^3,t^4,t^5
	// Row 2 d/dt(Row1)
	// Row 4 d/dt(Row2)
	A <<  T*T*T, T*T*T*T, T*T*T*T*T,
			3*T*T, 4*T*T*T, 5*T*T*T*T,
			6*T  , 12*T*T ,  20*T*T*T
			;

	// Create 3d vector Reperesenting delta_s, delta_s_dot and delta_s_double_dot
	VectorXd B = VectorXd(3);

	// taking the delta between the target end values and the final end values if they were calculated using newtons equations of motion

	B << end.v - (start.v + start.v_dot*T + 0.5*start.v_double_dot*T*T), //s = ut + 1/2at^2
			end.v_dot - (start.v_dot + start.v_double_dot*T), // v = u +at
			end.v_double_dot - start.v_double_dot; // acceleration difference


	// Use Householder rank-revealing QR decomposition of a matrix with column-pivoting
	VectorXd x = A.colPivHouseholderQr().solve(B);

	// return the values as coefficients
	return {	start.v,
				start.v_dot,
				start.v_double_dot * 0.5,
				x[0],
				x[1],
				x[2]
			};
}


}//namespace


#endif /* JERK_MINIMIZATION_H_ */
