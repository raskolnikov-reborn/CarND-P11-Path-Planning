#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"


#include "lane_controls.h"
#include "jerk_minimization.h"
#include "spline.h"
#include "VehicleInstance.h"

using namespace std;
using namespace frame_management;
using namespace objects;
using namespace global_params;
using namespace global_utils;
using namespace lane_controls;
using namespace jerk_minimization;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}


int main() {
	uWS::Hub h;

	//Initialize a frame manager
	FrameManager manager(global_params::map_filename);

	// create a std map of vehicles with integers as IDs
	map<int,VehicleInstance> vehicles;


	// variables to hold coefficients
	vector<double> jerk_coeffs;
	vector<double> jerk_coeffd;

	h.onMessage([&manager,&vehicles, &jerk_coeffs,&jerk_coeffd](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Ego Vehicle Information
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];

					// Previous path's Termination point
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// wrap all s values received
					end_path_s = wrap_s(end_path_s);
					car_s = wrap_s(car_s);

					// Sensor Fusion Data
					auto sensor_fusion = j[1]["sensor_fusion"];

					// Convert the speed to ms-1
					car_speed = convert_mph_to_ms(car_speed);


					// Get the lane we are on
					int car_lane = convert_d_to_lane(car_d);

					// Update Vehicle states based on sensor fusion update
					update_vehicles(car_speed, car_s, vehicles, sensor_fusion);


					// remember part of the previous path
					// create vector to input to json message
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// create trajectory states
					trajectory_state start_s, start_d;

					// flag to check if trajectory should be reset
					static bool reset_trajectory = true;


					// Keep only part of existing path
					// This helps in smoothing as well as reacting to changes in time
					if (previous_path_x.size() !=0 )
					{
						int n_to_keep = previous_path_x.size() > max_points_to_keep ? max_points_to_keep : previous_path_x.size() - 1;

						for (int i = 0; i <  n_to_keep; i++) {
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
						}

						// use the existing coefficients to seed the new control point
						double t;
						int trajectory_length = (int)(look_ahead_time_seconds/look_ahead_resolution_seconds);

						if (!reset_trajectory) {
							t = (trajectory_length - double(previous_path_x.size())) * look_ahead_resolution_seconds;
						}
						else {
							t = ((trajectory_length - double(previous_path_x.size())) + double(n_to_keep)) * look_ahead_resolution_seconds;
							reset_trajectory = false;
						}

						// Evaluate at time = t seconds
						start_s.v = wrap_s(eval_poly(jerk_coeffs, t));
						start_d.v = eval_poly(jerk_coeffd, t);

						// take first order derivatives
						vector<double> dcoeffs_s = derivative_poly(jerk_coeffs);
						vector<double> dcoeffs_d = derivative_poly(jerk_coeffd);

						// evaluate the derivatives
						start_s.v_dot = eval_poly(dcoeffs_s, t);
						start_d.v_dot = eval_poly(dcoeffs_d, t);

						//take second order derivatives
						vector <double> d2coeffs_s = derivative_poly(dcoeffs_s);
						vector <double> d2coeffs_d = derivative_poly(dcoeffs_d);

						// evaluate the derivatives
						start_s.v_double_dot = eval_poly(d2coeffs_s, t);
						start_d.v_double_dot = eval_poly(d2coeffs_d, t);
					}
					else {
						// initialize based on ego frenet and v
						start_s.v = car_s;
						start_s.v_dot = car_speed;
						start_s.v_double_dot = 0.0;

						start_d.v = car_d;
						start_d.v_dot = 0.0;
						start_d.v_double_dot = 0.0;

						// set reset trajectory flag to true
						reset_trajectory = true;
					}


					//Define time horizon in seconds
					double trajectory_look_ahead_seconds;

					// If some segment of the previous path is retained
					if (next_x_vals.size() > 1)
					{
						// reduce time for which the path has been retained from the simulation trajectory
						trajectory_look_ahead_seconds = look_ahead_time_seconds - double(next_x_vals.size() + 1) * look_ahead_resolution_seconds;
					}
					else
					{
						// else use the whole look ahead time
						trajectory_look_ahead_seconds = look_ahead_time_seconds;
					}


					// figure out lane action
					static int previous_end_lane = car_lane;
					int end_lane   = previous_end_lane;
					double min_cost = lethal_cost + 1;

					// If we have achieved our previous end(target)
					if (previous_end_lane == car_lane)
					{
						for (int lane_index=-1; lane_index < 2; lane_index++)
						{
							// Get the cost for lane change
							double cost = lane_action_cost(car_lane, car_s, car_d, lane_index, vehicles);

							if (debug)
								printf(" %d lane cost: %.04f \n", lane_index, cost);

							// Prefer to stay in the same lane if its cost is = minimum cost otherwise move to lane with minimum cost
							if (cost < min_cost || (cost == min_cost && lane_index == 0)) {
								min_cost = cost;
								end_lane = car_lane + lane_index;
							}
						}
						previous_end_lane = end_lane;
					}



					// Define End state
					// Select Target Vehicle in your own lane
					int target_vehicle_id = get_front_nearest_neighbor(end_lane, car_s, vehicles).id;

					// Acceleration
					double target_acc;

					// If there exists a target
					if (target_vehicle_id != -1)
					{
						// Project its state at the end of our trajectory
						VehicleInstance::state vstate = vehicles[target_vehicle_id].projected_state(trajectory_look_ahead_seconds);

						double goal_s = manager.xy_2_frenet(vstate.x,vstate.y, atan2(vstate.dy, vstate.dx))[0];

						// Keep some buffer clearance
						goal_s -= min_clearance;

						// if goal > start we should wrap our trajectory
						if ( goal_s < start_s.v )
						{
							goal_s += max_distance_to_track_meters;
						}

						// compute acceleration
						// using s = ut + 1/2at^2
						// a = 2(s-ut)/t^2
						target_acc = 2.0 * ( goal_s - start_s.v - start_s.v_dot * trajectory_look_ahead_seconds ) / (trajectory_look_ahead_seconds*trajectory_look_ahead_seconds);

						// compute acceleration to hit speed limit
						// This is pretty much only useful when target vehicle is violating speed limit
						double max_acc = (speed_limit_mps - start_s.v_dot) / trajectory_look_ahead_seconds;

						// Saturate target acc
						if ( max_acc < target_acc ) { target_acc = max_acc; }

					}
					else
					{
						// use the maximum permissible speed as target speed
						target_acc = (speed_limit_mps - start_s.v_dot) / trajectory_look_ahead_seconds;
					}
					// Saturate with global maximum permissible acceleration (rubric)
					if (target_acc > max_permissible_acceleration )
					{
						target_acc = max_permissible_acceleration;
					}

					// End Frenet states for the trajectory
					trajectory_state end_s, end_d;

					// Define final velocities and assume d2s/dt2 = 0.0 by the end.
					// using s = s0 + ut + 1/2at2
					// and v = u + at
					// a end = 0;
					end_s.v = start_s.v + start_s.v_dot * trajectory_look_ahead_seconds +  target_acc * trajectory_look_ahead_seconds * trajectory_look_ahead_seconds / 2.0;
					end_s.v = wrap_s(end_s.v);
					end_s.v_dot = start_s.v_dot + target_acc * trajectory_look_ahead_seconds;
					end_s.v_double_dot = 0.0;

					// Checking for transverse speed (accross lanes)
					double speed_d = (lane_controls::convert_lane_to_d(end_lane) - start_d.v) / trajectory_look_ahead_seconds;
					// cap the speed limit
					if (fabs(speed_d) >  transverse_speed_limit)
					{
						speed_d =  (fabs(speed_d) / speed_d) * transverse_speed_limit;
					}

					// Define target_ positions

					// Saturate with max permisssible values
					if (fabs(speed_d) <= transverse_speed_limit)
					{
						end_d.v_dot = speed_d;
					}
					else
					{
						end_d.v_dot = (fabs(speed_d) / speed_d) * transverse_speed_limit;
					}

					// using d = d0 + vt
					// (assuming a = 0)
					end_d.v = start_d.v + speed_d * trajectory_look_ahead_seconds;
					end_d.v_double_dot = 0.0;


					// Create path list by using jerk minimization polynomial
					// get coefficients based on current initial and final positions
					jerk_coeffs = get_jerk_coeff(start_s, end_s, trajectory_look_ahead_seconds);
					jerk_coeffd = get_jerk_coeff(start_d, end_d, trajectory_look_ahead_seconds);

					// Check collision by checking intersection while trying to shift lanes
					if (end_lane != car_lane)
					{
						// check if collision state is occuring
						while (is_collision(jerk_coeffs, jerk_coeffd, manager, vehicles, trajectory_look_ahead_seconds))
						{
							// Increase speed value
							speed_d += sgn(speed_d) * 0.2;

							// check till 3x speed limit
							if (fabs(speed_d) > transverse_speed_limit * 3.0)
							{
								break;
							}

							// using d = d0 + vt
							// (assuming a = 0)
							end_d.v = start_d.v + speed_d * trajectory_look_ahead_seconds;
							if (fabs(speed_d) >= transverse_speed_limit) {
								end_d.v_dot = speed_d;
							}
							else
							{
								// stop changing lane
								end_d.v_dot = 0.0;
							}

							end_d.v_double_dot = 0.0;
						}

						// New lane trajectory after updating d to avoid collision
						jerk_coeffd = get_jerk_coeff(start_d, end_d, trajectory_look_ahead_seconds);
					}


					// Saturate acceleration to maintain speed limit
					while (overspeeding(jerk_coeffs, jerk_coeffd, manager, vehicles, trajectory_look_ahead_seconds))
					{
						// lower acceleration
						target_acc -= 0.05;

						// define new target
						// using s = s0 + ut + 1/2at2
						// and v = u + at
						// a end = 0;
						end_s.v = start_s.v + start_s.v_dot * trajectory_look_ahead_seconds +  target_acc * trajectory_look_ahead_seconds * trajectory_look_ahead_seconds / 2.0;
						end_s.v_dot = start_s.v_dot + target_acc * trajectory_look_ahead_seconds;
						end_s.v_double_dot = 0.0;

						// Compute new coefficients
						jerk_coeffs = get_jerk_coeff(start_s, end_s, trajectory_look_ahead_seconds);
					}


					// convert the frenet coefficients output to x,y list
					for(double i = 0.0; i < trajectory_look_ahead_seconds; i+=look_ahead_resolution_seconds)
					{
						// create a frenet point based on jerk minimization polynomial
						double cur_s = wrap_s(eval_poly(jerk_coeffs, i));
						double cur_d = eval_poly(jerk_coeffd, i);

						// convert output into x,y co-ordinates
						vector<double> pos = manager.frenet_2_xy(cur_s, cur_d);

						// add to output list
						next_x_vals.push_back(pos[0]);
						next_y_vals.push_back(pos[1]);
					}

					json msgJson;

					// send message
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});


	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
			size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}











































































