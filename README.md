# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
In this project the goal was to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 

#### Information Provided 
* Sensor Fusion
* Localization 
* Sparse list of waypoints

#### Objectives 
* The car should try to go as close as possible to the 50 MPH speed limit 
* The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times
* The car should be able to make one complete loop around the 6946m highway. 
* Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


## Submission details

### Overview 

This was a thoroughly challenging project. The following subsections describe my efforts at arriving at a minimum viable solution.

#### Waypoint interpolation
The provided list of sparse waypoints in the csv file are used to create 4 spline definitions and not store the dense waypoints in RAM. These spline definitions are used at every iteration to create the dense waypoints list. This reduces memory usage and by our definition we can directly look up x,y,dx,dy values from the {s,d} frenet pair.

#### Co-ordinate frame management
An inherent requirement of the project is the management of both the frenet as well as the x,y frames simultaneously
Implementation of this module was part of the frame_management namespace and specifically done in the frame_management Class
which contains the spline models as well as the sparse waypoint lists

The spline models used are
* s wrt x
* s wrt y
* s wrt dx
* s wrt dy

This reduces the sparse waypoint lookup to 
x = sx(s)
y = sx(y)

and dense waypoint lookup to 
x += sdx(s) * d
y += sdy(s) * d


#### Tracking sensor fusion objects
This is an important step of the solution since it is using the output of the tracking module which projects the target vehicles' state temporally forward, that is used to determine collisions, path costs and lane navigation behaviour. A Vehicle Instance class was used to represent the state of each vehicle, This class provides methods to set and get the current state of the vehicle, project the state forward in time. This class also provides method to set and get whether a vehicle is marked as relevant(active) or not based on our current trajectory. Only active vehicles are projected and checked for collision.

** Note: Since Vehicles are only one type of object, Vehicle instance class and utility functions were implemented inside an objects workspace which I plan on developing further.

#### Lane Navigation
The car at any point of time t can take one of the following sets of actions
* Stay in lane and maintain speed
* Stay in lane and accelerate
* Stay in lane and decelerate
* Switch to left lane
* Switch to right lane

It is to be noted that any behaviour of changing lane and acceleration/deceleration is not really simultaneous but instantaneously sequential. 
** Note: The methods for lane navigation are implemented as part of the lane_controls workspace. As future work I would like to add more lane behaviours as well as a FSM to this namespace


#### Action Costs

* Going left from leftmost lane or right from rightmost lane carries a lethal cost
* Any action that results in collision carries a lethal cost ( Any state where a minimum clearance in front is violated is also considered as collision)
* For all non lethal actions costs are directly proportional to speed, inversely proportional to distance and directly proportional to amount of horizontal (transverse) distance to move. The constants for the costs were determined heuristically/empirically till I could drive 1 Lap. 

** Note: I have still some issues with the wrapping of the path around the track. I'm not sure which is the section where i missed the wrapping call. Still trying to figure this out

#### Path fusion
For the purposes of smoothness as well as for reaction to environment/vehicle changes, we only retain a part of the previous path and append a new path to it while maintaining a constant message length (with a +-1 tolerance). The length of the path segment to retain across iterations and the total lookahead time are tuned empirically.

If we retain too long a segment, the car can't react to any changes. 
If we retain too short a segment, motion smoothness is compromised

** Issues observed: Around 3.07 miles if I am in the right most lane, The simulator records an outside lane incident even though visually all 4 wheels are inside the lane markings


#### Jerk minimization
The jerk minimization function is a Householder rank-revealing QR decomposition of a matrix with column-pivoting of a 5th order polynomial. The input vector is a difference between the target position as specified and the projected target using linear motion from the starting position. The solver approximates the coefficients for smoothest change. The coefficients are retained across iterations for seeding the start position on the next cycle.


#### Dense path generation 
The smooth waypoints are generated using the interpolated spline equations described in section on frame management

#### Future work
* Try to integrate with the deep learning pipeline on object detection on real world data
* Experiment and integrate with GTAVisionExport to try and navigate on GTA Highway 
* figure out the wraparound problem



## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build` (Do this as a sibling of the project directory, not inside the source tree)
3. Compile: `cmake ../CarND-P11-Path-Planning && make`
4. Run it: `./path_planning`.



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
