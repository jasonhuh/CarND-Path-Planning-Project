# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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


# Note

## Snapshot of the Youtube video

TBD


## Libaries

As the project walkthrough suggested, this project utilizes the `spline` library insead of polynomial trajectory generation as `spline` is simple to use.

- Spline
- Eigen (Already included in the basic project)


## Behavior control

![Behavior Control](/images/behavior_control.png)

The snippet of the `main.cpp` shows the implementation of the above architecture. By modularizing each component with specific responsibility such as prediction, behavior planning and trajectory generation, the current solution achieved separate of concerns and loose coupling as shown below.

```cpp

// 1. Generate prediction
auto preds = Prediction::generate_prediction(&ego, sensor_fusion, prev_size, 
                                    end_path_s, end_path_d, car_s, car_d);
ego.step++;

// 2. Use cost function to update the state machine
Behavior::update_state(&ego, preds);
Behavior::apply_state(&ego, preds);

// 3. Generate the final trajectory
auto traj = Trajectory::generate_trajectory(&ego, previous_path_x, previous_path_y, prev_size,
                car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y, map_waypoints_s);

// 4. Send message back to motion control
json msgJson = {
    {"next_x", traj.first},
    {"next_y", traj.second}
};

```          


### Predictions

For prediction, the project leverages the model based approach instead of data driven or hybrid approach because the scenarios for this project are not complex enough to 
leverage data drive approach or hybrid approach that leverages machine learning technique which requires large mount of training data to be effective.

[cf prediction.cpp](src/prediction.cpp)

### Behavior planner

The behavior planner is responsible for suggesting the states / maneuvers which are feasible, safe, legal and efficient. This module leverages a finite state machine and a set of cost functions to determine the next state to transition into.

[cf behavior.cpp](src/behavior.cpp)

#### Finite State Machine

The solution uses the finite statement machine to track the state of the vehicle. The states that are utilized in this project are show below.

#### States of Vehicle
![States of Vehicle](/images/states_of_vehicle.png)

The list of states is found in [vehicle.h](src/vehicle.h), and here is the snippet: 
```cpp

enum State {
    KeepLane,
    CancelChange,
    Initial,
    PrepareLaneChangeLeft,
    PrepareLaneChangeRight,
    LaneChangeLeft,
    LaneChangeRight,
    Left,
    Right
};

```

#### Trajectories cost ranking

For each trajectory, cost is calculated, and the trajectory with the lowest cost will be chosen.

[cf cost.cp](src/cost.cpp)


```cpp

const int COMFORT           = 10;
const int CENTER            = 10;
const int PREPARE_WO_CHANGE = 10;
const int CANCEL_COST       = 100;
const int OCCUPIED_LANE     = 500;
const int DANGER            = 1000;
const int NEAR              = 10000;
const int INEFFICIENCY      = 10000;
const double MAX_COST       = 1000000; // 1M


```

```cpp

double calculate_cost(Vehicle& vehicle,
                      Vehicle::VehicleState to){
    double cost = 0.0;
    cost += inefficiency_cost(to.v_sum, to.step_count, vehicle.target_speed);
    cost += near_cost(to.closest_approach_list);
    cost += occupied_cost(to.lowest_time_front_list);
    cost += buffer_cost(to.lowest_time_list);
    cost += change_lane_cost(to.total_lane_changes);
    cost += at_lane_cost(to.trajectory);
    cost += cancel_previous_cost(to.cancel_count);
    cost += prepare_without_change_cost(vehicle.plcl_lcl, vehicle.plcr_lcr);
    return cost;
}

```

### Trajectory generation

Trajectory generation was inspired by the project walk through in the course. 

[cf trajectory.cpp](src/trajectory.cpp)

### Appendix - Units, Conversions

The following units and conversions were used in this project. Most of the units are defined in the [settings.h](src/settings.h) file.

- One meter per second is about 2.237 miles per hour.
- Width of a car lane: 4 meters

# [Rubic Points](https://review.udacity.com/#!/rubrics/1971/view)

## Compilation

### The code compiles correctly.

The CMakeLists.txt has been extended to include new cpp files, and the code has been compiled 
and has been tested from both Mac OS X (10.13.6) and Linux (Ubuntu 16.04).

## Valid trajectories

### The car is able to drive at least 4.32 miles without incident.
The car was able to drive for 30 minutes without incident.

### The car drives according to the speed limit.
No speed limit red message was seen.

### Max Acceleration and Jerk are not Exceeded.
Max jerk red message was not seen.

### Car does not have collisions.
No collisions have occurred for the duration of 30 minutes.

### The car stays in its lane, except for the time between changing lanes.
The car stayed in its lane except when it needed to change lanes.

### The car is able to change lanes
The car was able to change lanes by itself when a slower car was ahead of the car.


### Reflection

Unit testing

Testing is a time consuming process. C++ code is more prone to break when refactoring. Solid unit testing would be desired.

---

