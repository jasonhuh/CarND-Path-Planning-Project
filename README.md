# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

---

# Model Documentation

## Snapshot of the Youtube video

TBD


## Structure of the code


```cpp

set(sources 
    src/main.cpp
    src/prediction.cpp
    src/vehicle.cpp
    src/trajectory.cpp
    src/behavior.cpp
    src/cost.cpp
    src/util.cpp
    src/helpers.cpp
    )

```

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

The prediction module leverages the data from sensor fusion, detects other cars that are within the range of interest, and build predictions.

```cpp
// Using sensor fusion, build prediction.
for (int i = 0; i < sensor_fusion.size(); ++i) {
    vector<double> other = sensor_fusion[i];
    int other_id = (int)other[SF_ID_IDX];
    double other_s = other[SF_S_IDX];
    double other_d = other[SF_D_IDX];
    double other_v = sqrt(pow(other[SF_X_IDX],2) + pow(other[SF_Y_IDX],2));
    double other_sn = other_s + other_v * prev_size * tint;
    int other_l = get_lane(other_d);

    // Other car is within the range of interest.
    if ((other_l >= 0 && other_l < NUM_LANES) &&
        ((other_s - car_s) > -REAR_CAR_REACTION_RANGE)) {
        other[5] = other_sn;
        preds.push_back({(double)other_id,other_sn, other_d, other_v});
    }
}

```

### Behavior planner

The behavior planner is responsible for suggesting the states / maneuvers which are feasible, safe, legal and efficient. This module leverages a finite state machine and a set of cost functions to determine the next state to transition into.

![Behavior Planning](/images/behavior_planning.png)

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

Trajectory generation was inspired by the project walkthrough video in the course as well as the Python solution to the "Implement Behavior Planner in C++" quiz in the course.

[cf trajectory.cpp](src/trajectory.cpp)

```cpp
  // If previous path is almost empty, use the car as starting reference
  if (prev_size < 2) {
    // use two points that make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  // use the previous path's end point as starting reference
  else {

    // redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
```

In Frenet, add evenly 30 meter spaced points ahead of the starting reference. In the code below, `PLANNING_DISTANCE` is 30 meters, and `LANE_WIDTH` is 4 meters.

```cpp

  for (int i = 0; i < 3; ++i) {
    double target_s = ego->s + PLANNING_DISTANCE * (i + 1);
    if (target_s > MAX_S) {
      target_s -= MAX_S;
    }
    double target_d = ego->target_d;
    if (i == 0) {
      if (target_d - ego->d < - LANE_WIDTH) {
          target_d = ego->d - LANE_WIDTH;
      } else if (target_d - ego->d > LANE_WIDTH) {
          target_d = ego->d + LANE_WIDTH;
      }
    }

    vector<double> next_wp = getXY(target_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }

```


The spline library has been utilized for calculating y values as a function of x values. As provided in the walkthrough video, the last point of the vehicle and the point prior that was used were used as the initial two points. Three following points were also added to the spline. 


```cpp
  // create a spline
  tk::spline s;

  // set(x, y) points to the spline
  s.set_points(ptsx, ptsy);

  // Start with all of the previous path points from last time
  for (int i = 0; i < prev_size; ++i){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
```

### Conclusion

TBD


### Appendix - Units, Conversions

The following units and conversions were used in this project. Most of the units are defined in the [settings.h](src/settings.h) file.

- One meter per second is about 2.237 miles per hour.
- Width of a car lane: 4 meters


