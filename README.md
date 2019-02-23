# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Demo of the project (Youtube)

This is a screencast of the result for this project.

[![link to my video result](https://img.youtube.com/vi/K9E-zTFW750g/0.jpg)](https://www.youtube.com/watch?v=K9E-zTFW750)


---
# Model Documentation



## Structure of the project

This project modularizes code to different components to promote loose coupling and separate of concerns instead of having a monolithic code in `main.cpp`. This approach allows further extension of the code and unittest each component.

![Class Diagram](/images/class_diagram.png)

The `CMakeList.txt` has been modified accordingly so that the project can be compiled with files that have been created for the project:

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

- [Spline](https://kluge.in-chemnitz.de/opensource/spline/)
- [Eigen](http://eigen.tuxfamily.org)  (Already included in the basic project)


## Behavior control

At a high level, this project uses the behavior control architecture shown in the diagram below to enable automomous driving.

![Behavior Control](/images/behavior_control.png)

By modularizing each component with specific responsibility such as prediction, behavior planning and trajectory generation, the current solution achieved separate of concerns and loose coupling. The solution consists of 4 key steps:

1. Generate prediction
2. Behavior planner defines target candidates
3. Behavior planner calculates a cost for trajectories of each target candidate, and the trjectory with the lowest cost is chosen
4. Final trajectory is generated, and the trajectory information is sent to motion control

The snippet of the `main.cpp` shows the implementation of the above 4 key steps:

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

[cf prediction.cpp](src/prediction.cpp)

For prediction, the project leverages the model based approach instead of data driven or hybrid approach because the scenarios for this project are not complex enough to 
leverage data drive approach or hybrid approach that leverages machine learning technique which requires large mount of training data to be effective.

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

[cf behavior.cpp](src/behavior.cpp)

The behavior planner is responsible for suggesting the states / maneuvers which are feasible, safe, legal and efficient. This module leverages a finite state machine and a set of cost functions to determine the next state to transition into.

![Behavior Planning](/images/behavior_planning.png)



The solution uses the finite state machine to track the state of the vehicle. The states that are utilized in this project are show below:

- KeepLane (KL)
- CancelChange (CC)
- PrepareLaneChangeLeft (PLCL)
- PrepareLaneChangeRight (PLCR)
- LaneChangeLeft (LCL)
- LaneChangeRight (LCR)
- Left (Left)
- Right (Right)

The states can be transitioned into other states within the following constraint: 

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

### Trajectories cost ranking

[cf cost.cp](src/cost.cpp)

For each trajectory, cost is calculated, and the trajectory with the lowest cost will be chosen. The current project takes into account:

- Efficiency cost: Higher target speed is preferred.

- Safe distance cost: Minimum distance between predictions and trajectory over 1 second time horizon

- Lane cost: Staying in a lane is preferred over changing a lane. And a free lane without vehicle in Field Of View is preferred.


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
    cost += efficiency_cost(to.v_sum, to.step_count, vehicle.target_speed);
    cost += safe_distance_cost(to.closest_approach_list);
    cost += occupied_lane_cost(to.lowest_time_front_list);
    cost += buffer_cost(to.lowest_time_list);
    cost += change_lane_cost(to.total_lane_changes);
    cost += at_lane_cost(to.trajectory);
    cost += cancel_previous_cost(to.cancel_count);
    cost += prepare_without_change_cost(vehicle.plcl_lcl, vehicle.plcr_lcr);
    return cost;
}
```

### Trajectory generation

[cf trajectory.cpp](src/trajectory.cpp)

Trajectory generation was inspired by the project walkthrough video in the course as well as the Python solution to the "Implement Behavior Planner in C++" quiz in the course.



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

![Spline](/images/spline.jpg)

```cpp
  // create a spline
  tk::spline s;

  // set(x, y) points to the spline
  s.set_points(ptsx, ptsy);

  // ...

  // Obtain y value as a function of x value using spline
  double y_point = s(x_point);
```

### Conclusion

I would like to thank you Udacity and the instructors from Mercedes-Benz for proving easy-to-understand yet advanced course on the topic of path planning and providing the opportunity to build a
path planning project from ground-up. Also, I would like to thank Aaron
Brown from Udacity for distributing this great car simulator.


### Appendix - Units, Conversions

The following units and conversions were used in this project. Most of the units are defined in the [settings.h](src/settings.h) file.

- One meter per second is about 2.237 miles per hour.
- Width of a car lane: 4 meters


