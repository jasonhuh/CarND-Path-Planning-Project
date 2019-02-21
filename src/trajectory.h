#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <utility>
#include <vector>

class Vehicle;

using std::vector;

class Trajectory {
public:
    static std::pair< vector<double>, vector<double> > generate_trajectory(Vehicle *ego, vector<double> &previous_path_x, vector<double> &previous_path_y, 
                            std::size_t prev_size, double car_x, double car_y, double car_yaw,
                            vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s
                            );
};

#endif // TRAJECTORY_H