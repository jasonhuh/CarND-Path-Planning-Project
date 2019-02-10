#ifndef COST_H
#define COST_H

#include "vehicle.h"
#include <cmath>
#include "settings.h"

using std::map;
using std::string;
using std::vector;

double time_to_collision(double other_s, double ego_s, double other_v, double ego_v);

double cancel_previous_cost(int cancel_count);

double prepare_without_change_cost(bool plcl_lcr, bool plcr_lcr);

double inefficiency_cost(double speed_sum, int step_count, double target_speed);

double near_cost(vector<double> closest_approach_list);

double occupied_cost(vector<double> lowest_time_front_list);

double buffer_cost(vector<double> lowest_time_list);

// Add cost for each lane change
double change_lane_cost(int total_lane_changes);

double at_lane_cost(vector<Vehicle> trajectory);

double calculate_cost(Vehicle& vehicle,
                      Vehicle::TrajectoryObject to);
#endif  // COST_H