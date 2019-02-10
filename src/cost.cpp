#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include "settings.h"

using std::vector;

double time_to_collision(double other_s, double ego_s, double other_v, double car_v) {
    double s_dif = other_s - ego_s;
    double v_dif = other_v - car_v;
    double time_to = -1.0;
    if (v_dif != 0.0) {
        time_to = -s_dif / v_dif;
    }
    return time_to;
}

double cancel_previous_cost(int cancel_count){
    return cancel_count * CANCEL_COST;
}

double prepare_without_change_cost(bool plcl_lcr, bool plcr_lcr){
    double cost = 0.0;
    if (plcl_lcr) {
        cost++;
    }
    if (plcr_lcr) {
        cost++;
    }
    return cost * PREPARE_WO_CHANGE;
}

double inefficiency_cost(double speed_sum, int step_count, double target_speed){
    double avg_speed = speed_sum / (double)step_count;
    double diff = fabs(target_speed - avg_speed);
    double pct = diff / target_speed;
    double cost = 0.0;

    if (abs(diff) > 0.01) {
        cost += 20.0;
    }
    return cost + (pct * EFFICIENCY);
}

double near_cost(vector<double> closest_approach_list) {
    double cost = 0.0;
    for (auto closest_approach : closest_approach_list) {
        if (closest_approach < CAR_LENGTH) {
            double multiplier = 1.0 - pow(closest_approach / CAR_LENGTH,2);
            cost += multiplier * NEAR;
        }
    }
    return cost;
}

double occupied_cost(vector<double> lowest_time_front_list) {
    // lowest_time_front should be calculated with target velocity, not current vehicle velocity
    double cost = 0.0;
    for (auto lowest_time_front : lowest_time_front_list) {
      if (lowest_time_front <= 0.0) {
          cost += 1.0 * DANGER;
      }
      else if (lowest_time_front < TIME_DISTANCE) {
          double multiplier = 1.0 / lowest_time_front;
          cost += multiplier * OCCUPIED_LANE;
      }
    }
    return cost;
}

double buffer_cost(vector<double> lowest_time_list){
    double cost = 0.0;
    for (auto lowest_time : lowest_time_list) {
        if (lowest_time < DESIRED_BUFFER) {
          double multiplier = 1.0 - pow(lowest_time / DESIRED_BUFFER,2);
          cost += multiplier * DANGER;
      }
    }
    return cost;
}

// Add cost for each lane change
double change_lane_cost(int total_lane_changes){
    return COMFORT * total_lane_changes;
}

double at_lane_cost(vector<Vehicle> trajectory){
    double cost = 0.0;
    double center = double(NUM_LANES - 1) / 2.0;
    for (auto vehicle : trajectory) {
        double dif_center = abs((double)vehicle.current_lane - center);
        cost += dif_center * CENTER;
    }
    cost /= PLANNING_HORIZON;
    return cost;
}

double calculate_cost(Vehicle& vehicle,
                      Vehicle::TrajectoryObject to){
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