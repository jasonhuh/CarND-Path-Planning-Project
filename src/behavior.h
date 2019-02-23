#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <map>
#include <unordered_map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

class Vehicle;

class Behavior {
public:
    struct VehicleState {
        vector<Vehicle> projection;
        vector<double> acc_list;
        double v_sum = 0;
        double t_sum = 0;
        int step_count = 0;
        int cancel_count = 0;
        vector<double> closest_approach_list;
        vector<double> lowest_time_list;
        vector<double> lowest_time_front_list;
        int total_lane_changes = 0;

        State state;
        vector<State> state_list;
        double cost;
    };    

    static vector<State> get_available_states(Vehicle *ego);

    static void update_state(Vehicle *ego, vector<vector<double>> sensor_fusion);

    static VehicleState get_next_state(Vehicle *ego, vector<vector<double>> predictions, VehicleState to, int horizon = 7);

    static void apply_state(Vehicle *ego, vector<vector<double>> predictions);

    static StepObject acc_for_d(Vehicle *ego, vector<vector<double>> predictions);

    static double get_lowest_time_front(Vehicle *ego, vector<vector<double>> predictions);
};

#endif  // BEHAVIOR_H