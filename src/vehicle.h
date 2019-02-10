#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;


struct StepObject{
    double a;
    double lowest_time;
    double closest_approach;
};


class Vehicle {
public:
    struct TrajectoryObject{
        vector<Vehicle> trajectory;
        vector<double> a_list;
        double v_sum = 0;
        double t_sum = 0;
        int step_count = 0;
        int cancel_count = 0;
        vector<double> closest_approach_list;
        vector<double> lowest_time_list;
        vector<double> lowest_time_front_list;
        int total_lane_changes = 0;

        string state;
        vector<string> state_list;
        double cost;
    };


  double preferred_buffer = 13.0; // impacts "keep lane" behavior.

  int prep_lane;

  int target_lane;

  int current_lane;

  double s;

  double d;

  double target_d;

  double time;

  double v;

  double a;

  vector<double> a_list;

  double target_speed;

  string state;

  bool lane_changing;
  bool lane_change_finish;

  bool plcl_lcl;
  bool plcr_lcr;

  int step;


  /**
  * Constructor
  */
  Vehicle(int lane, double s, double v, double a, double target_speed);


  /**
  * Destructor
  */
  virtual ~Vehicle();

  Vehicle copy_vehicle();

  void restore_vehicle(Vehicle snapshot);

  vector<string> get_available_states();

  void update_state(vector<vector<double>> sensor_fusion);

  TrajectoryObject get_next_state_recursive(vector<vector<double>> predictions, TrajectoryObject to, int horizon = 5);

  void configure(vector<int> road_data);

  void realize_state(vector<vector<double>> predictions);

  void realize_lane_change(string direction);

  void realize_prep_lane_change(string direction);

  StepObject acc_for_d(vector<vector<double>> predictions);

  double get_lowest_time_front(vector<vector<double>> predictions);

  void update_current_a(double time);
};

#endif  // VEHICLE_H