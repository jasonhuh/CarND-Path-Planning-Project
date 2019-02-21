#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <unordered_map>
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

class Vehicle {
public:

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

  State state;

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

  void configure(vector<int> road_data);

  void apply_lane_change(State direction);

  void apply_prep_lane_change(State direction);

  void update_current_a(double time);
};

#endif  // VEHICLE_H