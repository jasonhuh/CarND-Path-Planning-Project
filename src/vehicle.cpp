#include "vehicle.h"
#include <iostream>
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include <cmath>
#include "util.h"
#include "settings.h"

using std::string;
using std::vector;

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, double s, double v, double a, double target_speed) {

    this->target_lane = lane;
    this->current_lane = lane;
    this->prep_lane = lane;
    this->target_d = get_d(lane);
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = Initial;
    this->step = 0;
    this->target_speed = target_speed;
    this->time = 0.0;
    this->lane_changing = false;
    this->lane_change_finish = false;
    this->plcr_lcr = false;
    this->plcl_lcl = false;
}

Vehicle::~Vehicle() {}

void Vehicle::restore_vehicle(Vehicle snapshot){
    this->target_lane = snapshot.target_lane;
    this->s = snapshot.s;
    this->v = snapshot.v;
    this->a = snapshot.a;
    this->state = snapshot.state;
    this->target_speed = snapshot.target_speed;
    this->d = snapshot.d;
    this->target_d = snapshot.target_d;
    this->current_lane = snapshot.current_lane;
    this->prep_lane = snapshot.prep_lane;
    this->time = snapshot.time;
    this->lane_changing = snapshot.lane_changing;
    this->lane_change_finish = snapshot.lane_change_finish;
    this->plcl_lcl = snapshot.plcl_lcl;
    this->plcr_lcr = snapshot.plcr_lcr;
}

Vehicle Vehicle::copy_vehicle(){
    Vehicle snapshot = Vehicle(this->target_lane,this->s,this->v,this->a,this->target_speed);
    snapshot.d = this->d;
    snapshot.target_d = this->target_d;
    snapshot.state = this->state;
    snapshot.current_lane = this->current_lane;
    snapshot.prep_lane = this->prep_lane;
    snapshot.time = this->time;
    snapshot.lane_changing = this->lane_changing;
    snapshot.lane_change_finish = this->lane_change_finish;
    snapshot.plcl_lcl = this->plcl_lcl;
    snapshot.plcr_lcr = this->plcr_lcr;
    return snapshot;
}

void Vehicle::update_current_a(double time) {
    double t_step = (int)(ceil(time / tpint));
    this->a = this->a_list[t_step];
}

void Vehicle::apply_lane_change(State dir) {
    this->target_lane += (dir == Right ? 1 : -1);
    this->prep_lane = this->target_lane;
    this->target_d = get_d(this->target_lane);
    this->lane_changing = true;
}

void Vehicle::apply_prep_lane_change(State dir) {
    this->prep_lane = this->target_lane + (dir == Right ? 1 : -1);
}
