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

vector<vector<double>>  filter_predictions_by_s(
                        vector<vector<double>> predictions,
                        double s,
                        double range,
                        double t){

    vector<vector<double>> filtered;
    for (int i = 0; i < predictions.size(); ++i) {
        vector<double> ocar = predictions[i];
        double other_s = ocar[1];
        double other_v = ocar[3];
        other_s += other_v * t;
        if (fabs(ocar[1] - s) <= range) {
            filtered.push_back(ocar);
        }
    }
    return filtered;
}

vector<vector<double>>  filter_predictions_by_d_range(
                        vector<vector<double>> predictions,
                        double d1, double d2){
    double d_small, d_big;
    if (d1 < d2) {
        d_small = d1;
        d_big = d2;
    }
    else {
        d_small = d2;
        d_big = d1;
    }
    vector<vector<double>> filtered;
    for (int i = 0; i < predictions.size(); ++i) {
        vector<double> ocar = predictions[i];
        double other_d = ocar[2];
        if((other_d >= d_small - CAR_WIDTH) && (other_d <= d_big + CAR_WIDTH))
        {
            filtered.push_back(ocar);
        }
    }
    return filtered;
}

vector<vector<double>>  filter_predictions_by_d(
                        vector<vector<double>> predictions,
                        double d){
    return filter_predictions_by_d_range(predictions, d - CAR_WIDTH, d + CAR_WIDTH);
}

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

// Initializes Vehicle
vector<State> Vehicle::get_available_states(){
    vector<State> available_states;
    // 3.1. previous lane change still continues
    if (this->lane_changing) {
        // 3.1.1. accept new state (lane)
        available_states.push_back(KeepLane);
        // 3.1.2. turn back to previous state (lane) if not already passed to the target_lane
        if (fabs(this->d - this->target_d) > 2.5) {
            available_states.push_back(CancelChange);
        }
    }
    // 3.2. no lane change is taking place now
    else {
        // prevent lane change from 2 (current_lane) to 0 (target_lane)
        bool not_in_leftmost = this->target_lane > 0;
        bool not_in_rightmost = this->target_lane < NUM_LANES - 1;
        bool left_available = (!(this->target_lane < this->current_lane));
        bool right_available = (!(this->target_lane > this->current_lane));
        available_states.push_back(KeepLane);
        if (this->state == KeepLane || this->state == LaneChangeLeft || this->state == LaneChangeRight) {
            if ((not_in_leftmost) &&
                left_available) {
                available_states.push_back(PrepareLaneChangeLeft);
                available_states.push_back(LaneChangeLeft);
            }
            if ((not_in_rightmost) &&
                right_available) {
                available_states.push_back(PrepareLaneChangeRight);
                available_states.push_back(LaneChangeRight);
            }
        }
        else if(this->state == PrepareLaneChangeLeft && not_in_leftmost){
            if (left_available) {
                available_states.push_back(LaneChangeLeft);
            }
            available_states.push_back(PrepareLaneChangeLeft);
        }
        else if(this->state == PrepareLaneChangeRight && not_in_rightmost){
            if (right_available) {
                available_states.push_back(LaneChangeRight);
            }
            available_states.push_back(PrepareLaneChangeRight);
        }

    }
    return available_states;
}

string get_state_code(State state) {
    switch (state) {
        case KeepLane:
            return "KL";
        case CancelChange:
            return "Cancel";
        case Initial:
            return "Init";
        case PrepareLaneChangeLeft:
            return "PLCL";
        case PrepareLaneChangeRight:
            return "PLCR";
        case LaneChangeLeft:
            return "LCL";
        case LaneChangeRight:
            return "LCR";
        case Left:
            return "Left";
        case Right:
            return "Right";
        default:
            return "Unknown";
    }
}

void Vehicle::update_state(vector<vector<double>> predictions) {
    this->plcl_lcl = false;
    this->plcr_lcr = false;
    Vehicle::VehicleState to;

    to = this->get_next_state_recursive(predictions, to, PLANNING_HORIZON);

    std::cout<<this->step<<" states:";
    for(int i = 0; i < to.state_list.size(); ++i) {
        std::cout << " "<< get_state_code(to.state_list[i]) << " ";
    }
    std::cout<<"| cost: " << to.cost << "" << std::endl;

    this->state = to.state;
    this->a_list = to.a_list;
    this->a = this->a_list[0];
}

// collect TrajectoryData for inner steps (horizon = 1) and outer steps (horizon = 5)
Vehicle::VehicleState Vehicle::get_next_state_recursive(vector<vector<double>> predictions, Vehicle::VehicleState trajectory_object, int horizon) {
    VehicleState to;
    double d_const = 0.1;

    double vi = this->v;
    double si = this->s;
    double di = this->d;
    double df = this->target_d;

    // 0. find d and target_d d_dif
    double dd = df - di;
    double dd_abs = fabs(dd);


    // End of lane changeing
    if (this->lane_changing) {
        if (this->lane_change_finish) {
            this->lane_changing = false;
            this->lane_change_finish = false;
        }
        else if (dd_abs < d_const) {
            this->lane_change_finish = true;
        }
    }
    // 2. Filter near vehicles s0 +- PLANNING_DISTANCE
    vector<vector<double>> filtered_s = filter_predictions_by_s(predictions, si, (double)PLANNING_DISTANCE, this->time);

    // 3. get available states.
    Vehicle snapshot = this->copy_vehicle();
    vector<State> available_states = this->get_available_states();
    vector<Vehicle::VehicleState> to_list;

    vector<double> costs;
    for (auto st: available_states) {
        to = trajectory_object;
        this->state = st;
        to.state = st;
        to.trajectory.push_back(*this);
        to.state_list.push_back(st);

        this->realize_state(predictions);
        if(state == CancelChange) {
            to.cancel_count++;
        }
        if (this->target_lane != this->current_lane) {
            to.total_lane_changes++;
        }

        vector<vector<double>> filtered_s_d_range = filter_predictions_by_d_range(filtered_s, this->d, this->target_d);

        double d_int = 0.0;
        double ds = (double)PLANNING_DISTANCE;

        df = this->target_d;
        dd = df - di;
        // change d if target_d not reached
        d_int = dd / (double)PLANNING_DISTANCE;

        double sf = si + ds;
        double lowest_time = DESIRED_BUFFER;
        double closest_approach = CAR_LENGTH;
        while (this->s < sf ) {
            // during iteration, construct the helper_data, calculated nearest, lowest_time, and lowest_time_front and others
            // filters should be used, not all others should be considered, filter by s, filter by ...., filter by d,
            // it is assumed that others wont change d!!! this assumption may be used. with notification.
            StepObject so = this->acc_for_d(filtered_s_d_range);
            this->a = so.a;
            if (so.lowest_time < lowest_time) {
                lowest_time = so.lowest_time;
            }
            if (so.closest_approach < closest_approach) {
                closest_approach = so.closest_approach;
            }
            this->time += tpint;
            this->s += tpint * this->v;
            if (d_int != 0.0) {
                double d_step = tpint * this->v * d_int;

                if (fabs(this->d - df) <= fabs(d_step)) {
                    this->d = df;
                }
                else {
                    this->d += d_step;
                }
            }
            this->current_lane = get_lane(this->d);
            to.v_sum += this->v;
            to.step_count++;
            to.t_sum += tpint;
            to.a_list.push_back(this->a);
            this->v += tpint * this->a;
        }
        to.closest_approach_list.push_back(closest_approach);
        to.lowest_time_list.push_back(lowest_time);
        // occupied, lowest_time_front
        to.lowest_time_front_list.push_back(get_lowest_time_front(predictions));

        double cost = MAX_COST;
        if (closest_approach < (CAR_LENGTH / 2.0)) {
          to.trajectory.push_back(this->copy_vehicle());

          to.cost = MAX_COST;
          to_list.push_back(to);
        }
        else if (horizon > 1) {
            Vehicle::VehicleState child_to = this->get_next_state_recursive(predictions, to, horizon - 1);
            cost = child_to.cost;
            to_list.push_back(child_to);
        }
        else {
            to.trajectory.push_back(this->copy_vehicle());
            cost = calculate_cost(*this, to);
            to.cost = cost;
            to_list.push_back(to);
        }
        costs.push_back(cost);
        this->restore_vehicle(snapshot);
        if (cost < 5.0) {
            break;
        }
    }
    double min_cost = MAX_COST;
    for (int i = 0; i < costs.size(); ++i) {
        if (costs[i] < min_cost){
            min_cost = costs[i];
            to = to_list[i];
            to.state = available_states[i];
        }
    }
    return to;
}

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

void Vehicle::realize_state(vector<vector<double>> predictions) {

    // Given a state, realize it by adjusting acceleration and lane.
    // Note - lane changes happen instantaneously.

    auto state = this->state;
    if (state == CancelChange) {
        if (this->target_d < this->d) {
            realize_lane_change(Right);
        } else if (this->target_d > this->d) {
            realize_lane_change(Left);
        }
    } else if (state == LaneChangeLeft) {
        this->plcl_lcl = false;
        realize_lane_change(Left);
    } else if (state == LaneChangeRight) {
        this->plcr_lcr = false;
        realize_lane_change(Right);
    } else if (state == PrepareLaneChangeLeft) {
        this->plcl_lcl = true;
        realize_prep_lane_change(Left);
    } else if (state == PrepareLaneChangeRight) {
        this->plcr_lcr = true;
        realize_prep_lane_change(Right);
    }
}

// combined to return acc, buffer_cost (lowest,time), near_cost (closest_approach)
StepObject Vehicle::acc_for_d(vector<vector<double>> predictions) {
    StepObject so;
    double delta_v_til_target = this->target_speed - this->v;
    double max_acc = fmin(MAX_A, delta_v_til_target);

    double car_s = this->s;
    double car_d = this->d;
    double car_td = get_d(this->prep_lane);
    double car_v = this->v;
    double t_step = this->time;

    double min_s = 0.0;
    double min_s_dif = 100.0;
    double min_v = 0.0;
    int min_id = -1;

    double lowest_time = DESIRED_BUFFER;
    double closest_approach = CAR_LENGTH;

    // Filter by s and d-range
    double d_small, d_big;
    if (car_d < car_td) {
        d_small = car_d - CAR_WIDTH;
        d_big = car_td + CAR_WIDTH;
    }
    else {
        d_small = car_td - CAR_WIDTH;
        d_big = car_d + CAR_WIDTH;
    }
    for (int i = 0; i < predictions.size(); ++i) {
        vector<double> other = predictions[i];
        double other_s = other[1];
        double other_d = other[2];
        double other_v = other[3];
        other_s += other_v * t_step;
        if((other_d >= d_small) && (other_d <= d_big))
        {
            double s_dif = other_s - car_s;
            if (s_dif >= 0.0 && s_dif < min_s_dif) {
                min_s_dif = s_dif;
                min_s = other_s;
                min_v = other_v;
                min_id = (int)other[0];
            }
            if (fabs(other_d - car_d) <= CAR_WIDTH) {
                double dist_abs = fabs(other_s - car_s);
                double time_to = DESIRED_BUFFER;
                if (dist_abs <= CAR_LENGTH) {
                    time_to = 0.0;
                }
                else if (other_s > car_s) {
                    time_to = time_to_collision(other_s, car_s + CAR_LENGTH, other_v, car_v);
                }
                else {
                    time_to = time_to_collision(other_s + CAR_LENGTH, car_s, other_v, car_v);
                }

                // buffer lowest_time
                if (time_to >= 0.0) {
                    if (time_to < lowest_time){
                        lowest_time = time_to;
                    }
                }

                //near closest_approach
                if (dist_abs < closest_approach) {
                    closest_approach = dist_abs;
                }
            }
        }
    }

    // If there is a leading vehicle
    if (min_s_dif < 100.0) {
      double next_pos = min_s + (min_v * tpint);
      double my_next = car_s + (car_v * tpint);
      double separation_next = next_pos - my_next;
      double available_room = separation_next - this->preferred_buffer;
      max_acc = fmin(max_acc, available_room);
      max_acc = fmax(max_acc, - MAX_A);
    }

    so.a = max_acc;
    so.lowest_time = lowest_time;
    so.closest_approach = closest_approach;

    this->a = max_acc;

    return so;
}


double Vehicle::get_lowest_time_front(vector<vector<double>> predictions) {
    double lowest_time_front = TIME_DISTANCE;

    double ego_s = this->s;
    double ego_d = this->d;
    double ego_td = this->target_d;
    double ego_v = this->v;
    double t_step = this->time;

    for (int i = 0; i < predictions.size(); ++i) {
        vector<double> other = predictions[i];
        double other_s = other[1];
        double other_d = other[2];
        double other_v = other[3];
        double time_to = TIME_DISTANCE;
        other_s += other_v * t_step;
        if(fabs(other_d - ego_td) <= CAR_WIDTH && other_s >= ego_s) {
            time_to = time_to_collision(other_s, ego_s, other_v, MAX_V);

            if (time_to > 0.0 && time_to < lowest_time_front) {
                lowest_time_front = time_to;
            }
        }
    }
    return lowest_time_front;
}

void Vehicle::update_current_a(double time) {
    double t_step = (int)(ceil(time / tpint));
    this->a = this->a_list[t_step];
}

void Vehicle::realize_lane_change(State dir) {
    this->target_lane += (dir == Right ? 1 : -1);
    this->prep_lane = this->target_lane;
    this->target_d = get_d(this->target_lane);
    this->lane_changing = true;
}

void Vehicle::realize_prep_lane_change(State dir) {
    this->prep_lane = this->target_lane + (dir == Right ? 1 : -1);
}
