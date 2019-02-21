#include "behavior.h"
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

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

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

vector<State> Behavior::get_available_states(Vehicle *ego) {
    vector<State> available_states;
    // 3.1. previous lane change still continues
    if (ego->lane_changing) {
        // 3.1.1. accept new state (lane)
        available_states.push_back(KeepLane);
        // 3.1.2. turn back to previous state (lane) if not already passed to the target_lane
        if (fabs(ego->d - ego->target_d) > 2.5) {
            available_states.push_back(CancelChange);
        }
    }
    // 3.2. no lane change is taking place now
    else {
        // prevent lane change from 2 (current_lane) to 0 (target_lane)
        bool not_in_leftmost = ego->target_lane > 0;
        bool not_in_rightmost = ego->target_lane < NUM_LANES - 1;
        bool left_available = (!(ego->target_lane < ego->current_lane));
        bool right_available = (!(ego->target_lane > ego->current_lane));
        available_states.push_back(KeepLane);
        if (ego->state == KeepLane || ego->state == LaneChangeLeft || ego->state == LaneChangeRight) {
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
        else if(ego->state == PrepareLaneChangeLeft && not_in_leftmost){
            if (left_available) {
                available_states.push_back(LaneChangeLeft);
            }
            available_states.push_back(PrepareLaneChangeLeft);
        }
        else if(ego->state == PrepareLaneChangeRight && not_in_rightmost){
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

string get_color_code(State state) {
    switch (state) {
        case KeepLane:
            return GREEN;
        case CancelChange:
            return BOLDRED;
        case Initial:
            return BOLDBLUE;
        case PrepareLaneChangeLeft:
            return BLUE;
        case PrepareLaneChangeRight:
            return RED;
        case LaneChangeLeft:
            return BOLDBLUE;
        case LaneChangeRight:
            return BOLDRED;
        case Left:
            return BLUE;
        case Right:
            return RED;
        default:
            return BOLDMAGENTA;
    }
}

string get_color_code(double cost) {
    if (cost < 10.0) {
        return BLUE;
    } else if (cost < 100.0) {
        return GREEN;
    } else if (cost < 1000.0) {
        return YELLOW;
    } else if (cost < 1500.0) {
        return RED;
    } else {
        return BOLDRED;
    }
}

void Behavior::update_state(Vehicle *ego, vector<vector<double>> predictions) {
    ego->plcl_lcl = false;
    ego->plcr_lcr = false;
    Behavior::VehicleState to;

    to = Behavior::get_next_state(ego, predictions, to, PLANNING_HORIZON);

    std::cout<< CYAN << ego->step << RESET << " states:";
    for(int i = 0; i < to.state_list.size(); ++i) {
        std::cout << " "<< get_color_code(to.state_list[i]) << get_state_code(to.state_list[i]) << RESET << " ";
    }
    std::cout<<"| cost: " << get_color_code(to.cost) << to.cost << RESET << "" << std::endl;

    ego->state = to.state;
    ego->a_list = to.a_list;
    ego->a = ego->a_list[0];
}

// collect VehicleState for inner steps (horizon = 1) and outer steps (horizon = 5)
Behavior::VehicleState Behavior::get_next_state(Vehicle *ego, vector<vector<double>> predictions, Behavior::VehicleState trajectory_object, int horizon) {
    VehicleState to;
    double d_const = 0.1;

    double vi = ego->v;
    double si = ego->s;
    double di = ego->d;
    double df = ego->target_d;

    // 0. find d and target_d d_dif
    double dd = df - di;
    double dd_abs = fabs(dd);


    // End of lane changeing
    if (ego->lane_changing) {
        if (ego->lane_change_finish) {
            ego->lane_changing = false;
            ego->lane_change_finish = false;
        }
        else if (dd_abs < d_const) {
            ego->lane_change_finish = true;
        }
    }
    // 2. Filter near vehicles s0 +- PLANNING_DISTANCE
    vector<vector<double>> filtered_s = filter_predictions_by_s(predictions, si, (double)PLANNING_DISTANCE, ego->time);

    // 3. get available states.
    Vehicle snapshot = ego->copy_vehicle();
    vector<State> available_states = Behavior::get_available_states(ego);
    vector<Behavior::VehicleState> to_list;

    vector<double> costs;
    for (auto st: available_states) {
        to = trajectory_object;
        ego->state = st;
        to.state = st;
        to.trajectory.push_back(*ego);
        to.state_list.push_back(st);

        Behavior::apply_state(ego, predictions);
        if (ego->state == CancelChange) {
            to.cancel_count++;
        }
        if (ego->target_lane != ego->current_lane) {
            to.total_lane_changes++;
        }

        vector<vector<double>> filtered_s_d_range = filter_predictions_by_d_range(filtered_s, ego->d, ego->target_d);

        double d_int = 0.0;
        double ds = (double)PLANNING_DISTANCE;

        df = ego->target_d;
        dd = df - di;
        // change d if target_d not reached
        d_int = dd / (double)PLANNING_DISTANCE;

        double sf = si + ds;
        double lowest_time = DESIRED_BUFFER;
        double closest_approach = CAR_LENGTH;
        while (ego->s < sf ) {
            // during iteration, construct the helper_data, calculated nearest, lowest_time, and lowest_time_front and others
            // filters should be used, not all others should be considered, filter by s, filter by ...., filter by d,
            // it is assumed that others wont change d!!! this assumption may be used. with notification.
            StepObject so = Behavior::acc_for_d(ego, filtered_s_d_range);
            ego->a = so.a;
            if (so.lowest_time < lowest_time) {
                lowest_time = so.lowest_time;
            }
            if (so.closest_approach < closest_approach) {
                closest_approach = so.closest_approach;
            }
            ego->time += tpint;
            ego->s += tpint * ego->v;
            if (d_int != 0.0) {
                double d_step = tpint * ego->v * d_int;

                if (fabs(ego->d - df) <= fabs(d_step)) {
                    ego->d = df;
                }
                else {
                    ego->d += d_step;
                }
            }
            ego->current_lane = get_lane(ego->d);
            to.v_sum += ego->v;
            to.step_count++;
            to.t_sum += tpint;
            to.a_list.push_back(ego->a);
            ego->v += tpint * ego->a;
        }
        to.closest_approach_list.push_back(closest_approach);
        to.lowest_time_list.push_back(lowest_time);
        // occupied, lowest_time_front
        to.lowest_time_front_list.push_back(get_lowest_time_front(ego, predictions));

        double cost = MAX_COST;
        if (closest_approach < (CAR_LENGTH / 2.0)) {
          to.trajectory.push_back(ego->copy_vehicle());

          to.cost = MAX_COST;
          to_list.push_back(to);
        }
        else if (horizon > 1) {
            Behavior::VehicleState child_to = Behavior::get_next_state(ego, predictions, to, horizon - 1);
            cost = child_to.cost;
            to_list.push_back(child_to);
        }
        else {
            to.trajectory.push_back(ego->copy_vehicle());
            cost = calculate_cost(*ego, to);
            to.cost = cost;
            to_list.push_back(to);
        }
        costs.push_back(cost);
        ego->restore_vehicle(snapshot);
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


void Behavior::apply_state(Vehicle *ego, vector<vector<double>> predictions) {

    // Given a state, realize it by adjusting acceleration and lane.
    // Note - lane changes happen instantaneously.

    auto state = ego->state;
    if (state == CancelChange) {
        if (ego->target_d < ego->d) {
            ego->apply_lane_change(Right);
        } else if (ego->target_d > ego->d) {
            ego->apply_lane_change(Left);
        }
    } else if (state == LaneChangeLeft) {
        ego->plcl_lcl = false;
        ego->apply_lane_change(Left);
    } else if (state == LaneChangeRight) {
        ego->plcr_lcr = false;
        ego->apply_lane_change(Right);
    } else if (state == PrepareLaneChangeLeft) {
        ego->plcl_lcl = true;
        ego->apply_prep_lane_change(Left);
    } else if (state == PrepareLaneChangeRight) {
        ego->plcr_lcr = true;
        ego->apply_prep_lane_change(Right);
    }

    ego->current_lane = get_lane(ego->d);
}

// combined to return acc, buffer_cost (lowest,time), near_cost (closest_approach)
StepObject Behavior::acc_for_d(Vehicle *ego, vector<vector<double>> predictions) {
    StepObject so;
    double delta_v_til_target = ego->target_speed - ego->v;
    double max_acc = fmin(MAX_A, delta_v_til_target);

    double car_s = ego->s;
    double car_d = ego->d;
    double car_td = get_d(ego->prep_lane);
    double car_v = ego->v;
    double t_step = ego->time;

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
      double available_room = separation_next - ego->preferred_buffer;
      max_acc = fmin(max_acc, available_room);
      max_acc = fmax(max_acc, - MAX_A);
    }

    so.a = max_acc;
    so.lowest_time = lowest_time;
    so.closest_approach = closest_approach;

    ego->a = max_acc;

    return so;
}


double Behavior::get_lowest_time_front(Vehicle *ego, vector<vector<double>> predictions) {
    double lowest_time_front = TIME_DISTANCE;

    double ego_s = ego->s;
    double ego_d = ego->d;
    double ego_td = ego->target_d;
    double ego_v = ego->v;
    double t_step = ego->time;

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