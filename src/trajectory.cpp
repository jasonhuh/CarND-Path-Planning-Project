#include "trajectory.h"
#include <vector>
#include <utility>
#include "helpers.h"
#include "settings.h"
#include "vehicle.h"
#include "spline.h"

using std::vector;

std::pair< vector<double>, vector<double> > Trajectory::generate_trajectory(Vehicle *ego, vector<double> &previous_path_x, vector<double> &previous_path_y, 
                            std::size_t prev_size, double car_x, double car_y, double car_yaw,
                            vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s
                            ) {
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interoplate these waypoints with a spline and fill it in with more points that control sp ...
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y, yaw states
          // either we will reference the starting point as where the car is or at the previous path end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          double car_yaw_rad = deg2rad(car_yaw);

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

          // In Frenet add evenly 30m spaced points ahead of the starting reference.
          for (int i = 0; i < 3; ++i) {
            double target_s = ego->s + PLANNING_DISTANCE * (i + 1);
            if (target_s > MAX_S) {
              target_s -= MAX_S;
            }
            double target_d = ego->target_d;
            if (i == 0) {
              if (target_d - ego->d < - LANE_WIDTH) {
                  target_d = ego->d - LANE_WIDTH;
              }
              else if (target_d - ego->d > LANE_WIDTH) {
                  target_d = ego->d + LANE_WIDTH;
              }
            }

            vector<double> next_wp = getXY(target_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_wp[0]);
            ptsy.push_back(next_wp[1]);
          }

          // Convert to the cars coordinates like MPC project.
          for (int i = 0; i < ptsx.size(); ++i) {

            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            double a = cos(0 - ref_yaw);
            double b = sin(0 - ref_yaw);

            ptsx[i] = (shift_x * a - shift_y * b);
            ptsy[i] = (shift_x * b + shift_y * a);
          }

          // create a spline
          tk::spline s;

          // set(x, y) points to the spline
          s.set_points(ptsx, ptsy);

          // Start with all of the previous path points from last time
          for (int i = 0; i < prev_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = PLANNING_DISTANCE;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));

          double x_add_on = 0;

          for (int i = 1; i <= PLANNING_STEPS - prev_size; ++i) {
            ego->update_current_a(tint * i);
            if (ego->a > MAX_A) {
              ego->a = MAX_A;
            }
            else if (ego->a < - MAX_A) {
              ego->a = -MAX_A;
            }

            if (ego->target_speed - ego->v < 0.0) {
              ego->v += (ego->a * tint);
            }
            else if (ego->target_speed - ego->v > 0.0) {
              ego->v += (ego->a * tint);
            }
            if (ego->v > MAX_V) {
              ego->v = MAX_V;
            }

            double N = (target_dist/(tint * ego->v));
            double x_point = x_add_on + (target_x / N);
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //rotate back to normal after rotating it earlier
            double a = cos(ref_yaw);
            double b = sin(ref_yaw);

            x_point = (x_ref * a - y_ref * b);
            y_point = (x_ref * b + y_ref * a);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
        return std::make_pair(next_x_vals, next_y_vals);
}