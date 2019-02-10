#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "settings.h"
#include "util.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  // Ego vehicle
  Vehicle ego = Vehicle(1, 0.0, 0.0, 0.0, MAX_V);  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          auto prev_size = previous_path_x.size();

          // Use sensor fusion
          car_speed = miles_per_hour_to_meters_per_second(car_speed);

          if (prev_size > 0) {
            ego.s = end_path_s;
            ego.d = end_path_d;
            if (ego.d < 1.0) {
              ego.d = 1.0;
            }
            else if (ego.d > (LANE_WIDTH * NUM_LANES) - 1.0) {
              ego.d = (LANE_WIDTH * NUM_LANES) - 1.0;
            }

            vector<vector<double>> filtered_sensor_fusion;
            for (int i = 0; i < sensor_fusion.size(); ++i) {
              vector<double> other = sensor_fusion[i];
              int other_id = (int)other[0];
              double other_s = other[5];
              double other_d = other[6];
              double other_v = sqrt(pow(other[3],2) + pow(other[4],2));
              double other_sn = other_s + other_v * prev_size * tint;
              int other_l = get_lane(other_d);

              if ((other_l < 0 || other_l >= NUM_LANES) ||
                  ((other_s - car_s) <= -70.0)) {
              }
              else {
                  other[5] = other_sn;
                  filtered_sensor_fusion.push_back({(double)other_id,other_sn, other_d, other_v});
              }
            }
            sensor_fusion = filtered_sensor_fusion;
          }
          else {
            ego.s = car_s;
            ego.d = car_d;
          }
          ego.step++;
          ego.update_state(sensor_fusion);
          ego.realize_state(sensor_fusion);
          ego.current_lane = get_lane(ego.d);

          
          json msgJson;

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
          for (int i = 0; i < 3; ++i){
            double target_s = ego.s + PLANNING_DISTANCE * (i + 1);
            if (target_s > MAX_S) {
              target_s -= MAX_S;
            }
            double target_d = ego.target_d;
            if (i == 0) {
              if (target_d - ego.d < - LANE_WIDTH) {
                  target_d = ego.d - LANE_WIDTH;
              }
              else if (target_d - ego.d > LANE_WIDTH) {
                  target_d = ego.d + LANE_WIDTH;
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
            ego.update_current_a(tint * i);
            if (ego.a > MAX_A) {
              ego.a = MAX_A;
            }
            else if (ego.a < - MAX_A) {
              ego.a = -MAX_A;
            }

            if (ego.target_speed - ego.v < 0.0) {
              ego.v += (ego.a * tint);
            }
            else if (ego.target_speed - ego.v > 0.0) {
              ego.v += (ego.a * tint);
            }
            if (ego.v > MAX_V) {
              ego.v = MAX_V;
            }

            double N = (target_dist/(tint * ego.v));
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

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}