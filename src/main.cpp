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
  double max_s = 6945.554;

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

  int tgt_lane = 1;     //0 - closest to yellow line, 2 farthest from yellow line
  double vel = 0;  //MPH
  double max_vel = 49.5;
  double tgt_vel = max_vel;
  double max_acc = 0.224;

  h.onMessage([&tgt_lane, &vel, &tgt_vel, &max_vel, &max_acc,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          //Test if we need to pass, and which side
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;

          //Speed of car ahead, for merging behind a car
          double car_left_ahead = -1;
          double car_right_ahead = -1;


          //Behavior Control
          for (int i = 0; i < sensor_fusion.size(); i++) {

            //Define vehicle x and y velocity
            double veh_x = sensor_fusion[i][3];
            double veh_y = sensor_fusion[i][4];
            //Vehicle velocity
            double vehicle_vel = sqrt(veh_x * veh_x + veh_y * veh_y);

            //Vehicle position
            double veh_s = sensor_fusion[i][5];
            double veh_d = sensor_fusion[i][6];
            veh_s += (double)prev_size * 0.02 * vehicle_vel;

            //Check if car is ahead of us in our lane
            if ( veh_s >= car_s && (veh_s - car_s) <= 30 && veh_d > (4 * tgt_lane) && veh_d < (4 + 4 * tgt_lane) ) {
              car_ahead = true;
              //Slow down to match speed ahead
              tgt_vel = vehicle_vel;
            }

            //Check if car is blocking us from changing lanes
            if ( (veh_s - car_s) >= -10 &&
                 (veh_s - car_s) <= 20 &&
                 veh_d >= (4 * (tgt_lane - 1)) &&
                 veh_d <= (4 + 4 * (tgt_lane - 1)) && veh_d > 0) {
              car_left = true;
            }

            if ( (veh_s - car_s) >= -10 &&
                 (veh_s - car_s) <= 20 &&
                 veh_d >= (4 * (tgt_lane + 1)) &&
                 veh_d <= (4 + 4 * (tgt_lane + 1)) && veh_d > 0) {
              car_right = true;
            }

            //Check if car is in position to merge behind it
            if ( (veh_s - car_s) > 20 &&
                 (veh_s - car_s) <= 50 &&
                 veh_d >= (4 * (tgt_lane - 1)) &&
                 veh_d <= (4 + 4 * (tgt_lane - 1)) && veh_d > 0) {
              car_left_ahead = vehicle_vel;
            } else {
              car_left_ahead = -1;
            }

            if ( (veh_s - car_s) > 20 &&
                 (veh_s - car_s) <= 50 &&
                 veh_d >= (4 * (tgt_lane + 1)) &&
                 veh_d <= (4 + 4 * (tgt_lane + 1)) && veh_d > 0) {
              car_right_ahead = vehicle_vel;
            } else {
              car_right_ahead = -1;
            }

          }

          //Check if we need to pass
          if (car_ahead) {
            //Check if we can pass left
            if (!car_left && tgt_lane != 0) {
              //Check if there is a car we can merge behind
              if (car_left_ahead == -1) {
                tgt_vel = max_vel;
              } else {
                tgt_vel = car_left_ahead;
              }
              tgt_lane -= 1;
              //Check if we can pass right
            } else if (!car_right && tgt_lane != 2) {
              //Check to merge behind
              if (car_right_ahead == -1) {
                tgt_vel = max_vel;
              } else {
                tgt_vel = car_right_ahead;
              }
              tgt_lane += 1;
            }
          } else {    //Get in center lane if no car ahead of us
            if (tgt_lane != 1) {
              if ( (tgt_lane == 0 && !car_right) || (tgt_lane == 2 && !car_left)) {
                tgt_lane = 1;
              }
            }
            tgt_vel = max_vel;
          }

          //std::cout <<  " ahead: " << car_ahead << " lane: " << tgt_lane << std::endl;

          //Speed Control
          if (vel <= (tgt_vel - max_acc / 2)) {
            vel += max_acc;
          } else if (vel >= (tgt_vel + max_acc / 2)) {
            vel -= max_acc;
          } else {
            vel = tgt_vel;
          }

          //Failsafe
          if (vel > max_vel) {
            vel = max_vel;
          }


          //Points for spline
          //This code comes from the Q & A video
          vector<double> x_points;
          vector<double> y_points;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          //Make points behind us when we start to create a smooth spline
          if (prev_size < 2) {
            x_points.push_back(car_x - cos(deg2rad(car_yaw)));
            y_points.push_back(car_y - sin(deg2rad(car_yaw)));

            x_points.push_back(car_x);
            y_points.push_back(car_y);

          } else { //Use previous points to smooth spline
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            x_points.push_back(ref_x_prev);
            y_points.push_back(ref_y_prev);

            x_points.push_back(ref_x);
            y_points.push_back(ref_y);
          }

          // Create spaced out waypoints where we want to go
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * tgt_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * tgt_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * tgt_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          x_points.push_back(next_wp0[0]);
          x_points.push_back(next_wp1[0]);
          x_points.push_back(next_wp2[0]);

          y_points.push_back(next_wp0[1]);
          y_points.push_back(next_wp1[1]);
          y_points.push_back(next_wp2[1]);

          //Shift into cars reference frame
          for (int i = 0; i < x_points.size(); i++) {
            double shift_x = x_points[i] - ref_x;
            double shift_y = y_points[i] - ref_y;

            x_points[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            y_points[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          //create spline
          tk::spline s;

          s.set_points(x_points, y_points);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //Populate next points with previous points so we dont have to recalculate everything everytime

          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }          

          //Beginning os speed calculations
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          int num_points = 50;

          //Create new points at the end of the spline
          for (int i = 0; i <= num_points - previous_path_x.size(); i++) {
            //Calculate how far apart to match speed
            double N = target_dist / (0.02 * vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //Back in global reference frame
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
	  
          //END TODO

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
