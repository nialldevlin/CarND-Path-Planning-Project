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
  double tgt_vel = 49;  //MPH

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          int prev_path_size = previous_path_x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          vector<double> x_points;
          vector<double> y_points;

          double pos_x = car_x;
          double pos_y = car_y;
          double pos_t = deg2rad(car_yaw);

          if (prev_path_size < 2) {
            x_points.push_back(car_x - cos(deg2rad(car_yaw)));
            y_points.push_back(car_y - sin(deg2rad(car_yaw)));

            x_points.push_back(car_x);
            y_points.push_back(car_y);

          } else {
            pos_x = previous_path_x[prev_path_size - 1];
            pos_y = previous_path_y[prev_path_size - 1];

            double prev_x = previous_path_x[prev_path_size - 2];
            double prev_y = previous_path_y[prev_path_size - 2];

            pos_t = atan2(pos_y - prev_y, pos_x - prev_x);

            x_points.push_back(prev_X);
            y_points.push_back(prev_y);

            x_points.push_back(pos_x);
            x_points.push_back(pos_y);
          }

          int increment = 30;
          int wp_ahead = 3;
          int l_m = 2 + 4 * tgt_lane;

          vector<double> wp;

          for (int i = 0; i < wp_ahead; i++) {
            wp = getXY(car_s + increment * (i + 1), l_m, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            x_points.push_back(wp[0]);
            y_points.push_back(wp[1]);
          }

          for (int i = 0; i < x_points.size(); i++) {
            double s_x = x_points[i] - pos_x;
            double s_y = y_points[i] - pos_y;

            x_points[i] = s_x * cos(0 - pos_t) - s_y * sin(0 - pos_t);
            x_points[i] = s_x * sin(0 - pos_t) + s_y * cos(0 - pos_t);
          }

          tk::spline s;

          s.set_points(x_points, y_points);

          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double tgt_x = 30;
          double tgt_y = s(tgt_x);
          double tgt_d = dist(0, 0, tgt_x, tgt_y);

          double x_p = 0;
          double y_p = 0;

          int num_points = 50;
          for (int i = 0; i < num_points - previous_path_x.size(); i++) {
            double N = tgt_d / (0.02 * ref_vel / 2.24);
            x_p += tgt_x / N;
            y_p = s(x_p);

            x_p = x_p * cos(pos_t) - y_p * sin(pos_t);
            y_p = x_p * sin(pos_t) + y_p * cos(pos_t);

            next_x_vals.push_back(x_p);
            next_y_vals.push_back(y_p);
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
