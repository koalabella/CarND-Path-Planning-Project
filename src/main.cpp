#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "global.h"
#include "helper.h"
#include "json.hpp"
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

extern vector<double> map_waypoints_x;
extern vector<double> map_waypoints_y;
extern vector<double> map_waypoints_s;
extern vector<double> map_waypoints_dx;
extern vector<double> map_waypoints_dy;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  Vehicle ego = Vehicle();
  ego.set_map(map_waypoints_x, map_waypoints_y, map_waypoints_s);

  h.onMessage([&ego,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            //cout << "current status: " << car_s << "," << car_d << "," << car_x << "," << car_y << "," << car_speed << endl;

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            //init ego car status in this cycle;
            ego.set_current(car_x, car_y, car_s, car_d, car_speed, car_yaw, TRAJECTORY_NUM - previous_path_x.size());

            vector<double> new_spline_x_pos;
            vector<double> new_spline_y_pos;

            double ref_x_pre, ref_y_pre, ref_x, ref_y, ref_vx, ref_vy;
            if (previous_path_x.size() > 1) {
              ref_x_pre = previous_path_x[previous_path_x.size() - 2];
              ref_y_pre = previous_path_y[previous_path_y.size() - 2];

              ref_x = previous_path_x[previous_path_x.size() - 1];
              ref_y = previous_path_y[previous_path_y.size() - 1];

            } else {
              ref_x_pre = car_x - car_speed * TIME_INTERVAL * cos(car_yaw) - 0.01;
              ref_y_pre = car_y - car_speed * TIME_INTERVAL * sin(car_yaw);

              ref_x = car_x;
              ref_y = car_y;

            }
            new_spline_x_pos.push_back(ref_x_pre);
            new_spline_y_pos.push_back(ref_y_pre);

            new_spline_x_pos.push_back(ref_x);
            new_spline_y_pos.push_back(ref_y);

            ego.spline_x_pos = new_spline_x_pos;
            ego.spline_y_pos = new_spline_y_pos;

            ref_vx = (ref_x - ref_x_pre)/TIME_INTERVAL;
            ref_vy = (ref_y - ref_y_pre)/TIME_INTERVAL;
            ego.set_ref(ref_x, ref_y, ref_vx, ref_vy);

            
            //get other cars on the road;
            map<int, Vehicle> round_vehicles;
            for (int i = 0; i < sensor_fusion.size(); i++) {
              vector<double> rc_data = sensor_fusion[i]; //sensor fusion [id, x, y, vx, vy, s, d]

              Vehicle road_car = Vehicle();
              //road_car.set_current();
              double rc_s = rc_data[5];
              double rc_d = rc_data[6];
              double rc_vx = rc_data[3];
              double rc_vy = rc_data[4];
              double rc_vs = sqrt(rc_vx*rc_vx + rc_vy*rc_vy);
              double rc_s_ref = rc_s + previous_path_x.size() * TIME_INTERVAL * rc_vs;
              road_car.set_rc_ref(rc_s_ref, rc_d, rc_vs);

              round_vehicles.insert(pair<int, Vehicle>(rc_data[0], road_car));
            }
            //cout << "round_vehicles" << endl;

            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            vector<vector<double>> next_points = ego.decide_next_state(round_vehicles);
            vector<double> x_points = next_points[0];
            vector<double> y_points = next_points[1];

            for (int i = 0; i < TRAJECTORY_NUM - previous_path_x.size(); i++) {
/*              vector<double> next_xy = getXY(x_points[i], y_points[i],
                                             map_waypoints_s, map_waypoints_x, map_waypoints_y);*/
              next_x_vals.push_back(x_points[i]);
              next_y_vals.push_back(y_points[i]);
              //cout << "new points: " << x_points[i] << "," << y_points[i] << endl;
            }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
