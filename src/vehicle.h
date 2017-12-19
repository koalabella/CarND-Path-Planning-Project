#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <vector>
#include <string>

#include "spline.h"

using std::vector;
using std::map;
using std::string;

class Vehicle {
public:
    string state;   //state of vehicle

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

    vector<double> spline_x_pos;
    vector<double> spline_y_pos;

    //state of vehicle
    int cur_lane;
    double x, y;
    double vs, yaw;
    double s, d;

    int ref_lane;
    double ref_x, ref_y;
    double ref_s, ref_d;
    double ref_vx, ref_vy;
    double ref_yaw, ref_vs;

    int pred_num;

    Vehicle(string state="KL");
    void set_map(vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                      vector<double> map_waypoints_s);
    void set_current(double x, double y, double s, double d, double vs, double yaw, int pred_num);
    void set_ref(double ref_x, double ref_y, double ref_vx, double ref_vy);
    void set_rc_ref(double ref_s, double ref_d, double ref_vs);

    vector<vector<double>> decide_next_state(const map<int, Vehicle> & round_vehicles);
    double target_lane_speed(const map<int, Vehicle> & round_vehicles, int lane) const;
    vector<double> _lane_speed_section(const map<int, Vehicle> & round_vehicles, int lane) const;
    int heading_lane() const;
    bool get_vehicle_ahead(const map<int, Vehicle> & round_vehicles,int lane, Vehicle & rVehicle) const;
    bool get_vehicle_behind(const map<int, Vehicle> & round_vehicles,int lane, Vehicle & rVehicle) const;
    double nearest_approach_to_any_vehicle(const vector<vector<double>> &traj_points,
                                           const map<int, Vehicle> & round_vehicles) const;
    double nearest_approach(const vector<vector<double>> &traj_points,
                            const Vehicle & predict) const;
private:

    vector<string> sucessor_states();
    vector<vector<vector<double>>> get_goal_positions(const map<int, Vehicle> & round_vehicles, int lane);
    vector<vector<vector<double>>> gen_trajectory(string next_state,
                                                  const map<int, Vehicle> & round_vehicles);
    vector<vector<vector<double>>> keep_lane_trajectory(const map<int, Vehicle> & round_vehicles);
    vector<vector<vector<double>>> prep_lane_change_trajectory(string state, const map<int, Vehicle> & round_vehicles);
    vector<vector<vector<double>>> lane_change_trajectory(string state, const map<int, Vehicle> & round_vehicles);
    vector<vector<vector<double>>> trajectory_points(vector<vector<vector<double>>> goals_sets, double goal_vs);
    vector<double> car2global_cor(double x, double y);
    vector<double> gobal2car_cor(double x, double y);
};

#endif