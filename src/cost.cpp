#include <functional>
#include <math.h>

#include "cost.h"
#include "global.h"
#include "helper.h"
#include "vehicle.h"

double calculate_costs(const Vehicle &vehicle,
                       const map<int, Vehicle> &round_vehicles,
                       const vector<vector<double>> &traj) {

    double cost = 0.0;

    //all cost functions and their corresponding weights.
    vector<std::function<double(const Vehicle & ,
                            const map<int, Vehicle> &,
                            const vector<vector<double>> &)>> cost_funcs = {inefficiency_cost,
                                                                            buffer_cost,
                                                                            full_lane_cost,
                                                                            change_lane_cost};
    vector<double> weight_list = {EFFICIENCY,
                                  CLOSENESS,
                                  FULL_LANE,
                                  AVOID_LANE_CNG};

    for (int i = 0; i < cost_funcs.size(); i++) {
        double new_cost = weight_list[i]*cost_funcs[i](vehicle,
                                                       round_vehicles,
                                                       traj);
        cost += new_cost;
    }

    return cost;

}

double inefficiency_cost(const Vehicle &vehicle,
                         const map<int, Vehicle> &round_vehicles,
                         const vector<vector<double>> &traj) {
    double v_intended_lane = vehicle.target_lane_speed(round_vehicles, vehicle.heading_lane());

    vector<double> ptxs = traj[0];
    vector<double> ptys = traj[1];
    double last_x = ptxs[ptxs.size() - 1];
    double pre_last_x = ptxs[ptxs.size() - 2];
    double last_y = ptys[ptys.size() - 1];
    double pre_last_y = ptys[ptys.size() - 2];

    double v_final_lane = distance(last_x, last_y, pre_last_x, pre_last_y)/TIME_INTERVAL; //velocity at the last point in this cycle

    double cost = fabs(2.0*TARGET_SPEED - v_intended_lane - v_final_lane)/TARGET_SPEED;

    return logistic(cost);
}

double buffer_cost(const Vehicle &vehicle,
                   const map<int, Vehicle> &round_vehicles,
                   const vector<vector<double>> &traj) {
    double nearest = vehicle.nearest_approach_to_any_vehicle(traj, round_vehicles);
    return logistic(2*VEHICLE_RADIUS / nearest);
}

double full_lane_cost(const Vehicle &vehicle,
                   const map<int, Vehicle> &round_vehicles,
                   const vector<vector<double>> &traj) {
    double distance = 0;
    for (auto it = round_vehicles.begin(); it != round_vehicles.end(); ++it) {
        Vehicle temp_vehicle = it->second;
        if (temp_vehicle.ref_lane == vehicle.heading_lane() && temp_vehicle.ref_s > vehicle.ref_s) {
            distance += 20.0/(temp_vehicle.ref_s - vehicle.ref_s);
        }
    }
    return logistic(distance);
}

double change_lane_cost(const Vehicle &vehicle,
                        const map<int, Vehicle> &round_vehicles,
                        const vector<vector<double>> &traj) {    //punish lane change with little advantage
    double cost = 0;
    if (vehicle.state == "LCR" || vehicle.state == "LCL")
        cost = 0.1;
    return cost;
}