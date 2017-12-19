#ifndef COST_H
#define COST_H

#include <vector>

#include "vehicle.h"
#include "spline.h"

double calculate_costs(const Vehicle &vehicle,
					   const map<int, Vehicle> &round_vehicles,
					   const vector<vector<double>> & traj);

double goal_distance_cost(const Vehicle &vehicle,
					  	  const map<int, Vehicle> &round_vehicles,
					  	  const vector<vector<double>> & traj);

double inefficiency_cost(const Vehicle &vehicle,
						 const map<int, Vehicle> &round_vehicles,
						 const vector<vector<double>> & traj);

double buffer_cost(const Vehicle &vehicle,
				  const map<int, Vehicle> &round_vehicles,
				  const vector<vector<double>> & traj);

double full_lane_cost(const Vehicle &vehicle,
                   const map<int, Vehicle> &round_vehicles,
                   const vector<vector<double>> &traj);

double change_lane_cost(const Vehicle &vehicle,
                   const map<int, Vehicle> &round_vehicles,
                   const vector<vector<double>> &traj);

#endif