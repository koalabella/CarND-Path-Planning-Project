#ifndef GLOBAL_H
#define GLOBAL_H
#include <chrono>

#include <math.h>
#include <iostream>
#include <random>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

const double TIME_INTERVAL = 0.02; //time gap between 2 points send to simulater
const int TRAJECTORY_NUM = 50;  //total num of point send to simulater

const int N_SAMPLES = 1;   //samples of perturbed goals

//period of behavior, prediction, trajectory
const double BEHAV_CYCLE = 4.0;
const double PREDIC_CYCLE = 2.0;
const double TRAJECTORY_STEP = 0.5;

//infomation about destination and preffered speed
const double TARGET_SPEED = 49.5/2.24;
const double TARGET_ACC = 0.2;
const double TARGET_DEC = 0.15;
const int GOAL_LANE = 2;
const double GOAL_S = 6945.554;

//constraints of moving vehicle
const double MAX_ACC = 0.5;
const double MAX_VEL = 50.0/2.24;

//all the weights of cost function
const double EFFICIENCY = 20;
const double CLOSENESS = 60;
const double FULL_LANE = 5;
const double AVOID_LANE_CNG =1;

const double SECURITY_DISTANCE = 50.0;  //buffer we'd like to keep with front cars

const double VEHICLE_RADIUS = 2.5;

const int LANES_NUM = 3;
const int LANE_WIDTH = 4;

void init_global_parameters();

#endif