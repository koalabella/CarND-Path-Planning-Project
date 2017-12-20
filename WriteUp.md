###README
####The Archetecture

This path planning project consist of 3 major modules:1) behavior planning; 2) prediction;3) trajectory planning. Sensor fusion is provided by the simulater.
Behavior planning module is implemented based on infinite state machine: for each possible next state, generate all possible trajectories, evaluate them by cost functions, and according to the costs, choose a best next state and it's corresponding trajectory.
Since the sensor only give information about current status, when plan behavior or trajectory in the comming time cycle, prection module is used estimate the status of moving vehicle and other vehicle around. 
Trajectory planning module is used to generate specific trajectory points(including velocity, accerleration, and jerk).
####2.Implementation

There are 7 added files: vehicle.cpp/h cost.cpp/h helper.cpp/h global.h.
global.h contains all the global variables, including weights of all cost functions and trajectory numbers in one time cycle.
helper.cpp/h contain some useful functions like getFrenet and getXY(copied from main.cpp).
cost.cpp/h contain all the cost functions, including effiency cost and buffer cost.
vehicle.cpp/h define the Vehicle class, which implementes behavior and trajectory plan.

In the main.cpp, I created a Vehicle instance, every time got current car information from sensor fusion, updating current status and used the last trajectory point generated in previous cycle as reference status. To handle the latency and smooth the trajectory, behavior and trajectory plan based on reference status.
And create new Vehicle instances every time got informations of vehicles around current vehicle, and used the predicted in time N(the time las previous trajectory point will be excuted) status as reference status.
After get information about current car and around cars, Vehicle.decide_next_state() will plan the best behavior and trajectory.

####behavior choosing process（Vehicle.decide_next_state(), vehicle.cpp 64L)
1) call Vehicle.sucessor()(vehicle.cpp 103L) based on current state, list all possible next status. For state "prepare change lane left", possible next status includes {"keep lane", "left lane change"}
2) for each possible next status, generate corresponding trajectory.
- a) if next status is "keep lane", call Vehicle.keep_lane_trajectory()(vehicle.cpp 156L). And need to decide target positions(3 points [s,d] the strategy would like to pass through) and proper speed(avoid collide yet still efficent) for the car. 
 - For proper speed, meet two conditions below would be ok:
		i)keep distance with the neareast vehicle ahead in current line and 
		ii)try to keep target speed.
 - For target positions, [current_s + 30, current_line_center], [current_s + 60, current_line_center], [current_s + 90, current_line_center] would be ok.
	
- b) if next status is "prepare lane change", call Vehicle.prep_lane_change_trajectory()(vehicle.cpp 180L), And also need to decide target positions and proper speed. 
 - For proper speed, meet these four conditions:
		i)keep distance with the neareast vehicle ahead in current line.
		ii) if there is a vehicle ahead in the target lane(preparing changing to), and the gap is no enough--then keep in the current lane and do nothing, or check behind cars.
		iii) if there is a vehicle behind in the target lane(preparing changing to), near the ego car--let it go and do nothing, far from the ego car--speed up and pass it(if not exceed the speed limit).
		iv)try to keep target speed.
 - For target positions, [current_s + 30, current_line_center], [current_s + 60, current_line_center], [current_s + 90, current_line_center] would be ok.
	
- c) if next status is "lane change", call Vehicle.lane_change_trajectory()(vehicle.cpp 232L), And also need to decide target positions and proper speed. 
 - For proper speed, change lane fast is enough.
 - For target positions, [current_s + 30 + 40, target_line_center], [current_s + 60 + 20, target_line_center], [current_s + 90 + 10, target_line_center] would be ok.

3) call Vehicle.trajectory_points()(vehicle.cpp ?line) to use these target positions generate spline for the trajectory path, and use spline and proper speed generate trajectory points.
4) calculate cost for each possible next state and its corresponding trajectory. Pick the minimun as best satate and return trajectory to main.

####4.spline

spline is used, it's behaves better than JMT. JMT influenced by stocastic disturb much.
####5.cost functions

There are 4 cost functions in cost.cpp: 1) efficiency cost function evaluate the bias from target speed; 2) buffer cost evaluates the closeness of current car and other cars. 3)full_lane_cost punish the car entering a crowded line. 4) change_lane_cost avoid unecessary lane change.