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

####3.spline

spline is used, it's behaves better than JMT. JMT influenced by stocastic disturb much.
####4.cost functions

There are x cost functions: 1) efficiency cost function evaluate the bias from target speed; 2) buffer cost evaluates the closeness of current car and other cars.