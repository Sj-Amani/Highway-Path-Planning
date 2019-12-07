# Highway Path Planning

![Highway_Path_Planning_10X_gif](results/Highway_Path_Planning_10X.gif)

## Overview
In this project I will `Design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic by using localization, sensor fusion, and map data`. The main codes are in the "src/" directory.

## Project Introduction   
### Goals
In this project my goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data has been provided. There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Simulator.
You can download the Udacity Term3 Simulator for this project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```


## How to Build and Run this project

1. Clone this repo.
2. Install uWebSockets: `install-ubuntu.sh` for case of ubuntu
3. Run the simulator: `./term3_sim.x86_64`
4. Compile and Run the path planning: `./run`

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Model Explantion For Generating Paths

### Prediction
The prediction module uses a map and data from sensor fusion to generate predictions for what all other dynamic objects in view are likely to do. 
For example, if the front/back car is changing lane or braking. 

### Behavior Planning
In this part, we decide what is best to do. It means both safety and efficiency. On the safty side, the car should be keep a distance from other cars, no matter it's keeping lane or changing lane. For efficiency, the car should stay in a lane where the traffic flows fast enough.
The car behave has five states: 
1.staying in the left lane 
2.the middle lane, 
3.the right lane, 
4.left lane change, 
5.right lane change.

Not all states can transition to all other states directly. Apparently, there's no way to make a left lane change if it's aleady in the left most lane; otherwise, it will have to cross the double yellow line and that's very dangerous. Also, if it's in the left most lane, it's not possible to be in the right most lane in the next time step.
In my implementation, the car constantly check the (potential) time to catch up with the cars in front for all the lanes. If a left lane change or right lane change can potentially increase the time, and it's safe to do so, then it will make a lane change. The creteria for safety is defined as "no car in parallel positions".

### Path Generation
This part deals with the trajectory calculation based on the speed and lane output from the behavior, car coordinates and past path points. The trajectories generated should be smooth enough, otherwise the jerk will exceed the maximum allowed value. I used the points left in the last time step and a couple of new points obtained from behavior planning as pivots. These pivots are input into the "spline" lib to generate a smooth trajectory. Also, I used coordinates transformation to make the work easier. The transformation was applied both between car fixed coordinate system and world coordinate system, and between world coordinate system and Frenet coordinate system (Please see the top video).

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

References:
---
https://classroom.udacity.com/nanodegrees

http://kluge.in-chemnitz.de/opensource/spline/

https://github.com/nlohmann/json

https://github.com/udacity/CarND-Path-Planning-Project

https://markbroerkens.github.io/CarND-Path-Planning-Project/

https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2

How Referencing This Project
---
If you like my code and you want to use it in your project, please refer it like this:

`Amani, Sajjad. "Practical highway path planning of a vehicle in Traffic using localization, sensor fusion, and map data." GitHub, 8 December 2019, https://github.com/Sj-Amani/Highway-Path-Planning`


