# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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



---

## How to generate the paths in details.
In this section, I will explain how to generate paths in details. 

* At first, we indentify which lane the car is currently now.

```cpp
// Obtain distance of car to the reference in the road according to sensor data.
float d = sensor_fusion[i][6];

// Identify the lane of the car in question
int car_lane;
if (d >= 0 && d < 4) {
  car_lane = 0;
} else if (d >= 4 && d < 8) {
  car_lane = 1;
} else if (d >= 8 && d <= 12) {
  car_lane = 2;
            } else { continue;}
```

* Next, define three basical dager signals for future path planning.

##### Too Close situation.
 
```cpp
// Singnal for the situation where there is another car in front 
// The distance between cars is less than 20
int lane = 1;
if (lane == car_lane)  {
too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < 30);
} 
```
##### Car is to the right situation

```cpp
// detected car_lane is right next to the lane of the car 
// and the distance is between 20miles behind and 30 miles ahead
if (car_lane - lane == 1) 
{car_right |= ((car_s - 20) < check_car_s) && ((car_s + 30) > check_car_s);
        } 

```
##### Car is to the left situation

```cpp
// detected car_lane is left next to the lane of the car 
// and the distance is between 20miles behind and 30 miles ahead
if (lane - car_lane == 1) 
        {car_left |= ((car_s - 20) < check_car_s) && ((car_s + 30) > check_car_s);
				}
```
* Finally, according to the danger situation, the car_s is making a planning for the future.


##### If the car is too close, do the following planning

- if there no car to the right, and there is a lane on the right, change lane to right one.
- if there no car to the left, and there is a lane on the left, change lane to right one.
- If the above situation can not be satisfied, then slow down the speed of the car

```cpp
if (too_close)
{
if (!car_right && lane < 2 ) // no car to the right and there is a lane to go right
{lane++;}
else if (!car_left && lane > 0) // no car to the left and there is a lane to ghe left
{lane--; }
else
{ref_vel -= .224;}
}
```
##### If the car is not in lane 1, which is the center, them move back to the center
```cpp
else
{if (lane !=1)
{if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) 
  {lane = 1;}
 ```
 ##### If the car's speed is less than 50 miles, speed up to 50 miles.
 ```cpp
       if (ref_vel < 49.5)
          { ref_vel += .224; }
```
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

