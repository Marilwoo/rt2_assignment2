# Research Track 2 - Second Assignment
##### Maria Luisa Aiachini - 4375373


## How to run
At first you need to clone [this repository](https://github.com/Marilwoo/rt2_assignment2) in a ros workspace. Build the package in a ros1 sourced terminal running
```
catkin_make
```
For running the simulation the user needs to open two terminals, both sourced with ROS. In the first one the simulation will be run, as well as all the nodes but this one. Run:
```
roslaunch rt2_assignment2 sim.launch
```
In the second terminal, move in the folder
```
ros_ws/src/rt2_assignment2/notebook
```
Here run:
```
jupyter notebook --allow-root
```
Once the jupyter is opened go in the file user_interface.ipynb and run every node. At this point everything is running and the user is able to decide how to interact with the robot.
