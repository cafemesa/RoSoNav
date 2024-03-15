# RoSoNav: Robot Social Navigation Testbed

These tools for simulated scene generation was created for the publication **RoSoNav: Robot Social Navigation Testbed** submitted to the International Conference on Robotics on Intelligent Robots and Systems (IROS 2024). This environment is presented as an alternative to evaluate social navigation algorithms in robots, taking into account real trajectories from the BIWI Walking Pedestrians dataset (https://icu.ee.ethz.ch/research/datsets.html), as well as the dynamics of the robot. This tools allows the possibility to easily integrate new navigation algorithms, create new scenes and modify the speed at which the agents move.

## 1. Requirements

1. Ubuntu 20
2. ROS Noetic
3. Gazebo 9 or later
4. Turtlebot2 Simulator in ROS
5. Evarobot Simulator in ROS

**Note**: We offer installation files that are designed to fully meet the prerequisites 2 and 3 on our GitHub repository at https://github.com/cafemesa/NoeticFocalInstaller.


## 2. Clone and compile the tools

```
cd ~/catkin_ws/src
git clone https://github.com/cafemesa/RoSoNav
chmod +x RoSoNav/scripts/*.py
cd ~/catkin_ws
catkin_make
```


## 3. Run the simulator

1. First terminal
```
roscore
```

2. Second terminal

```
rosrun social_robot_testbed run_experiment.py
```

The results of the experiment will be stored in annotations/, robot positions, success rate and minimum distance between persons

## 4. Customize the simulator

To customize the simulator you can edit the run_experiment in the script folder as follow

1. Testing algorithms: The algorithms are defined in the navigation methods array. As default  the tools works with ["unaware", "sf", "rvo" , "sacadrl"]. If you want, is possible to remove from the simulator some algorithms just removing from the array. If you want to add a new one, add the name to the array and your controller node into the navigation_test.launch file in the section "Launch Robot Controller" under the same format.

2. Testing speeds: as default we define 6 different simulation speeds. These speeds are the normalized maximal speeds of agent in the different scenes defined in the scene generations. These speeds are: 0.3, 0.4, 0.5,0.6, 0.7, 1.0, 1.8 and 2.0. To avoid simulate one speed, just remove from the array. 

If you want to create simulated scenes with different normalized speeds, edit the node file in scripts/dataset_transform_eth.py or scripts/dataset_transform_hotel.py. The array sim_speed_limits[] contains the different speed limits. Then compile your workspace and run the following commant to generate the .world files. Then also add the added speed to the speeds_values[] array in scripts/run_experiment.py:

```
rosrun social_robot_testbed dataset_transform_XXXXXX.launch 
```


3. Testing scenes: The scenes are defined in sceneIds array. As default  the simulator works with the scenes 0 to 9 and 0 to 14 (ETH and HOTEL respectivily). If you want, is possible to remove from the simulator some scenes just removing from the array. 

If you want to create a new simulated environment on another fragment of the dataset, edit the node file in scripts/dataset_transform_eth.py or scripts/dataset_transform_hotel.py, lines 25-26. Here you can define the initFrame, the endFrame. Then compile your workspace and run:

```
rosrun social_robot_testbed dataset_transform_XXXXXX.launch
```
Then add the scene id to the scene_ids array in scripts/run_experiment.py and run the node


## 5. Other tools

<!-- 1. Path irregularity calculation.

As part of the simulator, we have added the file path_irregularity.py. This script will calculate the irregularity of the path for the conducted experiments. You only need to modify lines 11-13. In line 11, you should change the scene IDs, in line 12 the velocities, and in line 13 the navigation algorithms. The script will print the results in the terminal for the scenarios with the selected combinations. To run the script execute the following commands.
```
cd ~/catkin_ws/src/NaSoNav
python path_irregularity.py
```

2. Plot resulnting paths.

As part of the simulator, we have added the file graph_paths.py. This script will plot the path of the agent and robot in the selected results files. You only need to modify lines 40 and 41. In line 40, you should add the labels of each path you will plot and in line 41 you should add the corresponding files with the imprmation to be plotted. The paths are stores in the annotation folder, in "model" are the results for the expermient with the selected agent and in "tests" are the results with the robots. the files follow the following pattern:

sceneX_Y.Y_Z_agentPositionsFileName.csv (for the experiment with agent)
sceneX_Y.Y_Z_METHOD_robotPositionsFileName.csv (for the experiment with robot)

Where X is the sceneID, Y.Y is the normalized speed. Z is the indentifier that tell us if is a simulation with robot or person. And METHOD is the navigation algorith name.

```
cd ~/catkin_ws/src/NaSoNav
python graph_paths.py
``` -->