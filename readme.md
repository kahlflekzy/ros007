# BehaviorTree with Internal Map Service
- [BehaviorTree with Internal Map Service](#behaviortree-with-internal-map-service)
  - [Introduction](#introduction)
  - [Start](#start)
    - [Workspace](#workspace)
    - [Install Dependencies](#install-dependencies)
    - [Create Package](#create-package)
  - [Main Updates](#main-updates)
    - [Creating the Map Service](#creating-the-map-service)
    - [Node Arguments](#node-arguments)
  - [Execution](#execution)
  - [REFERENCES](#references)
## Introduction
In this project, we created our own map server, instead of using the `map_server` node from the `navigation` stack. 
Creating our own server, helps us eliminate a bottleneck of the `map_server` node.  

This restriction is that, the node is required to be executed from the command-line, which is a challenge on its own, 
when we want to do everything in code. Although this is possible, like we did in 
[ros006](https://github.com/kahlflekzy/ros006), it is not desirable.

Secondly, we would like a feature in which maps are swapped from code, and thus creating our own map server, helps us 
in this future bids.

## Start
### Workspace
Create a workspace by running,
```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### Install Dependencies
Install `BehaviorTree` and `Turtblebot3 Navigation Simulations` from GitHub. Refer to 
[ros005](https://github.com/kahlflekzy/ros005) for details on this.

### Create Package
Create a package as in [ros006](https://github.com/kahlflekzy/ros006).
```
catkin_create_pkg behavior_tree_navigation_v3 actionlib behaviortree_cpp_v3 rospy roscpp \
actionlib_msgs geometry_msgs nav_msgs std_msgs message_generation message_runtime
```

Since I was cloning `behavior_tree_navigation_v2`, I had to copy all relevant folders to the new package and change 
all the occurrences of its name to my new package name i.e., `behavior_tree_navigation_v3`.

I then added `tf_conversions` to `exec_depends` in `package.xml`

I subsequently realized I needed setup.py after all (because the move_to_base node imports from the execute_task node). 
But the process was clear: 
1. I used the old `setup.py`, and changed the package name to the new package,
2. Uncommented `catkin_python_setup()`
3. Ran `catkin_make` 

And the console output indicated files where installed in the `devel space` where the ought to be.

## Main Updates
### Creating the Map Service
Creating the Map Service proved tricky. I first had to install the Python package with the code below.
```
python3.8 -m pip install pysdl2 pysdl2-dll
```
Particularly, ensure to install these packages using the Python version your ROS uses. Mine is `python3.8`

Then I did a lot of reading, and research, some links are provided below. And finally was able to get it up and running. 
Please refer to the codes `scripts\map_server_node.py` for details. I could have implemented this in C++, which might 
have been easier, but doing it in Python presented some benefits, 
1. I learned Python in depth,
2. I had to pore deeply into the C++ code rather than just copy and paste it.
3. I later realized doing it in C++ would have required me installing the dependencies of `SDL`, `SDL_Image`, and 
`Bullet`, none of which was clear on its procedures, but of course researchable.

### Node Arguments
I made changes to the nodes to passed in paths to files via the command line using `ROS` arguments which are fed to the 
`Nodes` via the launch file. 

I also created a data folder in the package and placed all data files (maps, goals, poses).

## Execution
In one terminal, run the instructions below line by line, the bot model can be `burger`, `waffle` or `waffle_pi`
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
conda deactivate
roslaunch behavior_tree_navigation_v3 turtlebot3_navigation.launch
```
In another terminal, note that the `args` are optional.
```
source devel/setup.bash
roslaunch behavior_tree_navigation_v3 behaviortree_navigation.launch [map_file:=path_to_map.yaml] [init_pose_file:=path_to_init_pose_file.dat]
```
Then in yet another terminal
```
rosrun behavior_tree_navigation_v3 task_publisher.py
```

## REFERENCES
1. [Map Server C++ Node](https://github.com/ros-planning/navigation/blob/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/map_server/src/main.cpp)
2. [Image Loader](https://github.com/ros-planning/navigation/blob/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/map_server/src/image_loader.cpp)
3. [PYSDL2](https://pysdl2.readthedocs.io/en/latest/install.html)
4. [SDL2_image](https://github.com/libsdl-org/SDL_image)
5. [LOAD](https://pysdl2.readthedocs.io/en/latest/modules/sdl2_sdlimage.html#general-image-loading-functions)
6. [f-strings float precision](https://stackoverflow.com/a/46062115)
7. [C++ unsigned char *](https://www.geeksforgeeks.org/unsigned-char-in-c-with-examples/)
8. [Value in Memory](https://stackoverflow.com/a/8250902)
9. [Hex to Int](https://stackoverflow.com/a/209550)
