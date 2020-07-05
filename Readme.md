# Self driving Prius with human and obstacle detection

This repository includes a ROS framework and solutions for the actuation of a self-driving Prius, human detection and obstacle detection.

## Installation

Open a terminal window and use the following shell commands to install this repository:

```bash
source /opt/ros/kinetic/setup.sh
cd
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://gitlab.me41025.3me.tudelft.nl/students-1920/lab4/group61
git clone git@gitlab.me41025.3me.tudelft.nl:students-1920/me41025_simulator.git
cd ..
catkin_make
source devel/setup.sh
roslaunch control_solution solution.launch
```

## Usage
The simulation shows the prius following a path and in doing so, avoiding the abstacles around it. It also completely halts if there is a human detection. You can customise the simulation environment by adding a gazebo tag to the simulation command:


```ROS
roslaunch control_solution solution.launch gazebo_gui:=true
```

## License
Tom Kerssemakers

Binnert Prins

