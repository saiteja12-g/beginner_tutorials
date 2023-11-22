# ROS2 CPP Publisher and Subscriber Tutorial

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- rclcpp
- stdmsgs

# ROS2 CPP Publisher and Subscriber Tutorial

## Overview

The main purpose of this package is to learn ROS.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- rclcpp
- stdmsgs

## Building the package

### Clone Package
```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src 
git clone https://github.com/saiteja12-g/beginner_tutorials.git
```

### Build the Package
```sh
cd ..
colcon build --packages-select beginner_tutorials
```

## Testing the package
```sh
cd ~/ros2_ws
source install/setup.bash
colcon test --event-handlers console_direct+ --packages-select beginner_tutorials
```
close the terminal after the above command

## Checking tf
Open 4 terminals. In all the terminals, run the following commands
```sh
cd ~/ros2_ws
#sourcing the terminals
source install/setup.bash
```
### Start static tf publisher
##### Terminal 1
```sh
ros2 run beginner_tutorials talker
```
### Check tf transform on terminal
##### Terminal 2
```sh
ros2 run tf2_ros tf2_echo world talk
```

### Starting the server
##### Terminal 3
```sh
ros2 run beginner_tutorials server
```

### Save tf tree as pdf
##### Terminal 4
```sh
cd ~/ros2_ws/src/beginner_tutorials/results
ros2 run tf2_tools view_frames
```


## Bag Files

Open a new terminal
### Record a Bag file
##### Terminal 1
make sure you didnt close `Terminal 1` & `Terminal 3` from above
```sh
cd ~/ros2_ws
source install/setup.bash
cd ~/ros2_ws/src/beginner_tutorials/launch
ros2 launch beginner_tutorials rosbag_launch.xml bag_record:=1
```
To stop recording, press `Ctrl+C`

### View info in recorded bag file
In the same terminal,
```sh
cd
cd ~/ros2_ws/src/beginner_tutorials
ros2 bag info all_topics
```

### Play a Bag file
Open 2 terminals, run the following commands.
```sh
cd ~/ros2_ws
#sourcing the terminals
source install/setup.bash
```
##### Terminal 1
```sh
cd ~/ros2_ws/
ros2 run beginner_tutorials listener
```

##### Terminal 2
```sh
cd ~/ros2_ws/src/beginner_tutorials
ros2 bag play all_topics
```
Close all terminals

## Launch the nodes
Open 3 terminals, run the following commands.
```sh
cd ~/ros2_ws
#sourcing the terminals
source install/setup.bash
```
### Run the Publisher Node(talker)
##### Terminal 1
```sh
ros2 run beginner_tutorials talker
```

### Run the Subscriber Node(listener)
##### Terminal 2
```sh
ros2 run beginner_tutorials listener
```
### Launch server
##### Terminal 3
```sh
ros2 run beginner_tutorials server
```
Close all terminals

## Build and Run nodes by launch file
Open 2 terminals, run the following commands
```sh
cd ~/ros2_ws
#sourcing the terminals
source install/setup.bash
```
#### Terminal 1
```sh
cd src/beginner_tutorials/launch
ros2 launch launch.yaml frequency:=20.0
```

### Change `frequency` parameter 
```sh
ros2 param set \minimal_publisher freq 5.0
```
## Results 
Terminal output
![Alt text](/results/rqt_&_.png)

Logging info and RQT node graph
![Alt text](/results/rqt_&_.png)
