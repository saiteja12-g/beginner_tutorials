# ROS2 CPP Publisher and Subscriber Tutorial

## Overview

The main purpose of this package is to disseminate personalized text messages.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble

## Build and Run Instructions

1. **Clone Package**

   ```sh
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src 
   git clone https://github.com/saiteja12-g/beginner_tutorials.git
   ```

2. **Build the Package**

    Navigate to your root ROS 2 workspace and build the package using colcon.

    ```sh
    colcon build --packages-select beginner_tutorials
    ```

3. **Source the Workspace**
    Source the ROS 2 workspace to set the environment for the package.

    ```sh
    source install/setup.bash
    ```

4. **Run the Publisher Node(talker)**
    You can now run the custom string publisher node.

    ```sh
    ros2 run beginner_tutorials talker
    ```

5. **Run the Subscriber Node(listener)**
    Open a new terminal, navigate to ros2 workspace and run the subscriber node.

    ```sh
    source install/setup.bash
    ros2 run beginner_tutorials listener
    ```

## References

[1] <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>