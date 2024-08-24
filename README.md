# TurtleBot Navigation with PID Implementation

This project implements precise navigation of a TurtleBot in a Gazebo environment using PID controller. The robot is tuned to move to specific coordinates, ensuring accurate movement and positioning. 

## Features

- **Precise TurtleBot movement** to specific coordinate using custom made GUI controller
- **Simulation in Gazebo**, providing a virtual environment for testing the turtlebot PID navigation.

## Project Structure

- **Subscriber Node**: Listens to the target coordinates and processes the movement of the TurtleBot.
- **Receiver Node**: Sends the target coordinates to the TurtleBot.

## Getting Started

### Prerequisites

Before running the project, ensure that you have the following installed:

- ROS (tested on ROS Noetic)
- Gazebo
- TurtleBot3 packages
#### For installing turtlebot3 visit the following website: 
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

## Setting Up the Workspace

1. Create a new ROS workspace:

    ```bash
    mkdir -p ~/turtlebot_movement/src
    cd ~/turtlebot_movement/src
    ```

2. Clone the repository:

    ```bash
    git clone https://https://github.com/shafi075/Turtlebot_Movement.git
    ```

3. Build the workspace:

    ```bash
    cd ~/turtlebot_movement
    catkin_make
    ```

4. Source the workspace:

    ```bash
    source devel/setup.bash
    ```

### Launching the Project

1. Launch the Gazebo environment with TurtleBot3:

    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```

2. Run the Subscriber Node:

    ```bash
    rosrun turtlebot_movement move_sub_with_pid.py
    ```

3. Run the Receiver Node:

    ```bash
    rosrun turtlebot_movement move_pub.py
    ```
After running the publisher node, a custom made GUI controller will pop-up. Using the controller, set the x and y value to move the turtlebot.



### Results

Upon running the project, the TurtleBot will move towards the specified coordinates with precision due to the tuned PID controller. After the first iteration three CSV files will be generated. The python scripts are given in the package. Run the python scripts to see the results graphically.

- **Trajectory Plot with PID**: Showing the TurtleBot's path towards the goal with PID Implementation.
- **Trajectory Plot with PID**: Showing the TurtleBot's path towards the goal without PID Implementation.
- **Error vs. Time Graph**: Displaying the error decrease over time due to PID correction.


### Future Work

This project can be extended by:

- Implementing obstacle avoidance.
- Implementing different path planning algorithms. (like RRT, A* etc)
- Enhancing the PID tuning for faster convergence.
- Applying the same control logic to real-world TurtleBot3 hardware.


## Acknowledgments

- Special thanks to the open-source contributors for ROS, Gazebo, and TurtleBot3.
