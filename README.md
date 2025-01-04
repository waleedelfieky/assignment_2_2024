# Robot Navigation System with Action Server

## Table of Contents:
- [overview-of-system](#overview-of-system)
- [Create_Custom_Message](#Create_Custom_Message)
- [Create_Custom_Service](#Create_Custom_Service)
- [Node_One_implementation](#Node_One_implementation)
- [Node_Two_implementation](#Node_Two_implementation)
- [video](#video)
- [Contact Information](#contact-information)

## overview-of-system:
This project provides a ROS-based solution to improve robot navigation towards a target point. The system resolves the blocking behavior by implementing an action server, allowing the robot to continue functioning while moving towards the target.

### Key Features:
- **Action Client Node**: Allows the user to set or cancel a target (x, y) and uses feedback/status from the action server to track progress. It also publishes the robot's position and velocity as a custom message (`x, y, vel_x, vel_z`) based on the `/odom` topic.
- **Service Node**: A service that returns the coordinates of the last target sent by the user.
- **Launch File**: A launch file to start the entire simulation setup, ensuring smooth execution of all components.

## Task Breakdown

1. **Action Client Node**:  
   - Set or cancel a target (x, y) for the robot.
   - Monitor the robot’s progress using feedback/status from the action server.
   - Publish the robot’s position and velocity as a custom message (`x, y, vel_x, vel_z`).
  
2. **Service Node**:  
   - Return the coordinates of the last target set by the user.

3. **Launch File**:  
   - Launch the entire simulation and initialize all components.

## Create_Custom_Message:
in this stage we created a msg directory and define a custom message inside it the following content
![image](https://github.com/user-attachments/assets/a1a8b72b-42db-4029-8107-651b56448729)

## Create_Custom_Service:
in this stage we created a srv directory and define a custom srv inside it the following content
![image](https://github.com/user-attachments/assets/94102c74-12ff-44e6-83a2-7c9189bf86c6)


## Node_One:
# Robot Action Client

## Overview

This ROS-based application implements an action client that sends goals to a robot to navigate to a target location. The client communicates with the action server to control the robot's movement and monitors its state through feedback. It also publishes the robot's position and velocity as custom messages.

### Key Features:
- **Action Client**: Sends navigation goals (x, y) to the robot.
- **Goal Control**: Allows the user to set, cancel, or terminate navigation goals through command-line input.
- **Robot State Publishing**: The robot's position and velocity are published in the `RobotState` message format, based on `/odom` data.
- **Feedback and Status**: Provides real-time feedback on the state of the goal (active, completed, or canceled).

### Workflow:
1. **Set Goal**: The user enters a target (x, y) coordinate, and the robot navigates to that point.
2. **Cancel Goal**: The user can cancel an ongoing goal.
3. **End Program**: The user can terminate the program.

1. Follow the on-screen instructions for this node:
- Press `s` to set a new target location.
- Press `c` to cancel the current goal.
- Press `e` to exit the program.

![image](https://github.com/user-attachments/assets/a0195d51-3853-4d05-a7bd-b25de3765fcb)

there is an error check here is the user entered invalid input as shown in the follwoing picture
![image](https://github.com/user-attachments/assets/3285be2a-d347-467a-a299-d83a2505018b)


after we enter the command the robot will move as stated in the video at the end

also the updates is being publiched to the update state topic as showing in the following picture 
![image](https://github.com/user-attachments/assets/633a2c65-c4d6-448d-8e57-cf73f7ee6305)


## Node_Two:
## Overview

The `get_target_service_node` is a ROS node designed to manage and provide the robot's target position. It listens for target goal updates via a subscriber and offers a service to retrieve the current target coordinates.

## Functionality

This node has two main components:
1. **Subscriber**: It subscribes to the `/reaching_goal/goal` topic and updates the target position whenever a new goal is received.
2. **Service**: It advertises a service called `"get_target"`, which allows users to request the robot's current target position.

### Key Features:
- **Goal Update**: The target position (x, y) is updated based on messages received on the `/reaching_goal/goal` topic.
- **Service**: The node provides a service to return the current target coordinates, which can be queried by other nodes or users.

## ROS Topics and Services

- **Subscribed Topic**: `/reaching_goal/goal`
  - **Message Type**: `assignment_2_2024/PlanningActionGoal`
  - **Purpose**: Receives the target goal (x, y) coordinates.

- **Advertised Service**: `get_target`
  - **Service Type**: `assignment_2_2024/GetTarget`
  - **Purpose**: Returns the current target coordinates (x, y).

## How It Works

1. **Goal Subscription**: When a new goal message is published to the `/reaching_goal/goal` topic, the node's `goalCallback` function is triggered, updating the robot's target position (x, y).
2. **Service**: When the service `"get_target"` is called, the node responds with the latest stored target coordinates.

the service call output is shown in following picture 

![image](https://github.com/user-attachments/assets/c468a174-2fbc-4d3c-9054-a31e1c50536f)

overall system in action would be observed in the attached video at the end 

##launch_file

# Launch File README

## Overview

This launch file initializes several nodes and parameters for the `assignment_2_2024` package, including simulation setup, wall-following behavior, and navigation.

## Modifications

The following lines were added to the existing launch file:

![image](https://github.com/user-attachments/assets/20917fdb-709e-46d8-9728-eca49f574ec7)

These additions integrate:

- robot_action_client: A node for sending goals to the robot.
- target_service: A node that provides the target position as a ROS service.


## video for the final output:

https://github.com/user-attachments/assets/9b975f53-e362-4c72-9152-10f24c2b3669



## Contact Information:

If you have any questions, suggestions, or feedback regarding these this project, please feel free to contact me at waleedelfieky@gmail.com

