# Robot Navigation System with Action Server

## Overview

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

  
## Table of Contents:

- [overview-of-system](#overview-of-system)
- [Create_Custom_Message](#Create_Custom_Message)
- [Create_Custom_Service](#Create_Custom_Service)
- [Node_One_implementation](#Node_One_implementation)
- [Node_Two_implementation](#Node_Two_implementation)
- [video](#video)
- [Contact Information](#contact-information)

## overview-of-system:



## Node_One:

## Node_Two:

## video for the final output:




## Contact Information:

If you have any questions, suggestions, or feedback regarding these this project, please feel free to contact me at waleedelfieky@gmail.com

