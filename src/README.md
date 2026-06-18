<h1 align="center">🛑CONTROL SOFTWARE🛑</h1>
This file contains the core components for the robot's control system and program. This shows how the robot is able to navigate the course and avoid the obstacles. Below is the general breakdown of the program:

# <kbd> **cameraMain** </kbd>

Handles camera initialization and frame acquisition. This module controls what the robot sees, which helps it detect obstacles and corners.

# <kbd>**robot-obstacle-challenge**</kbd>

Helps the robot move through areas with obstacles and to detect corners. It reads data from the camera and the sensors to detects blocks and curls around them. This makes the robot drives smoothly without hitting or crashing into the wall.

# <kbd>**robot-open-challenge**</kbd>

Helps the robot move through the track with precision. It reads data from the camera to detect the wall, ultrasonic sensors to determine the direction (clockwise or counterclockwise), and color sensors to determine whether it will do Avoidblocks or detect corner.
