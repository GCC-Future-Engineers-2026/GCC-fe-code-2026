# Content (GitHub)
**scheme** - This folder contains schematic diagrams of the electromechanical components(sensors and motors) used in the vehicle and how they are connected to each other. This folder also includes the schematic diagram of the robot during open and obstacle challenge.

**src** - This folder contains all the code and programming used for the WRO 2026

**t-photos** - This folder contains photos of the participants 

**v-photos** - This folder contains photos of the vehicle from multiple sides.(Top, Bottom, Front, Back, etc)

**video** - This folder contains video footage of how the robot completes the challenges presented to it.

# Table of Contents
* [Introduction](#introduction)

* team
* [1.0 Design of self-driving car](#1.0-design-of-self-driving-car)
  * [1.1 Tradeoffs](#1.1-tradeoffs)

  * [1.2 Why components were chosen](#1.2-why-components-were-chosen)
* [2.0 Mobility Management](#2.0-mobility-management)
  * [2.1 Chassis of car](#2.1-chassis-of-car)
* [3.0 Power and sense management](#3.0-power-and-sense-management)
  * [3.1 Power source](#3.1-power-source)

  * [3.2 Sensors](#3.2-sensors)
* [4.0 Materials Needed](#4.0-materials-needed)
* [5.0 Wiring Diagram](#5.0-wiring-diagram)
* [6.0 Obstacle Management](#6.0-obstacle-management)
  * [6.1 Open Challenge](#6.1-open-challenge)

  * [6.2 Obstacle Challenge](#6.2-obstacle-challenge)
* [7.0 Engineering Factor](#7.0-engineering-factor)
* [8.0 Engineering decisions???](#8.0-engineering-decisions)
## Introduction
This documentation cantains comprehensive information about the robot's design, mobility management, power and sensor management, wirng diagram, materials needed, third-party factors and components, obstacle detection and avoidance, and the decisions and improvements made in preparation made for Philippine Robot Olympiad (PRO) located in Mall of Asia (MOA), Pasay in Philippines 2026 under the Future Engineers category.
## Team, Philippines
From the Philippines, [] is a three-member team
## 1.0 Design of self-driving car

## 1.1 Tradeoffs
We have had multiple tradeoffs due to the changes we made with the design. We increased the value of steering, but we had to make the robot slower as a result. We also moved the motor to a different position, so we could have better weight distribution. We moved the ultrasonic sensor to on top of the wheel, so that our robot doesn't turn too early. Our robot was also braced, but the tradeoff is our robot's width is longer.
## 1.2 Why components were chosen
We chose spike as our component, because it has similar multithreading capabilities to EV3 and is smaller, faster, easier to build around, and has better connectivity to openmv.
## 2.0 Mobility Management

## 2.1 Chassis of car
**1. Tire**


**2. Implementation of motor**


**3. Use of differential gear**


**4. Engineering principle for steering**


## 3.0 Power and sense management
This segment is dedicated to the vehicle's power supply and usage and its sensor systems. It covers each sensor's implementation and use along with information about the robot's power supply. There is also a wiring diagram given to illustrate the connection of sensors.
## 3.1 Power source

## 3.2 Sensors

## 3.3.0 Wiring Diagram
This diagram shows the sensor's power usage and how the motors are connected with them to run the vehicle.
## 4.0 Materials Needed
This section shows the materials needed to build this vehicle.
## 5.0 Obstacle Management
For this project, we used Python as the coding language to operate the robot. Our robot uses two different codes that both utilize a camera to complete the open and obstacle challenge. This section includes the code overview and a flowchart for the open challenge and obstacle challenge.
## 5.1 Open Challenge
The robot uses two ultrasonic sensors on its left and right, an openmv camera to trigger the turning, and the in-built gyro sensor to assist its turns and face the correct direction. At the start of the challenge, the robot will use pid to navigate towards a corner. The robot has a roi (region of interest) at the middle of its camera to detect if it is getting close to a corner. When that roi detects black, the robot will execute a 90 degree turn by reversing. The robot determines the direction of the turn based on the difference of the left and right ultrasonic sensor. After completing a turn, a variable called “num_turn” will increase by one. The robot uses the absolute heading value to determine where to face after turning and during pid. While the variable is less than 12, the robot will continue to repeat the pid and turning code. When the variable reaches 12, the robot will continue to go forward for a few second with pid towards its starting section.
(flowchart)
## 5.2 Obstacle Challenge
For the obstacle challenge, the robot uses a code similar to the open challenge. The camera has another roi that takes up half of the camera to detect an approaching obstacle and its color. The robot uses the ultrasonic sensors to determine which direction it needs to face after leaving the parking lot. The robot uses pid to face straight forward. When a block enters the roi, the camera also detects its color. By using the x coordinate and the color of the obstacle, the robot is able to maneuver away from the obstacle. To avoid detecting corner blocks that cause the robot to face the wrong direction, we utilized the orange and blue lines of the track. When the robot passes the orange or blue line, the robot will ignore the obstacle until it makes a turn. The robot will repeat this until num_turn is equal to 12. When num_trun reaches 12, the robot will align itself using the walls and execute a parallel parking.
## 6.0 Engineering Factor

## 7.0 Engineering decisions???
