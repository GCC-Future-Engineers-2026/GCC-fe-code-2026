# Content (GitHub)
**scheme** - This folder contains schematic diagrams of the electromechanical components(sensors and motors) used in the vehicle and how they are connected to each other. This folder also includes the schematic diagram of the robot during open and obstacle challenge.

**src** - This folder contains all the code and programming used for the WRO 2026

**t-photos** - This folder contains photos of the participants 

**v-photos** - This folder contains photos of the vehicle from multiple sides.(Top, Bottom, Front, Back, etc)

**video** - This folder contains video footage of how the robot completes the challenges presented to it.

# Table of Contents
* [Introduction](#introduction)

* team
* [1.0 Design of self-driving car](#10-design-of-self-driving-car)
  * [1.1 Tradeoffs](#11-tradeoffs)

  * [1.2 Why components were chosen](#12-why-components-were-chosen)
* [2.0 Mobility Management](#20-mobility-management)
  * [2.1 Chassis of car](#21-chassis-of-car)
* [3.0 Power and sense management](#30-power-and-sense-management)
  * [3.1 Power source](#31-power-source)

  * [3.2 Sensors](#32-sensors)
  * [3.3 Wirning Diagram](33-wiring-diagram)
* [4.0 Bill of Materials (BOM)](#40-bill-of-materials-bom)
* [5.0 Obstacle Management](#50-obstacle-management)
  * [5.1 Open Challenge](#51-open-challenge)

  * [5.2 Obstacle Challenge](#52-obstacle-challenge)
* [6.0 Engineering Factor](#60-engineering-factor)
* [7.0 Engineering decisions and Improvements](#70-engineering-decisions-and-improvements)
  * [7.1 Robot Construction](#71-robot-construction)

  * [7.2 Robot Programming](#72-robot-programming)
## Introduction
This documentation cantains comprehensive information about the robot's design, mobility management, power and sensor management, wirng diagram, materials needed, third-party factors and components, obstacle detection and avoidance, and the decisions and improvements made in preparation made for Philippine Robot Olympiad (PRO) located in Mall of Asia (MOA), Pasay in Philippines 2026 under the Future Engineers category.
## Team, Philippines
From the Philippines, [] is a three-member team
## 1.0 Design of self-driving car
This autonomous self driving car is made with Lego Education Spike Prime Set. This segment also explains the tradeoffs and the reason components were used in the design.

<img width="600" alt="690765661_1347477953937247_5994651162939509079_n_page-0001" src="https://github.com/user-attachments/assets/e884ec7b-a35a-4bfd-a3b0-52f834bb8b44" />

<img width="600" alt="708917439_1277372907521741_2746426644713458765_n_page-0001" src="https://github.com/user-attachments/assets/f8382ac1-9e5b-4125-8ed9-9c8552495eb2" />

<img width="600" alt="707406137_959257723587473_4280487683574412733_n_page-0001" src="https://github.com/user-attachments/assets/b0415d3f-6992-4de4-a209-524523ec0ead" />

<img width="600" alt="714203118_1504382398009752_7752009258684834922_n_page-0001" src="https://github.com/user-attachments/assets/e87cf11e-f254-4b0b-b9b9-a55469a1e50b" />

<img width="600" alt="714463242_1721986348804527_7473713022599512553_n_page-0001" src="https://github.com/user-attachments/assets/acdd670f-3299-4272-a3c4-efb147d2ff72" />

<img width="600" alt="713583167_2016277132356987_8938094069705343189_n_page-0001" src="https://github.com/user-attachments/assets/f39435a3-d9d6-4b37-80e0-fa5e4accd08d" />




## 1.1 Tradeoffs
We have had multiple tradeoffs due to the changes we made with the design. We increased the value of steering, but we had to make the robot slower as a result. We also moved the motor to a different position, so we could have better weight distribution. We moved the ultrasonic sensor to on top of the wheel, so that our robot doesn't turn too early. Our robot was also braced for better stability, but the tradeoff is our robot's width is longer.
## 1.2 Why components were chosen
We chose spike as our component, because it has similar multithreading capabilities to EV3, smaller, faster. It is also easier to build around, has better connectivity to openmv, and has more flexible ports due to all ports and either be all sensors or all motors.
## 2.0 Mobility Management
This segment shows the propulsion system and mechanical structure of the robot. It covers the main structure, tires, motors, and the entire layout of the vehicle. It also includes information about the engineering principle of the motor and steering that controls speed and torque.
## 2.1 Chassis of car
**1. Tire**

<img width="600"  alt="IMG20260603094123" src="https://github.com/user-attachments/assets/494423df-6eed-41d8-aac9-5b865e05e298" />

**2. Implementation of motor**

Our robot consists of two Lego Technic motors, one being medium angular motor (vertical) and the other being large angular motor (horizontal). The medium (vertical) motor faces inwards to further makes the robot compact to reduce overall length. This keeps our robot compact and saves space, reduces turning radius for sharper turns.

**3. Use of differential gear**

The differential gear is integrated into a drive axle that lets the wheels rotate at different speeds. This stops the wheels from locking up and helps the vehicle turn smoothly and stay stable. This is especially helpful when turning in both challenges.


<img width="600" <img width="2340" height="1988" alt="IMG20260603094123" src="https://github.com/user-attachments/assets/e2d7b4ab-645c-4b0e-8637-dffae848b71f" />
alt="rn_image_picker_lib_temp_f4224a3c-ca7e-411c-a0ef-28fc0d756e2f" src="https://github.com/user-attachments/assets/dffd6470-33db-416c-9bd1-e7ddb2f6bbb8" />

**4. Engineering principle for steering**

This robot is designed using Ackermann steering geomatry.

<img width="600" alt="IMG20260603094609" src="https://github.com/user-attachments/assets/5d011a8d-1e35-4b20-b7be-6d5b6f4d23cc" />


This setup turns the inside front wheel at a sharper angle than the outside one, so both wheels follow perfect turning circles. By stopping the tires from scrubbing sideways, it cuts down on friction and wear. This gives the vehicle better traction, more stability, and the ability to make sharp turns with great precision.

## 3.0 Power and sense management
This segment is dedicated to the vehicle's power supply and usage and its sensor systems. It covers each sensor's implementation and use along with information about the robot's power supply. There is also a wiring diagram given to illustrate the connection of sensors.
## 3.1 Power source
The robot is powered by the SPIKE Prime Hub, which features a 2,000 mAh rechargeable lithium-ion battery. It was selected due to the fact that it's easily replaced. It is also at peak voltage and performance for most of its run time, until it dies.

<img width="327" height="205" alt="Screenshot 2026-06-03 100053" src="https://github.com/user-attachments/assets/41e484c0-4b90-4e50-833b-3ca5c74df42b" />


## 3.2 Sensors
**Camera**-The Openmv camera’s function is to detect the blocks and the parking lot. It detects red and green blocks while the robot is in motion and helps it turn left or right. It also helps find the magenta walls after the final turn to help in parking.

<img width="600"  alt="IMG20260604154608" src="https://github.com/user-attachments/assets/34274b7e-d81a-44bd-aeec-8053cfd089f8" />



**Gyro**-The gyro sensor in our robot is used to find and maintain the correct direction. It’s also used to turn at an accurate angle on the turns. It is also built into the Spike Prime Hub.

**Color**-The color sensor is used to detect the orange and blue lines depending on the direction(clockwise or counterclockwise) and tells the robot to not avoid the blocks and find the corner on the wall and turn properly.

<img width="600"  alt="IMG20260604154618" src="https://github.com/user-attachments/assets/010c4c14-a570-4bc2-a299-015fdb562431" />

**Ultrasonic**-Our robot is equipped with two Spike Prime ultrasonic sensors. These sensors detect how far or near the walls are to go closer or avoid contact with the wall. These sensors operate by sending sound waves and calculating the time it takes for the sound wave to bounce back after it hits the wall and converts it to a value. This allows the robot to move to the center of two walls without making contact and makes efficient movement throughout the challenge.

<img width="326" height="197" alt="Screenshot 2026-06-03 100152" src="https://github.com/user-attachments/assets/487a52d6-1fad-4d75-9d63-c4e1a96d1be1" />


## 3.3 Wiring Diagram
This diagram shows the sensor's power usage and how the motors are connected with them to run the vehicle.

<img width="600"  alt="Screenshot 2026-06-03 100708_page-0001" src="https://github.com/user-attachments/assets/746e9d27-c4bf-4e84-b338-048a96054c89" />


## 4.0 Bill of Materials (BOM)
This section shows the materials needed to build this vehicle. Below is the link:

[future engineers.pdf](https://github.com/user-attachments/files/28587582/future.engineers.pdf)


## 5.0 Obstacle Management
For this project, we used Python as the coding language to operate the robot. Our robot uses two different codes that both utilize a camera to complete the open and obstacle challenge. This section includes the code overview and a flowchart for the open challenge and obstacle challenge.
## 5.1 Open Challenge
The robot uses two ultrasonic sensors on its left and right, an openmv camera to trigger the turning, and the in-built gyro sensor to assist its turns and face the correct direction. At the start of the challenge, the robot will use pid to navigate towards a corner. The robot has a roi (region of interest) at the middle of its camera to detect if it is getting close to a corner. When that roi detects black, the robot will execute a 90 degree turn by reversing. The robot determines the direction of the turn based on the difference of the left and right ultrasonic sensor. After completing a turn, a variable called “num_turn” will increase by one. The robot uses the absolute heading value to determine where to face after turning and during pid. While the variable is less than 12, the robot will continue to repeat the pid and turning code. When the variable reaches 12, the robot will continue to go forward for a few second with pid towards its starting section.

<img width="600" alt="Untitled Diagram drawio_page-0001" src="https://github.com/user-attachments/assets/4eb3d02e-a88e-4a4b-9331-e58ade7158f8" />

## 5.2 Obstacle Challenge
For the obstacle challenge, the robot uses a code similar to the open challenge. The camera has another roi that takes up half of the camera to detect an approaching obstacle and its color. The robot uses the ultrasonic sensors to determine which direction it needs to face after leaving the parking lot. The robot uses pid to face straight forward. When a block enters the roi, the camera also detects its color. By using the x coordinate and the color of the obstacle, the robot is able to maneuver away from the obstacle. To avoid detecting corner blocks that cause the robot to face the wrong direction, we utilized the orange and blue lines of the track. When the robot passes the orange or blue line, the robot will ignore the obstacle until it makes a turn. The robot will repeat this until num_turn is equal to 12. When num_trun reaches 12, the robot will align itself using the walls and execute a parallel parking.


<img width="600" alt="Untitled Diagram oc drawio_page-0001" src="https://github.com/user-attachments/assets/4280ebfb-2854-457f-9cae-dfe5255cf6ac" />

## 6.0 Engineering Factor
The third-party factor that we used is the Openmv camera, which enables us to see the blocks and their color. It gives the robot the ability to see the blocks, and in turn, make the right decisions that are needed.

<img width="309" height="347" alt="Screenshot 2026-06-03 093710" src="https://github.com/user-attachments/assets/67ad3cf5-2a82-496e-ae5a-0b0daca77669" />

## 7.0 Engineering decisions and Improvements
This segment explains the challenges, both in the build and programming, when making the robot and the improvements made to clear the challenges.

## 7.1 Robot Construction
The first problem we had was when our robot's ultrasonic sensors were infront of the wheels, which made the timing of the turn too early. To solve that, we made an entirely new design with the sensors on top of the wheels to help the robot turn better. We also switch from cantilever to braced for better stability on the wheels.
## 7.2 Robot Programming
