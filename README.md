# Content (GitHub)
**scheme** - This folder contains schematic diagrams of the electromechanical components(sensors and motors) used in the vehicle and how they are connected to each other. This folder also includes the schematic diagram of the robot during open and obstacle challenge.

**src** - This folder contains all the code and programming used for the WRO 2026

**t-photos** - This folder contains photos of the participants 

**v-photos** - This folder contains photos of the vehicle from multiple sides.(Top, Bottom, Front, Back, etc)

**video** - This folder contains video footage of how the robot completes the challenges presented to it.

# Table of Contents
* [Introduction](#introduction)

* [Team Grace Christian College](#team-grace-christian-college)
* [1.0 Design of self-driving car](#10-design-of-self-driving-car)
  * [1.1 Tradeoffs](#11-tradeoffs)

  * [1.2 Why components were chosen](#12-why-components-were-chosen)
* [2.0 Mobility Management](#20-mobility-management)
  * [2.1 Chassis of car](#21-chassis-of-car)
* [3.0 Power and sense management](#30-power-and-sense-management)
  * [3.1 Power source](#31-power-source)

  * [3.2 Sensors](#32-sensors)
  * [3.3 Wirning Diagram](33-wiring-diagram)
* [4.0 Bill of Materials (BOM) and instructions to build](#40-bill-of-materials-bom-and-instructions-to-build)
* [5.0 Obstacle Management](#50-obstacle-management)
  * [5.1 Open Challenge](#51-open-challenge)

  * [5.2 Obstacle Challenge](#52-obstacle-challenge)
* [6.0 Engineering Factor](#60-engineering-factor)
  * [6.1 Usage of Camera](#61-usage-of-camera)
* [7.0 Engineering decisions and Improvements](#70-engineering-decisions-and-improvements)
  * [7.1 Robot Construction](#71-robot-construction)

  * [7.2 Robot Programming](#72-robot-programming)
## Introduction
This documentation contains comprehensive information about the robot's design, mobility management, power and sensor management, wiring diagram, materials needed, third-party factors and components, obstacle detection and avoidance, and the decisions and improvements made in preparation made for Philippine Robot Olympiad (PRO) located in Mall of Asia (MOA), Pasay in Philippines 2026 under the Future Engineers category.
## Team Grace Christian College
From the Philippines, Team Grace Christian College is a three-member team made to compete in the PRO 2026 in Mall of Asia. 

<img width="30%" alt="Messenger_creation_9F03665D-973D-4025-9D37-10E926EF3D5B" src="https://github.com/user-attachments/assets/1ff4be49-f7dc-450e-8937-915cecd19cf5" />


Its members are Thomas Lao, Yuri Valero, and Kyron Chen.
## 1.0 Design of self-driving car
<img align="right" width="20%" alt="flix" src="https://github.com/user-attachments/assets/09a6862f-22b9-45d2-bd73-e3e6188f6f17" />This autonomous self driving car is made with Lego Education Spike Prime Set. This segment also explains the tradeoffs and the reason components were used in the design.


## 1.1 Tradeoffs
We have had multiple tradeoffs due to the changes we made with the design. We increased the value of steering, but we had to make the robot slower as a result. We also moved the motor to a different position, so we could have better weight distribution. We moved the ultrasonic sensor to on top of the wheel, so that our robot doesn't turn too early. Our robot was also braced for better stability, but the tradeoff is our robot's width is longer.

| | |
|:---|:----|
|<img width="400" alt="a910624c-00f3-4ee8-b32c-c6e39e3e3a63" src="https://github.com/user-attachments/assets/b93fe076-9378-4c38-8162-7b4818c5dacb" />|<img width="400" alt="6" src="https://github.com/user-attachments/assets/d1ae3761-6af4-4ca4-9069-cd16263bd0e6" />|
| **OLD DESIGN** | **NEW DESIGN** |



## 1.2 Why components were chosen
We chose spike as our component, because it has similar multithreading capabilities to EV3, smaller, faster. It is also easier to build around, has better connectivity to openmv, and has more flexible ports due to all ports and either be all sensors or all motors. The Large and Medium Spike angular motor were chosen due to it having special sensors called absolute encoders, which help it move in a straight line and do precise tasks with ease. It is also very easy to build around due to its box-like shape and the fact it has slim wires and not thick ones, which can hinder the build around it.
## 2.0 Mobility Management
This segment shows the propulsion system and mechanical structure of the robot. It covers the main structure, tires, motors, and the entire layout of the vehicle. It also includes information about the engineering principle of the motor and steering that controls speed and torque.
## 2.1 Chassis of car
**1. Tire**

**Lego Technic Tire 62.4 x 20**

<img width="20%"  alt="OIP" src="https://github.com/user-attachments/assets/bc7f8fc5-ba6a-487f-8f1a-bacec84432b2" />

We used these tires because they're big enough that they can still move fast and small enough to make accurate turns, unlike other tires which sacrifice speed or turning for their size. Unlike other larger Lego tires, the 62.4 x 20 has thicker sidewalls, so it won't flatten out under the robot's weight.

**Integrated Wheel 43 x 14(Part 65834)**

<img width="20%" alt="Kolo-Robota-Mindstorms-43x14-1szt-LEGO-65834" src="https://github.com/user-attachments/assets/93f40a8d-c51b-4f4b-a18a-cd7e0507d101" />

We used this wheel compared to others, because of the advantage that it's a single piece, which prevents the tire from slipping off unlike others. The tire is also made of hard rubber, which prevents significant friction and allows the wheel to slide smoothly over the track, requiring less motor power to steer.

**2. Implementation of motor**

Our robot consists of two Lego Technic motors, one being medium angular motor (vertical) and the other being large angular motor (horizontal). The medium (vertical) motor faces inwards to further makes the robot compact to reduce overall length. This keeps our robot compact and saves space, reduces turning radius for sharper turns.

**3. Use of differential gear**

The differential gear is integrated into a drive axle that lets the wheels rotate at different speeds. This stops the wheels from locking up and helps the vehicle turn smoothly and stay stable. This is especially helpful when turning in both challenges.

<img width="25%" alt="Messenger_creation_2DD2113F-E5B7-4058-AAD0-0F9EC8B3905B" src="https://github.com/user-attachments/assets/cebe4825-b5ef-4ca0-b05a-08d2a869794f" />

**4. Engineering principle for steering**

This robot is designed using Ackermann steering geometry.

<img width="50%" alt="Messenger_creation_A0C92DA4-F460-49D0-89A5-i" src="https://github.com/user-attachments/assets/9bf7b1c8-57c6-436e-9b63-53230b88d505" />

This setup turns the inside front wheel at a sharper angle than the outside one, so both wheels follow perfect turning circles. By stopping the tires from scrubbing sideways, it cuts down on friction and wear. This gives the vehicle better traction, more stability, and the ability to make sharp turns with great precision.

## 3.0 Power and sense management
This segment is dedicated to the vehicle's power supply and usage and its sensor systems. It covers each sensor's implementation and use along with information about the robot's power supply. There is also a wiring diagram given to illustrate the connection of sensors.
## 3.1 Power source
<img align="right" width="25%"  alt="Screenshot 2026-06-03 100053" src="https://github.com/user-attachments/assets/41e484c0-4b90-4e50-833b-3ca5c74df42b" />The robot is powered by the SPIKE Prime Hub, which features a 2,000 mAh rechargeable lithium-ion battery. It was selected due to the fact that it's easily replaced. It is also at peak voltage and performance for most of its run time, until it dies.




## 3.2 Sensors
**Camera**-The Openmv camera’s function is to detect the blocks and the parking lot. It detects red and green blocks while the robot is in motion and helps it turn left or right. It also helps find the magenta walls after the final turn to help in parking.

<img width="30%" alt="a" src="https://github.com/user-attachments/assets/98a59458-286c-4f7c-a0f9-371ffc370521" />


**Gyro**-The gyro sensor in our robot is used to find and maintain the correct direction. It’s also used to turn at an accurate angle on the turns. It is also built into the Spike Prime Hub.

**Color**-The color sensor is used to detect the orange and blue lines depending on the direction(clockwise or counterclockwise) and tells the robot to not avoid the blocks and find the corner on the wall and turn properly.

<img width="50%" alt="Messenger_creation_A0C92DA4-F460-49D0-89A5-o" src="https://github.com/user-attachments/assets/9056b1a8-1a10-4d55-8d19-9b44090fe660" />


**Ultrasonic**-Our robot is equipped with two Spike Prime ultrasonic sensors. These sensors detect how far or near the walls are to go closer or avoid contact with the wall. These sensors operate by sending sound waves and calculating the time it takes for the sound wave to bounce back after it hits the wall and converts it to a value. This allows the robot to move to the center of two walls without making contact and makes efficient movement throughout the challenge.

<img width="40%" alt="Messenger_creation_CAF01661-0C92-4251-9CD1-j" src="https://github.com/user-attachments/assets/953f666c-4e99-4b5f-973f-a4d4987c5e5f" />


## 3.3 Wiring Diagram
This diagram shows the sensor's power usage and how the motors are connected with them to run the vehicle.

<img width="70%"  alt="1" src="https://github.com/user-attachments/assets/6b8fc953-147c-4fd8-a46e-876f7e10923e" />

We took advantage of Spike's versatility and used four ports for sensors and only two for motors. This setup helps the robot to sense the obstacles and make smart decisons on the track.

## 4.0 Bill of Materials (BOM) and instructions to build
This section shows the BOM and the instructions to build the vehicle. 

|  |  |
|:----------:|:--------:|
|<img width="450" alt="8_1x" src="https://github.com/user-attachments/assets/995b8d30-48a8-49ff-ad69-789564530cb2" />| |
|<img width="450" alt="9_1x" src="https://github.com/user-attachments/assets/3f4d7998-b673-44e7-9ae5-87aff9d329a2" />|<img width="450" alt="10_1x" src="https://github.com/user-attachments/assets/7f0d7522-93d9-4384-a706-20ade446da86" />|
|<img width="450" alt="11_1x" src="https://github.com/user-attachments/assets/acfdb71d-4d6c-4196-b598-ec89c42d81cf" />|<img width="450" alt="12_1x" src="https://github.com/user-attachments/assets/8282e2f9-77a6-46e8-9915-29daf5fefd8d" />|
|<img width="450" alt="13_1x" src="https://github.com/user-attachments/assets/8badba14-8d87-490b-a61a-00b47f16a207" />|<img width="450" alt="14_1x" src="https://github.com/user-attachments/assets/609d227e-743b-4ea7-b16b-e3389cd930e8" />|
|<img width="450" alt="15_1x" src="https://github.com/user-attachments/assets/4e61a2d1-0432-43ac-9220-47491a47a4a7" />|<img width="450" alt="16_1x" src="https://github.com/user-attachments/assets/4b8826f4-866c-4c5d-8da8-0ee74a5fa729" />|
|<img width="450" alt="17_1x" src="https://github.com/user-attachments/assets/f6e4129d-4542-4c42-8e55-0cccf8477cbb" />|<img width="450" alt="18_1x" src="https://github.com/user-attachments/assets/9835c342-d6dc-45cc-a283-dfdafc937130" />|
|<img width="450" alt="19_1x" src="https://github.com/user-attachments/assets/1ee05c23-3a90-4111-861b-ac4451eb467f" />|<img width="450" alt="20_1x" src="https://github.com/user-attachments/assets/52d1695a-c94d-45f0-b3f3-e7e9972787d3" />|
|<img width="450" alt="21_1x" src="https://github.com/user-attachments/assets/10e8ceae-83f8-4c89-8238-06b180670bd0" />|<img width="450" alt="22_1x" src="https://github.com/user-attachments/assets/1c9c2f24-47fd-4af2-9fc3-7de6fc390062" />|
|<img width="450" alt="23_1x" src="https://github.com/user-attachments/assets/0036b099-3d6a-4ff3-8a00-64fad978eeea" />|<img width="450" alt="24_1x" src="https://github.com/user-attachments/assets/34daa1dc-5eb5-4ce4-9f52-a5b5d97f1ecc" />|
|<img width="450" alt="25_1x" src="https://github.com/user-attachments/assets/47e814c1-d003-4c82-a983-d66518b33518" />|<img width="450" alt="26_1x" src="https://github.com/user-attachments/assets/cc7943b3-a4a7-42c6-bca0-a61b587eb09e" />|
|<img width="450" alt="27_1x" src="https://github.com/user-attachments/assets/04aca3d1-2e07-4f05-a4fb-669fdb401cf1" />|<img width="450" alt="28_1x" src="https://github.com/user-attachments/assets/e1298143-62f2-47af-bd55-2abe99be1fee" />|
|<img width="450" alt="29_1x" src="https://github.com/user-attachments/assets/6aa327cc-d918-4d70-a108-e264629ee9dc" />|<img width="450" alt="30_1x" src="https://github.com/user-attachments/assets/c2ae3975-38d8-444e-8b79-b8a4bf1b16da" />|
|<img width="450" alt="31_1x" src="https://github.com/user-attachments/assets/6e6ba415-30d6-4a22-87ec-494c7e137acf" />|<img width="450" alt="32_1x" src="https://github.com/user-attachments/assets/0e5bb546-6224-4c85-8d8d-0ab8576dbe50" />|
|<img width="450" alt="33_1x" src="https://github.com/user-attachments/assets/9eb429d0-aba8-4823-93c6-6f271c933624" />|<img width="450" alt="34_1x" src="https://github.com/user-attachments/assets/dc726c52-cb4b-457c-aa70-89e4dd63dcf6" />
|<img width="450" alt="35_1x" src="https://github.com/user-attachments/assets/a369356a-8433-4953-827f-8ad973797647" />|<img width="450" alt="36_1x" src="https://github.com/user-attachments/assets/73a7ca37-52bb-4f03-834f-60dd4a160e53" />|
|<img width="450" alt="37_1x" src="https://github.com/user-attachments/assets/e617f8a6-fe62-4be9-8b2a-09af5cd977ea" />|<img width="450" alt="38_1x" src="https://github.com/user-attachments/assets/7d730e15-ef01-4531-bff8-65cb567ff772" />
|<img width="450" alt="39_1x" src="https://github.com/user-attachments/assets/6237811c-7781-4f37-b399-40d63db73447" />|<img width="450" alt="40_1x" src="https://github.com/user-attachments/assets/0ecb07c7-01d1-4ffa-b7c7-3db5ae9ddbf4" />|
|<img width="450" alt="41_1x" src="https://github.com/user-attachments/assets/3b1fe572-30bc-42a3-91e3-237ddd8e9826" />|<img width="450" alt="42_1x" src="https://github.com/user-attachments/assets/0950cae2-b555-4aef-a57b-a9add4a7c574" />|
|<img width="450" alt="43_1x" src="https://github.com/user-attachments/assets/804e9363-5d5f-468c-aaef-983b66c89a99" />|<img width="450" alt="44_1x" src="https://github.com/user-attachments/assets/2ef3f56e-3fef-43a1-aa06-29342fefedd5" />|
|<img width="450" alt="45_1x" src="https://github.com/user-attachments/assets/83762ec4-87c5-4df1-844f-e53aef38c188" />|<img width="450" alt="46_1x" src="https://github.com/user-attachments/assets/f4244109-4b35-48da-bce0-a189e17edb3c" />|
|<img width="450" alt="47_1x" src="https://github.com/user-attachments/assets/39036c39-c12e-45f3-8b7c-935f866e7eb7" />|<img width="450" alt="48_1x" src="https://github.com/user-attachments/assets/e85435ac-e750-4664-b0a1-073d66d9e64d" />|
|<img width="450" alt="49_1x" src="https://github.com/user-attachments/assets/f4a610e7-484c-42ce-8d3c-93ff8f2d2360" />|<img width="450" alt="50_1x" src="https://github.com/user-attachments/assets/2e8e7c36-3e4e-4248-837b-4e5bc713495b" />|
|<img width="450" alt="51_1x" src="https://github.com/user-attachments/assets/779c9a41-8a84-45c8-aa6e-8d28c66490d6" />|<img width="450" alt="52_1x" src="https://github.com/user-attachments/assets/041e1444-5a1d-430f-ae61-f0830cf891d4" />|
|<img width="450" alt="53_1x" src="https://github.com/user-attachments/assets/dba14b06-36f4-4125-ab39-053ecea9aec3" />|<img width="450" alt="54_1x" src="https://github.com/user-attachments/assets/67f9536c-c993-4ff4-9b60-3c177c8c90bc" />|
|<img width="450" alt="55_1x" src="https://github.com/user-attachments/assets/f343c95c-c48e-4304-8be3-e8a21d02b847" />|<img width="450" alt="56_1x" src="https://github.com/user-attachments/assets/573cf3aa-a690-4904-b234-b73ca776dd98" />|
|<img width="450" alt="57_1x" src="https://github.com/user-attachments/assets/1c24200b-707e-4557-be3a-92231ffda569" />|<img width="450" alt="58_1x" src="https://github.com/user-attachments/assets/319fa26c-9a28-42a9-a1ce-9b955c81580f" />|
|<img width="450" alt="59_1x" src="https://github.com/user-attachments/assets/785b9ffc-f7e9-4c90-9c69-ff7e70d6fc36" />|<img width="450" alt="60_1x" src="https://github.com/user-attachments/assets/1577efbb-761f-4679-af53-ee9c959a7dd5" />|
|<img width="450" alt="61_1x" src="https://github.com/user-attachments/assets/3dbf2d6e-b682-4345-8089-099d3e24749c" />|<img width="450" alt="62_1x" src="https://github.com/user-attachments/assets/7cad4e43-92c4-45ba-97ed-964c6aac9b76" />|
|<img width="450" alt="63_1x" src="https://github.com/user-attachments/assets/602d0f3d-856b-4083-b23a-631c8754e9a1" />|<img width="450" alt="64_1x" src="https://github.com/user-attachments/assets/11f53fca-8244-436f-a690-3d64c9f8dbc2" />|
|<img width="450" alt="65_1x" src="https://github.com/user-attachments/assets/039968a4-5391-4655-a551-e10380ace6d1" />|<img width="450" alt="66_1x" src="https://github.com/user-attachments/assets/e019409d-38de-4ad1-bbfb-9a090680def3" />|
|<img width="450" alt="67_1x" src="https://github.com/user-attachments/assets/23a5f242-fd5b-4d04-90e0-08c8feacdbfd" />|<img width="450" alt="68_1x" src="https://github.com/user-attachments/assets/8c09b91b-d6f9-4e49-9dba-e7821bbbf159" />|
|<img width="450" alt="69_1x" src="https://github.com/user-attachments/assets/1b7b5493-845f-469f-b273-e5c89697c402" />|<img width="450" alt="70_1x" src="https://github.com/user-attachments/assets/b85bb40f-2fb9-4af8-8620-eda74fda7b8c" />|
|<img width="450" alt="71_1x" src="https://github.com/user-attachments/assets/dca482e1-d475-446f-9b63-bc4415e254b6" />|<img width="450" alt="72_1x" src="https://github.com/user-attachments/assets/0e48ddc4-94be-402c-aff4-236d4a8b4e9c" />|
|<img width="450" alt="73_1x" src="https://github.com/user-attachments/assets/9e0de179-e6e8-46ee-8b57-06f340eb8430" />|<img width="450" alt="74_1x" src="https://github.com/user-attachments/assets/fbe0b51c-1439-4a50-8bf8-8f036af58fbb" />|
|<img width="450" alt="75_1x" src="https://github.com/user-attachments/assets/163b633d-26c4-4d3e-b764-964a6a2b3dfe" />|<img width="450" alt="76_1x" src="https://github.com/user-attachments/assets/0511df52-d451-481e-b353-540bdb6477e4" />|
|<img width="450" alt="77_1x" src="https://github.com/user-attachments/assets/4b83f8ad-d059-4e7d-9d8c-b8a405e7c8b1" />|<img width="450" alt="78_1x" src="https://github.com/user-attachments/assets/4003b1e5-cf52-48bf-ae1d-9030bc967f53" />|
|<img width="450" alt="79_1x" src="https://github.com/user-attachments/assets/e0d6ccf7-a9b8-4826-93fb-42f4acc82c71" />|<img width="450" alt="80_1x" src="https://github.com/user-attachments/assets/0ba0dd61-a2a6-48c2-85d0-60735963110e" />|
|<img width="450" alt="81_1x" src="https://github.com/user-attachments/assets/63095b8a-53ff-4424-bbb3-8dd98b5f4803" />| |


**Build of Materials (BOM)**
| | |
|:---|:---|
|<img width="450" alt="1_1x" src="https://github.com/user-attachments/assets/012e493a-47a0-4b4b-9e71-5d446988dcfb" />|<img width="450" alt="2_1x" src="https://github.com/user-attachments/assets/3a46e1a0-72a6-4392-975d-e46c169e90af" />|
|<img width="450" alt="3_1x" src="https://github.com/user-attachments/assets/af129b96-11ad-4494-bd68-36f0c6c9bca8" />|<img width="450" alt="4_1x" src="https://github.com/user-attachments/assets/76bf120f-a79b-4e04-bc3b-0912f05189f0" />|
|<img width="450" alt="5_1x" src="https://github.com/user-attachments/assets/1f7a261e-23f2-4c93-8c18-6e7933e22516" />|<img width="450" alt="6_1x" src="https://github.com/user-attachments/assets/78e69178-5a95-4f12-92d5-7b0fdb9b774c" />|
|<img width="450" alt="7_1x" src="https://github.com/user-attachments/assets/2855d29b-dfa2-4fad-b5ef-b807ef798764" />|


|Component |QTY |PHOTO|
|:---|:---|:---|
|OpenMV H7 Plus Camera|1|<img width="30%" alt="490818226-ca1d665c-cde8-455f-a26d-12d1ab6409a1" src="https://github.com/user-attachments/assets/7c5ee0b8-33c3-4ba0-bd69-2e91ee1c7bdc" />|


## 5.0 Obstacle Management
For this project, we used Python as the coding language to operate the robot. Our robot uses two different codes that both utilize a camera to complete the open and obstacle challenge. This section includes the code overview and a flowchart for the open challenge and obstacle challenge.
## 5.1 Open Challenge
The robot uses two ultrasonic sensors on its left and right, an openmv camera to trigger the turning, and the in-built gyro sensor to assist its turns and face the correct direction. At the start of the challenge, the robot will use pid to navigate towards a corner. The robot has a roi (region of interest) at the middle of its camera to detect if it is getting close to a corner. When that roi detects black, the robot will execute a 90 degree turn by reversing. The robot determines the direction of the turn based on the difference of the left and right ultrasonic sensor. After completing a turn, a variable called “num_turn” will increase by one. The robot uses the absolute heading value to determine where to face after turning and during pid. While the variable is less than 12, the robot will continue to repeat the pid and turning code. When the variable reaches 12, the robot will continue to go forward for a few second with pid towards its starting section.

<img width="70%" alt="Untitled Diagram drawio_page-0001" src="https://github.com/user-attachments/assets/4eb3d02e-a88e-4a4b-9331-e58ade7158f8" />

```
p = PUPRemoteHub(Port.D)
p.add_command("msg", to_hub_fmt="repr", from_hub_fmt="repr")
p.add_channel('cam', to_hub_fmt='bhhb')
```
PUPRemoteHub connects the hub to an external OpenMV camera via Port D. It sets up a command channel called 'cam' to receive 4 variables packaged as binary data.
```
async def getMedianR(samples):
    i = 0
    while i < samples:
        distListR[i] = await eyesR.distance()
        i += 1
    
    distListR.sort()
    n = len(distListR)
    mid = n // 2 # median of the 3 samples

    if n % 2 == 1: # if odd number of samples, return information 
        return distListR[mid]
    else: # if even number, find average then return information 
        return (distListR[mid-1] + distListR[mid]) / 2

async def getMedianL(samples):
    i = 0
    while i < samples:
        distListL[i] = await eyesL.distance()
        i += 1
    
    distListL.sort()
    n = len(distListL)
    mid = n // 2

    if n % 2 == 1:
        return distListL[mid]
    else:
        return (distListL[mid-1] + distListL[mid]) / 2
```
To avoid reading random spikes from the ultrasonic sensors, this function takes 3 quick distance samples, sorts them in ascending order, and extracts the middle value (median). This completely discards outlier anomalies.
```
async def remote_engine():
    while True:
        # connects hub to the camera and relay information
        await p.process_async()
        # A tiny wait is necessary to let other tasks in
        await wait(1)

# updates information from camera and sensors
async def update_robot_data():
    global rob_data
    
    while True:
        # This is specifically designed for Pybricks multitasking.
        result = await p.call_multitask("cam")
        
        if result:
            # Unpack the tuple directly from the result
            color, block_x, block_y, corner = result 
            
            heading = hub.imu.heading()
            lineColor = await getLineColor()
            us_R = await getMedianR(3)
            us_L = await getMedianL(3)

            rob_data['color'] = color
            rob_data['block_x'] = block_x
            rob_data['block_y'] = block_y
            rob_data['corner'] = corner
            rob_data['heading'] = heading
            rob_data['ultra_R'] = us_R
            rob_data['ultra_L'] = us_L
            rob_data['line_color'] = lineColor
            
        await wait(20) # Increased to 20ms to give the connection 'breathing room'
```
* remote_engine: Keeps communication alive with the OpenMV camera.

* update_robot_data: Constantly populates a global dictionary (rob_data) every 20 milliseconds with refreshed telemetry: camera tracking data (color, block_x, corner), IMU gyro heading, color sensor color, and filtered ultrasonic distances.
```
async def ultrasonic_PID(kp=0.075, ki=0.000001, kd=10000.0, minPower=900, maxPower=1100):
    global sum_error, prev_error, max_steer, direction, num_turn
    global rob_data, steer_center, speed_center

    drive_power = 0.0
    compensation = 0.0
    
    while num_turn <= 12:  
        # after 3 laps, pid stops   
        error = rob_data['ultra_R'] - rob_data['ultra_L']    
            
        prop = kp * error # proportional
        
        sum_error += error
        integral = ki * sum_error # integral

        if integral > 10.0:
            integral = 10.0

        derivative = kd * (error - prev_error) # derivative

        compensation = prop + integral + derivative

        prev_error = error

        # negative steer = steer to the left
        # positive steer = steer to the right
        
        if compensation > max_steer:
            compensation = max_steer
        elif compensation < -max_steer:
            compensation = -max_steer
        
        maxP = maxPower
        minP = minPower
        max_error = 2000
        
        # cap the steering of the robot 
        if error > max_error:
            error = max_error
        elif error < 0:
            error = 0

        # adjust speed depending on the error - bigger the error, slower the movement and vice versa
        drive_power = -(maxP - minP) * (error - max_error) / max_error + minP   
        
        steer_center = compensation
        speed_center = drive_power     
                    
        await wait(100)
    
    # set speed to 0 to stop robot from moving
    speed_center = 0
```
The Error for the PID is calculated by subtracting the left wall distance from the right wall distance. If the robot is perfectly centered, the error is 0.

This function also gives the robot **dynamic speed**. The function slows the robot down dynamically if the error is large (approaching a wall) and speeds up to maximum power when the error is low (driving perfectly straight).
```
def turn90_pid(num_turn, kp=35, ki=0.000001, kd=1.0, forward=False, maxP=1100, minP=400):
    global sumS_error, prevS_error
    global speed_sequence

    maxS = maxP
    minS = minP
    steer_error = abs((90 * (num_turn)) - abs(rob_data['heading']))
    prop = kp * steer_error # proportional
    
    sumS_error += steer_error
    integral = ki * sumS_error # integral

    if integral > 10.0:
        integral = 10.0

    derivative = kd * (steer_error - prevS_error) # derivative

    compensation = prop + integral + derivative

    prevS_error = steer_error
    
    if compensation > maxS:
        compensation = maxS
    elif compensation < minS:
        compensation = minS
    
    if forward:
        speed_sequence = compensation
    else:
        speed_sequence = -compensation

# signal to turn after detecting corner wall
async def detect_corner():
    global num_turn, direction
    global rob_data, steer_sequence, speed_sequence, is_sequencing
    global can_sense_corner

    while True:
        if rob_data["corner"] == 1 and not is_sequencing and can_sense_corner:  # corner wall
            is_sequencing = True
            speed_sequence = 0
            await hub.speaker.beep()
            
            if direction == 0:
                if rob_data['ultra_L'] > rob_data['ultra_R']: # left corner turn
                    direction = -1
                elif rob_data['ultra_L'] < rob_data['ultra_R']: # right corner turn
                    direction = 1
                
            if direction == -1:
                steer_sequence = -75 # left turn
            elif direction == 1:
                steer_sequence = 75 # right turn

            # reduce the turn amount to stop momentum from increasing the turn
            while abs(rob_data['heading']) < (90 * num_turn - 14):
                turn90_pid(num_turn, forward=True, maxP=1000, minP=300)                
                await wait(1)

            num_turn += 1
            # crucial for knowing how many laps the robot did
            
            steer_sequence = 0
            speed_sequence = 0
            is_sequencing = False
            can_sense_corner = False
            
        await wait(50)
```
* detect_corner: Waits until the OpenMV camera sets corner == 1 and a color lane line has been crossed (can_sense_corner).

Once triggered, it locks out normal wall-following (is_sequencing = True) and forces a hard turn (75 or -75 steering angle) while dynamically calculating speed using turn90_pid based on the internal Gyro heading.

Momentum Compensation: It cuts the turn sequence short by 14 degrees (90 * num_turn - 14) because the physical weight and speed of the robot will cause it to drift smoothly through the remaining 14 degrees.
```
async def sense_line_color():
    global can_sense_corner

    while True:
        # can_sense_corner allows the robot to turn at the corner
        if rob_data['line_color'] == 1 and (direction == 0 or direction == 1): # orange clockwise
            can_sense_corner = True
            await hub.speaker.beep(100)
        elif rob_data['line_color'] == 2 and (direction == 0 or direction == -1): # blue counter-clockwise
            can_sense_corner = True
            await hub.speaker.beep(100)
        
        await wait(20)
```
To ensure the camera doesn't accidentally trigger a corner turn sequence in the middle of a straightaway due to a visual glitch, This function forces the robot to physically cross a colored line (Orange for clockwise tracks, Blue for counter-clockwise tracks) before it is permitted to execute a corner turn.
```
async def motor_controller():
    global rob_data, steer, speed, is_sequencing
    global steer_center, speed_center
    global steer_sequence, speed_sequence

    while True:
        if is_sequencing:
            steer = steer_sequence
            speed = speed_sequence          
        elif rob_data['block_x'] == 0:
            steer = steer_center
            speed = speed_center 
            
        car.steer(steer)
        car.drive_speed(speed)
        
        await wait(50)
```
This function acts as the multiplexer. It checks if the robot is currently executing a corner turn (is_sequencing). If it is, it feeds the corner variables to the motors. If not, it feeds the wall-following PID values (steer_center, speed_center) to the motors.
```
async def main():
    # Run tasks concurrently
    await multitask(
        remote_engine(), 
        update_robot_data(), 
        detect_corner(),
        sense_line_color(),
        ultrasonic_PID(kp=0.0005, ki=0.0000001, kd=1.0),
        motor_controller(),
        race=True
        # if even one function stops, all function stops
    )

    car.drive_speed(0)
    car.steer(0)
    gc.collect() # <--- Force a full memory clear here!
    await wait(10) # Give the hub a moment to breathe

    rear.reset_angle(0)
    while rear.angle() < 1850:
        await UltrasonicPID_2Sensor_C(num_turn=13, kp=0.075, ki=0.000001, kd=1.0, max_steer=20, gyro_angle_correct=25, with_gyro=True)
        gc.collect()
        
    car.drive_speed(0)
```
* race=True: This Pybricks feature means that if any single task ends, all tasks are stopped. The ultrasonic_PID loop ends when num_turn > 12 (meaning the robot has finished 3 full laps of a 4-corner track).

Once the multitasking race finishes, the car drops out of the concurrent loop, stops, fires gc.collect() to free memory, and runs a standalone function (UltrasonicPID_2Sensor_C) utilizing gyro correction angles to smoothly drive forward exactly 1850 motor degrees back across the starting finish line to complete the run.


## 5.2 Obstacle Challenge
For the obstacle challenge, the robot uses a code similar to the open challenge. The camera has another roi that takes up half of the camera to detect an approaching obstacle and its color. The robot uses the ultrasonic sensors to determine which direction it needs to face after leaving the parking lot. The robot uses pid to face straight forward. When a block enters the roi, the camera also detects its color. By using the x coordinate and the color of the obstacle, the robot is able to maneuver away from the obstacle. To avoid detecting corner blocks that cause the robot to face the wrong direction, we utilized the orange and blue lines of the track. When the robot passes the orange or blue line, the robot will ignore the obstacle until it makes a turn. The robot will repeat this until num_turn is equal to 12. When num_turn reaches 12, the robot will align itself using the walls and execute a parallel parking.


<img width="100%"  alt="Untitled Diagram oc drawio_page-0001" src="https://github.com/user-attachments/assets/216c2a1c-e014-44f7-8426-ec3c51461759" />

**Logic and Behavoir**
```
p = PUPRemoteHub(Port.D) # Connection to an external openMV/SPIKE camera hub
p.add_command("msg", to_hub_fmt="repr", from_hub_fmt="repr")
p.add_channel('cam', to_hub_fmt='bhhb')
```
PUPRemoteHub handles data streaming from an external smart camera over a serial port. This camera passes information regarding obstacle colors, coordinates, and corner signs.
```
async def getMedianR(samples):
    i = 0
    while i < samples:
        distListR[i] = await eyesR.distance()
        i += 1
    distListR.sort()
    n = len(distListR)
    mid = n // 2
    if n % 2 == 1: return distListR[mid]
    else: return (distListR[mid-1] + distListR[mid]) / 2

async def getMedianL(samples):
    i = 0
    while i < samples:
        distListL[i] = await eyesL.distance()
        i += 1
    distListL.sort()
    n = len(distListL)
    mid = n // 2
    if n % 2 == 1: return distListL[mid]
    else: return (distListL[mid-1] + distListL[mid]) / 2
```
Ultrasonic sensors are prone to occasional noise or random spikes. Both getMedianR and getMedianL take 3 rapid readings, sort them, and pick the middle value (median filter). This ensures smooth, reliable data for the steering logic.

```
# --- Background Engines ---
# Relays information from the camera
async def remote_engine():
    while True:
        await p.process_async()
        await wait(1)

# update information from sensors and camera
async def update_robot_data():
    global rob_data
    while True:
        rob_data['heading'] = hub.imu.heading()
        rob_data['line_color'] = await getLineColor()
        rob_data['ultra_R'] = await getMedianR(3)
        rob_data['ultra_L'] = await getMedianL(3)

        result = await p.call_multitask("cam")
        if result:
            color, block_x, block_y, corner= result 
            rob_data['color'] = color
            rob_data['block_x'] = block_x
            rob_data['block_y'] = block_y
            rob_data['corner'] = corner
        await wait(20)

# changes motor control according to priority
async def motor_controller():
    global rob_data, is_sequencing
    global steer_center, speed_center
    global steer_sequence, speed_sequence
    
    while True:
        if is_sequencing:
             # PRIORITY 1
            current_steer = steer_sequence
            current_speed = speed_sequence
        else:
            # PRIORITY 2
            current_steer = steer_center
            current_speed = speed_center
                
        car.steer(current_steer)
        car.drive_speed(current_speed)
        
        await wait(10)
```
These asynchronous loops run constantly in the backround using Python uasyncio:

* remote_engine(): Keeps the communication channel with the camera active.

* update_robot_data(): Constantly refreshes the robot’s telemetry—its IMU (gyro) heading, line colors, ultrasonic distances, and camera tracking metrics—saving them into a global dictionary called rob_data.

* motor_controller(): Acts as a safety/priority switchboard. If the robot is executing a strict maneuver (like avoiding a block or taking a corner), is_sequencing is true, and it takes commands from steer_sequence. Otherwise, it defaults to steer_center (basic wall-centering).

```
async def get_out_parking():
    global direction
    # Auto detect layout orientation
    if await getMedianR(3) > await getMedianL(3): 
        direction = 1
    else: 
        direction = -1

    # Scale initial steering angle directly based on track direction
    car.steer(75 * direction)
    await wait(100)
    while abs(hub.imu.heading()) < 65:
        car.drive_speed(500)
        await wait(10)
    car.drive_speed(0)
    car.steer(0)
    await wait(100)
    rear.reset_angle()
    while rear.angle() < 1000:
        car.drive_speed(800)
        await wait(10)
    car.drive_speed(0)
    car.steer(75 * direction)
    await wait(100)
    
    # Watch gyro orientation bound dynamically 
    while (direction == 1 and hub.imu.heading() > 10) or (direction == -1 and hub.imu.heading() < -10):
        car.drive_speed(-500)
        await wait(10)
    car.drive_speed(0)
    car.steer(0)
```
This function runs immediately at launch. It reads the left and right sensors to see which wall is further away, automatically determining if the track runs clockwise (direction = 1) or counter-clockwise (direction = -1). It then performs a hard-coded swing maneuver to clear the starting stall and align with the track.
```
# pid in corner turns for smooth execution
async def turn90_PID_L(num_turn, kp=20.0, ki=0.0, kd=55.0, maxP=900, minP=400, forward=False):
    global sumS_error, prevS_error, direction
    global is_sequencing, speed_sequence
    base_target = 90 * num_turn
    target_angle = base_target * direction 
    sumS_error = 0
    prevS_error = 0
    while True:
        current_heading = hub.imu.heading()
        steer_error = target_angle - current_heading
        abs_error = abs(steer_error)
        # cut turn short to avoid turning over 90
        if direction == 1:
            if abs_error <= 5: 
                break
        if direction == -1:
            if abs_error <= 2:
                break
        prop = kp * abs_error
        sumS_error += abs_error
        derivative = kd * (abs_error - prevS_error)
        compensation = prop + (ki * sumS_error) + derivative
        prevS_error = abs_error
        if compensation > maxP:
            compensation = maxP
        elif compensation < minP:
            compensation = minP
        if forward:
            speed_sequence = compensation
        else:
            speed_sequence = -compensation
        await wait(10)
    speed_sequence = 0
    is_sequencing = True
```
This function uses a PID controller tied to the internal gyro (hub.imu.heading()) to seamlessly execute a precise 90-degree corner turn. It slows down dynamically as it reaches its target heading to avoid overshooting.
```
async def parallel_parking():
    await hub.speaker.beep(100,500)
    await wait(500)
    await hub.speaker.beep(100,500)

    global num_turn, speed_sequence, steer_sequence
    global distance_wall, direction

    hub.imu.reset_heading(0)
    rear.reset_angle(0)
    if direction == 1:
        while rear.angle() > -100:
            await straight_backward()
            await wait(10)
    if direction == -1:
        while rear.angle() > -100:
            await straight_backward()
            await wait(10)
    car.drive_power(0)
    car.steer(0)
    await wait(500)
    hub.speaker.beep(600,100)
    await last_turn()
    car.steer(0)
    car.drive_power(0)
    await wait(100)

    rear.reset_angle(0)
    car.steer(0)
    hub.imu.reset_heading(0)
    if direction == 1:
        while rear.angle() > int(-(distance_wall * 2.5)):
            await straight_backward() 
            await wait(10) 
    if direction == -1:
        while rear.angle() > int(-(distance_wall * 3.5)):
            await straight_backward() 
            await wait(10)
    car.drive_power(0)
    await wait(100)
    
    hub.imu.reset_heading(0)
    car.steer(0)
    await wait(100)
    rear.reset_angle(0)
    if direction == 1:
        while rear.angle() < 3330:
            await straight_forward()
            await wait(10)
    if direction == -1:
        while rear.angle() < 3325:
            await straight_forward()
            await wait(10)
    car.drive_power(0)
    rear.reset_angle(0)
    await wait(100)
    
    # difference in value from the location of the differential
    if direction == 1:
        car.steer(-75 * direction)
        while rear.angle() > -460:
            car.drive_power(-200)
            await wait(10)
        car.drive_power(0)
        await wait(200)
        car.steer(75 * direction)
        
        await wait(200)
        while rear.angle() > -1070:
            car.drive_power(-200)
            await wait(10)
            if (direction == 1 and hub.imu.heading() <= 7):
                break
    if direction == -1:
        car.steer(-75 * direction)
        while rear.angle() > -460:
            car.drive_power(-200)
            await wait(10)
        car.drive_power(0)
        await wait(200)
        car.steer(75 * direction)
        
        await wait(200)
        while rear.angle() > -1200:
            car.drive_power(-200)
            await wait(10)
            if (direction == -1 and hub.imu.heading() >= -2):
                break
            
    car.drive_power(0)
    await wait(500)
    car.steer(0)
    await wait(500)
```
parallel_parking is a long, sequential function triggered at the very end of the race (after 12 turns/3 laps). It uses a sequence of timed straight reverses, a 90-degree pivot (last_turn()), and calculated back-and-forth S-turns to cleanly park the vehicle inside the designated finish zone.
```
def avoid_blocks(color=0, kp=0.5, ki=0.000001, kd=0.1, bl_x=0.0, bl_y=0.0, max_steer_obs=55):
    global sum_Rerror, prev_Rerror, sum_Gerror, prev_Gerror
    global steer_sequence, speed_sequence, turn_right, turn_left
    
    Rerror = 65 - bl_x
    Gerror = 290 - bl_x     
    current_error = 0 
    
    if color == 2:  # Red block
        current_error = Rerror
        prop = kp * Rerror 
        sum_Rerror += Rerror
        integral = ki * sum_Rerror 
        if integral > 10.0: integral = 10.0
        derivative = kd * (Rerror - prev_Rerror) 
        compensation = prop + integral + derivative
        prev_Rerror = Rerror
        if compensation > max_steer_obs: compensation = max_steer_obs
        elif compensation < -max_steer_obs: compensation = -max_steer_obs
        
        steer_sequence = -compensation
        speed_sequence = -(45 / 28) * (abs(Rerror) - 1120 / 3) 
        turn_left = True
        turn_right = False
        
    elif color == 1:  # Green block
        current_error = Gerror
        prop = kp * Gerror 
        sum_Gerror += Gerror
        integral = ki * sum_Gerror 
        if integral > 10.0: integral = 10.0
        derivative = kd * (Gerror - prev_Gerror)
        compensation = prop + integral + derivative
        prev_Gerror = Gerror
        if compensation > max_steer_obs: compensation = max_steer_obs
        elif compensation < -max_steer_obs: compensation = -max_steer_obs
        
        steer_sequence = -compensation
        speed_sequence = -(45 / 28) * (abs(Gerror) - 1120 / 3)  
        turn_right = True
        turn_left = False
```
This function is used when it detects a block near it (color == 2 for Red, color == 1 for Green). This function calculates a PID error based on where the block is horizontally (bl_x) relative to safe zones (65 for Red, 290 for Green). It swerves the car away from the block.
```
def position_to_center(position_target, p_error, kp=6, ki=0.000001, kd=1.0, max_steer=40):
    global sump_error, prevp_error, num_turn
    global steer_sequence, speed_sequence
    p_error = position_target - hub.imu.heading()
    prop = kp * p_error 
    sump_error += p_error
    integral = ki * sump_error 
    if integral > 10.0: integral = 10.0
    elif integral < -10.0: integral = -10.0
    derivative = kd * (p_error - prevp_error) 
    compensation = prop + integral + derivative
    prevp_error = p_error
    if compensation > max_steer: compensation = max_steer
    elif compensation < -max_steer: compensation = -max_steer
    steer_sequence = compensation

# return to the middle on the track
async def return_to_center_position( num_turn, direction, gyro_angle_correct=20, max_steer = 45, color = 0):
    global prevp_error, sump_error, p_error
    global steer_sequence, speed_sequence, turn_right, turn_left
    
    if direction == 1:
        if turn_left == True: position_target = 90 * (num_turn-1) - 30
        elif turn_right == True: position_target = 90 * (num_turn-1) + 30
    elif direction == -1:
        if turn_right == True: position_target = 90 * (num_turn-1) * direction + 35
        elif turn_left == True: position_target = 90 * (num_turn-1) * direction - 35
    
    while True:
        p_error = position_target - hub.imu.heading()
        await wait(10)
        if abs(p_error) <= 5:
            steer_sequence = 0
            await wait(10)
            current_rot = rear.angle()
            while rear.angle() < current_rot + 140:
                steer_sequence = 0
                speed_sequence = 900
                await wait(10)
                if rob_data['line_color'] in [1, 2]:
                    break
            break 
        position_to_center(position_target, p_error, max_steer=max_steer)
        speed_sequence = 1100
        await wait(10)

# pid for return to center from block
def steer_to_center(target, kp=12, ki=0.000001, kd=1.0, max_steer=45):
    global sumH_error, prevH_error, steer_sequence
    h_error = target - hub.imu.heading()
    prop = kp * h_error 
    sumH_error += h_error
    integral = ki * sumH_error 
    if integral > 10.0: integral = 10.0
    elif integral < -10.0: integral = -10.0
    derivative = kd * (h_error - prevH_error) 
    compensation = prop + integral + derivative
    prevH_error = h_error
    if compensation > max_steer: compensation = max_steer
    elif compensation < -max_steer: compensation = -max_steer
    steer_sequence = compensation

# straighten out the robot after returning to center position
async def return_center_fr_block(num_turn, direction, max_steer=40):
    global steer_sequence, speed_sequence, turn_left, turn_right
    global sumH_error, prevH_error
    sumH_error = 0
    prevH_error = 0
    if direction == 0: target = 0
    else:
        if turn_left == True:
            target = (90 * (num_turn - 1)) * direction -5
        if turn_right == True:
            target = (90 * (num_turn - 1)) * direction +5
    while True:
        h_error = target - hub.imu.heading()
        if turn_left or turn_right:
            if abs(h_error) <= 10: 
                steer_sequence = 0
                break
                
        steer_to_center(target, max_steer=max_steer)
        speed_sequence = 1100
        await wait(10)
```
return_to_center_position() and return_center_fr_block() calculate a counter-heading using the Gyro sensor to steer the car diagonally back toward the middle of the track lane, straightening out once it gets there and preparing for the next block.
```
async def detect_corner():
    global num_turn, direction
    global rob_data, steer_sequence, speed_sequence, is_sequencing
    global can_sense_corner, can_sense_corner_CW, can_sense_corner_CCW
    global detect_color, distance_wall
    while True:
        if num_turn < 12:
            if direction == 1: can_sense_corner = can_sense_corner_CW
            elif direction == -1: can_sense_corner = can_sense_corner_CCW
            
            if rob_data["corner"] == 1 and not is_sequencing and can_sense_corner: 
                is_sequencing = True
                await hub.speaker.beep()
                if direction == 0:
                    if rob_data['ultra_L'] > rob_data['ultra_R']: direction = -1
                    elif rob_data['ultra_L'] < rob_data['ultra_R']: direction = 1
                if num_turn in [4, 8]:
                    rear.reset_angle(0)
                    steer_sequence = 0
                    while rear.angle() < 700: 
                        current_angle = hub.imu.heading()
                        error = (90 * (num_turn-1) * direction) - current_angle
                        if direction == 1:
                            steer_sequence = error * 1.35
                        if direction == -1:
                            steer_sequence = error * 1.65
                        speed_sequence = 750
                        await wait(10)
                    hub.imu.reset_heading(90 * (num_turn-1) * direction)
                    rear.reset_angle(0)
                    steer_sequence = 0
                    await wait(10)
                    while rear.angle() > -370:
                        speed_sequence = -800
                        await wait(10)
                else:
                    rear.reset_angle(0)
                    while rear.angle() > -150:
                        speed_sequence = -800
                        await wait(10)
                
                steer_sequence = -75 * direction
                await turn90_PID_L(num_turn, forward=False, maxP=1100, minP=400)
                num_turn += 1
                await wait(10)
                steer_sequence = 0
                await wait(200)
                rear.reset_angle(0)
                is_sequencing = False
                if direction == 1: can_sense_corner_CW = False
                elif direction == -1: can_sense_corner_CCW = False
        elif num_turn == 12:
            # initiating parking sequence
            if direction == 1: can_sense_corner = can_sense_corner_CW
            elif direction == -1: can_sense_corner = can_sense_corner_CCW
            if rob_data["corner"] == 1 and not is_sequencing and can_sense_corner:
                rear.reset_angle(0)
                while rear.angle() < 500: 
                    current_angle = hub.imu.heading()
                    error = (90 * (num_turn-1) * direction) - current_angle
                    steer_sequence = error * 5
                    speed_sequence = 800
                    await wait(10) 
                if direction == 1:
                    distance_wall = await getMedianL(3)
                if direction == -1:
                    distance_wall = await getMedianR(3)
                speed_sequence = 0
                steer_sequence = 0
                return 
        await wait(20)
```
This function monitors rob_data["corner"]. When the color sensor spots a corner line tape, it freezes normal driving (is_sequencing = True), tracks what lap/turn it is on (num_turn), adjusts its alignment, and kicks off the 90-degree PID turn. Once complete, it bumps num_turn by 1. When num_turn hits 12, it breaks out of the loop to trigger parking.
```
async def ultrasonic_PID(kp=0.001, ki=0.000001, kd=1000.0, minPower=900, maxPower=1100):
    global sum_error, prev_error, max_steer, direction, num_turn
    global rob_data, steer_center, speed_center, is_sequencing
    while num_turn <= 12:        
        if is_sequencing:
            sum_error = 0
            prev_error = 0
            await wait(20)
            continue
            
        error = rob_data['ultra_R'] - rob_data['ultra_L']    
        prop = kp * error 
        sum_error += error
        integral = ki * sum_error 
        if integral > 10.0: integral = 10.0
        derivative = kd * (error - prev_error) 
        compensation = prop + integral + derivative
        prev_error = error
        if compensation > max_steer: compensation = max_steer
        elif compensation < -max_steer: compensation = -max_steer
        maxP, minP = maxPower, minPower
        max_error = 1000
        if error > max_error: error = max_error
        elif error < 0: error = 0
        drive_power = -(maxP - minP) * (error - max_error) / max_error + minP   
        steer_center = compensation
        speed_center = drive_power                    
        await wait(20) 
    speed_center = 0
```
The error for the PID is calculated by using the two ultrasonic sensors and finding the difference between them. If the difference is zero the robot is in the center of the track. If it drifts, the PID loop outputs a correction to steer_center to bring the robot back to the middle of the lane.
```
async def sense_line_color():
    global can_sense_corner_CW, can_sense_corner_CCW, detect_color
    while True:
        if rob_data['line_color'] == 1 and (direction == 0 or direction == 1) and detect_color == True: 
            can_sense_corner_CW = True
            await hub.speaker.beep(100)
            await wait(100)
            detect_color = False
        elif rob_data['line_color'] == 2 and (direction == 0 or direction == -1): 
            can_sense_corner_CCW = True
            await hub.speaker.beep(100)
        await wait(20)
```
This function converts raw sensor values to colors. Spotting orange(1) tells the system it needs to prepare for a Clockwise corner turn; spotting blue(2) prepares it for a Counter-Clockwise turn.
```
async def main():
    await get_out_parking()
    # asynchronously controls functions
    await multitask(
        remote_engine(),  
        update_robot_data(),
        avoid_blocks_and_return_center(),
        sense_line_color(),
        detect_corner(),
        ultrasonic_PID(kp=0.003, ki=0.000001, kd=10000.0), 
        motor_controller(),
        race=True        
    )
    await parallel_parking()
    car.drive_speed(0)
    car.steer(0)
    gc.collect() 
    await wait(500) 
```
This function orchestrates everything. The multitask() function allows the robot to do PID, obstacle avoidance, and line checking at the same time. The moment detect_corner() finishes lap 3 (turn 12), the multitasking race ends, and the script moves directly to the final parallel_parking() function.


## 6.0 Engineering Factor
The third-party factor that we used is the Openmv camera, which enables us to see the blocks and their color. It gives the robot the ability to see the blocks, and in turn, make the right decisions that are needed.

<img width="40%"  alt="Screenshot 2026-06-03 093710" src="https://github.com/user-attachments/assets/67ad3cf5-2a82-496e-ae5a-0b0daca77669" />

## 6.1 Usage of Camera

**OPEN CHALLENGE**

Our Camera Code for the open challenge searches for objects and sends the data to a Lego Hub using the PUPRemote library. When it starts, it turns on a light. It locks the camera's brightness and color settings so the pictures always look the same. It also sets up a Region of Interest (ROI) to limit where the camera looks. In the main loop, the camera takes a picture and makes a copy of it. This lets the script read the data on one copy while drawing helpful lines on the screen with the other. A special function tweaks the picture's contrast to make objects stand out clearly from the background. The script checks a small ROI at the top-center of the picture for dark objects. If it finds a dark object there, it changes a status number from 0 to 1. Finally, the script puts the object data and the status number into a small group of 4 pieces of information. It sends this packet to the Lego Hub right away before starting over.
```
import sensor
from pupremote import PUPRemoteSensor
from pyb import Pin, Timer

def msg(txt):
    print(txt)
    return txt+txt

light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
light.pulse_width_percent(100)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(False)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
sensor.skip_frames(time=2000)

p = PUPRemoteSensor(power=False)
p.add_command('msg', "repr", "repr")
p.add_channel('cam', to_hub_fmt='bhhb')

img_center = (160, 120)
img_roi = (5, 110, 310, 125)
roi_rect = (img_roi[0], img_roi[1], img_roi[2], img_roi[3])
roi_left_bottom = (5, 201, 70, 35)
roi_right_bottom = (245, 201, 70, 35)
img_roi_corner = (155, 80, 10,40)

def find_block(img, img_debug, distance_cap):
    img_contrast = img.copy()
    img_contrast.gamma_corr(gamma=1.9, contrast=1.1, brightness=-0.1)
    color = 0
    nearestRed = None
    nearestGreen = None
    nearestPink = None
    red_val = 0
    green_val = 0
    pink_val = 0
    blocks = 0

    # Bright Magenta / Pink
    threshold_pink = (30, 95, 40, 127, -127, -10)

    # Just Red
    threshold_red = (20, 90, 30, 127, 15, 127)

    # Standard Green
    threshold_green = (20, 90, -127, -20, -10, 80)

    img_debug.draw_rectangle(img_roi, color=(0, 0, 255))
    red = img_contrast.find_blobs([threshold_red], area_threshold=150, roi=img_roi, merge=True)
    green = img_contrast.find_blobs([threshold_green], area_threshold=150, roi=img_roi, merge=True)
    pink = img_contrast.find_blobs([threshold_pink], area_threshold=150, roi=img_roi, merge=True)

    center_x = 0
    center_y = 0
    p_center_x = 0
    p_center_y = 0

    if red:
        for b in red:
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 0))
            center_x = b.x() + (b.w() // 2)
            center_y = b.y() + (b.h() // 2)
            img_debug.draw_cross(center_x, center_y, color=(255, 0, 0))
            val = b.y() + b.h()
            if val > distance_cap:
                blocks += 1
            if val > red_val:
                nearestRed = b
                red_val = val

    if green:
        for b in green:
            img_debug.draw_rectangle(b.rect(), color=(0, 255, 0))
            center_x = b.x() + (b.w() // 2)
            center_y = b.y() + (b.h() // 2)
            img_debug.draw_cross(center_x, center_y, color=(0, 255, 0))
            val = b.y() + b.h()
            if val > distance_cap:
                blocks += 1
            if val > green_val:
                nearestGreen = b
                green_val = val

    if pink:
        for b in pink:
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 255))
            p_center_x = b.x() + (b.w() // 2)
            p_center_y = b.y() + (b.h() // 2)
            img_debug.draw_cross(p_center_x, p_center_y, color=(255, 0, 255))
            val = b.y() + b.h()
            if val > distance_cap:
                blocks += 1
            if val > pink_val:
                nearestPink = b
                pink_val = val

    if red_val == 0 and green_val == 0:
        block = {"center_x": center_x, "center_y": center_y, "color": 0, "p_center_x": p_center_x, "p_center_y": p_center_y}
        return block

    if nearestRed and nearestGreen:
        if red_val >= green_val:
            color = 2
        else:
            color = 1
    elif nearestRed:
        color = 2
    elif nearestGreen:
        color = 1
    else:
        color = 0

    block = {"center_x": center_x, "center_y": center_y, "color": color, "p_center_x": p_center_x, "p_center_y": p_center_y}
    return block

def check_corner_roi(img, img_debug):
    img_contrast = img.copy()
    img_contrast.gamma_corr(gamma=1.9, contrast=1.1, brightness=-0.1)
    threshold_black = (0, 37, -128, 15, -54, 37)
    img_debug.draw_rectangle(img_roi, color=(0, 0, 255))
    img_debug.draw_rectangle(img_roi_corner, color=(255, 255, 0))
    black = img_contrast.find_blobs([threshold_black], area_threshold=150, roi=img_roi_corner, merge=True)
    if black:
        black_val = 1
    else:
        black_val = 0
    corner = {"black": black_val}
    return corner

while True:
    img_debug = sensor.snapshot()
    img = img_debug.copy()
    img_debug.draw_cross(160, 120, color=(0, 0, 0))
    block = find_block(img, img_debug, 30)
    corner = check_corner_roi(img, img_debug)
    data = (block["color"], block["center_x"], block["center_y"], corner["black"])
    p.update_channel('cam', *data)
    p.process()
```

**OBSTACLE CHALLENGE**

How our camera code works for obstacle challenge is like this:
This MicroPython script allows an OpenMV smart camera to track colored objects and send that tracking data to a LEGO control hub. At startup, it turns on an external LED light at full brightness to ensure consistent, stable lighting for the camera. The camera is configured to a 320 x 240 QVGA resolution and locks its exposure and white balance so changing room lights won't mess up its vision. It defines two "Regions of Interest" (ROIs): a wide middle strip to look for blocks and a tiny central box to look for a black corner marker. In the main loop, the camera takes a snapshot and enhances its contrast and gamma to make specific colors stand out sharply. The find_block function searches the wide strip for color clusters matching pre-defined ranges for Red, Green, and Pink.If multiple blocks are visible, the script calculates which one is lowest on the screen, recognizing it as the closest block to the robot.The check_corner_roi function looks strictly inside the tiny central box to see if the robot has driven over a black marker line. All of this gathered information is packed into a compact 4-part data packet containing the closest color ID, its coordinates, and the black line status. Finally, the script continuously transmits this data stream over a PUPRemote channel so the connected LEGO robot can steer toward the blocks or react to the corner.

```   
import sensor
import time
from pupremote import PUPRemoteSensor
from pyb import Pin, Timer

# ===========================================================================
# Tunables
# ===========================================================================
DEBUG = True             # prints FPS to the OpenMV IDE; does NOT touch PUP data.
FPS_PRINT_EVERY = 30     # print fps once every N frames when DEBUG is on.

LED_DUTY = 30            # LED brightness %.

# Lock exposure / gain to fixed values to fight MOTION BLUR.
EXPOSURE_US = None
GAIN_DB = None

# --- IMAGE PROCESSING LAYER ---
USE_CLAHE = True         # Normalizes local lighting spots caused by your LEDs.
CLAHE_CLIP_LIMIT = 3     # High numbers = more contrast, lower numbers = less noise.

AREA_THRESHOLD = 150     #minimum number of pixels needed to detect block.

# Gamma / contrast applied to the analysis frame before find_blobs.
GAMMA = 1.9
CONTRAST = 1.1
BRIGHTNESS = -0.1

# ===========================================================================
# Colour thresholds (LAB) -- BLOCK AND CORNER DETECTION
# ===========================================================================
threshold_pink  = (30, 95, 40, 127, -127, -10)   # bright magenta / pink
threshold_red   = (20, 90, 30, 127, 15, 127)     # red
threshold_red2 = (0, 37, 11, 55, 6, 127)
threshold_green = (20, 90, -128, -29, -1, 44)    # green
# threshold_green2 = (19, 39, -92, -19, 6, 38)
threshold_black = (0, 37, -25, 0, -19, 8)   # black (corner marker)
black2 = (0, 20, -7, -2, -14, 4)

# ===========================================================================
# ROIs FOR CORNER AND BLOCK DETECTION
# ===========================================================================
img_center = (160, 120)
img_roi = (0, 110, 320, 130)
roi_rect = (img_roi[0], img_roi[1], img_roi[2], img_roi[3])
roi_left_bottom = (5, 201, 70, 35)
roi_right_bottom = (245, 201, 70, 35)
img_roi_corner = (155, 110, 10, 80)


# ===========================================================================
# PUPRemote Command Callback
# ===========================================================================
def msg(txt):
    # Callback handler for PUPRemote 'msg' command received from the Hub.
    # PUPRemote requires a return value matching the 'repr' format.
    print(txt)
    return txt + txt


# ===========================================================================
# LED Control
# ===========================================================================
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
light.pulse_width_percent(LED_DUTY)

# ===========================================================================
# Camera Sensor Initialization
# ===========================================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(False)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
sensor.skip_frames(time=2000)

# Optional explicit locks, applied AFTER skip_frames so they actually stick.
if GAIN_DB is not None:
    sensor.set_auto_gain(False, gain_db=GAIN_DB)
if EXPOSURE_US is not None:
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)

# ===========================================================================
# PUPRemote Communication
# ===========================================================================
p = PUPRemoteSensor(power=True)
p.add_command('msg', "repr", "repr")
p.add_channel('cam', to_hub_fmt='bhhb')


def _bbox_center(b):
    return (b.x() + (b.w() // 2), b.y() + (b.h() // 2))


def find_block(img_proc, img_debug):
    # Detect the nearest red / green / pink block.

    # img_proc  : Shared gamma-corrected and CLAHE analysis frame.
    # img_debug : Raw overlay frame displayed in OpenMV IDE.

    nearestRed = None
    nearestGreen = None
    nearestPink = None
    red_val = 0
    green_val = 0
    pink_val = 0

    img_debug.draw_rectangle(img_roi, color=(0, 0, 255))

    red = img_proc.find_blobs([threshold_red, threshold_red2], area_threshold=AREA_THRESHOLD, roi=img_roi, merge=True)
    green = img_proc.find_blobs([threshold_green], area_threshold=AREA_THRESHOLD, roi=img_roi, merge=True)
    pink = img_proc.find_blobs([threshold_pink], area_threshold=AREA_THRESHOLD, roi=img_roi, merge=True)

    if red:
        for b in red:
            cx, cy = _bbox_center(b)
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 0))
            img_debug.draw_cross(cx, cy, color=(255, 0, 0))

            # Proximity calculation: Y + H equals the bottom edge of the blob.
            # Higher pixel value = lower down in the frame = closer to the robot camera.
            val = b.y() + b.h()
            if val > red_val:
                red_val = val
                nearestRed = b

    if green:
        for b in green:
            cx, cy = _bbox_center(b)
            img_debug.draw_rectangle(b.rect(), color=(0, 255, 0))
            img_debug.draw_cross(cx, cy, color=(0, 255, 0))

            # Proximity calculation: Higher pixel value = closer to the robot camera.
            val = b.y() + b.h()
            if val > green_val:
                green_val = val
                nearestGreen = b

    if pink:
        for b in pink:
            cx, cy = _bbox_center(b)
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 255))
            img_debug.draw_cross(cx, cy, color=(255, 0, 255))

            # Pink is actively tracked to isolate it from red, preventing false red positives.
            val = b.y() + b.h()
            if val > pink_val:
                pink_val = val
                nearestPink = b

    # Pick dominant colour based on closest block (lowest in frame)
    # COLOR MAPPING KEY FOR HUB: 0 = None, 1 = Green, 2 = Red
    if nearestRed and nearestGreen:
        color = 2 if red_val >= green_val else 1
    elif nearestRed:
        color = 2
    elif nearestGreen:
        color = 1
    else:
        color = 0

    if color == 2:
        center_x, center_y = _bbox_center(nearestRed)
    elif color == 1:
        center_x, center_y = _bbox_center(nearestGreen)
    else:
        center_x, center_y = 0, 0

    if nearestPink is not None:
        p_center_x, p_center_y = _bbox_center(nearestPink)
    else:
        p_center_x, p_center_y = 0, 0

    return {
        "center_x": center_x,
        "center_y": center_y,
        "color": color,
        "p_center_x": p_center_x,
        "p_center_y": p_center_y,
    }


def check_corner_roi(img_proc, img_debug):
    img_debug.draw_rectangle(img_roi_corner, color=(255, 255, 0))
    black = img_proc.find_blobs([threshold_black], area_threshold=100, roi=img_roi_corner, merge=True)
    return {"black": 1 if black else 0}


# ===========================================================================
# Main Execution Loop
# ===========================================================================
clock = time.clock()
_frame = 0

while True:
    clock.tick()
    img_debug = sensor.snapshot()

    # Image preparation layer
    img_proc = img_debug.copy()
    img_proc.gamma_corr(gamma=GAMMA, contrast=CONTRAST, brightness=BRIGHTNESS)

    if USE_CLAHE:
        img_proc.histeq(adaptive=True, clip_limit=CLAHE_CLIP_LIMIT)

    img_debug.draw_cross(160, 120, color=(0, 0, 0)) # place cross on the center of the camera

    block = find_block(img_proc, img_debug)
    corner = check_corner_roi(img_proc, img_debug)

    # NOTE: Pink block data is purposely omitted from 'data' tuple because the
    # 'cam' PUPRemote channel format ('bhhb') is only configured to send 4 values.
    data = (block["color"], block["center_x"], block["center_y"], corner["black"])
    p.update_channel('cam', *data)
    p.process()

    if DEBUG:
        _frame += 1
        if _frame >= FPS_PRINT_EVERY:
            print("fps:", clock.fps())
            _frame = 0

    # Give the hardware background handler a tiny window to service the USB link
    time.sleep_ms(1)
```

## 7.0 Engineering decisions and Improvements
This segment explains the challenges, both in the build and programming, when making the robot and the improvements made to clear the challenges.

## 7.1 Robot Construction
The first problem we had was when our robot's ultrasonic sensors were infront of the wheels, which made the timing of the turn too early. To solve that, we made an entirely new design with the sensors on top of the wheels to help the robot turn better. We also switch from cantilever to braced for better stability on the wheels.

| | |
|:---|:---|
|<img width="50%" alt="a910624c-00f3-4ee8-b32c-c6e39e3e3a63" src="https://github.com/user-attachments/assets/db19628c-23a0-4470-924d-ea5b12803162" />|<img width="30%" alt="6" src="https://github.com/user-attachments/assets/d8fe5bac-05d1-4e37-9ee0-4aa5278ebcee" />|
|**OLD ROBOT**|**CURRENT ROBOT**|

## 7.2 Robot Programming
We had many problems in our old code and we made many improvements since then. We managed to complete the parking and made the AvoidBlocks command more consistent. We have also managed to make it go both clockwise and counterclockwise.

https://drive.google.com/file/d/1IwvF9whDRN1dr9YeoEUpp2IKtazWew4s/view (old code)

[https://drive.google.com/file/d/1YzoAbaAszqIbo_PungKgBqTQ1u02eVbx/view?usp=sharing
](https://drive.google.com/file/d/1YzoAbaAszqIbo_PungKgBqTQ1u02eVbx/view?usp=sharing) (current code)

### Disclaimer
Gemini and Claude were used to enchance the formulated code and as an inspiration for the github by providing examples. Claude was used to improve the threshold of the colors and Gemini was used to help build the code in specific parts by providing an example and was modified by us to complete the challenge.
