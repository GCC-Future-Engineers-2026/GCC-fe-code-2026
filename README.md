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
This documentation cantains comprehensive information about the robot's design, mobility management, power and sensor management, wirng diagram, materials needed, third-party factors and components, obstacle detection and avoidance, and the decisions and improvements made in preparation made for Philippine Robot Olympiad (PRO) located in Mall of Asia (MOA), Pasay in Philippines 2026 under the Future Engineers category.
## Team Grace Christian College
From the Philippines, Team Grace Christian College is a three-member team made to compete in the PRO 2026 in Mall of Asia. 

<img width="30%" alt="Messenger_creation_9F03665D-973D-4025-9D37-10E926EF3D5B" src="https://github.com/user-attachments/assets/1ff4be49-f7dc-450e-8937-915cecd19cf5" />


Its members are Thomas Lao, Yuri Valero, and Kyron Chen.
## 1.0 Design of self-driving car
<img align="right" width="20%" alt="NewProject (1)" src="https://github.com/user-attachments/assets/30526914-fc04-4f29-b330-35a93779b463" />This autonomous self driving car is made with Lego Education Spike Prime Set. This segment also explains the tradeoffs and the reason components were used in the design.






## 1.1 Tradeoffs
We have had multiple tradeoffs due to the changes we made with the design. We increased the value of steering, but we had to make the robot slower as a result. We also moved the motor to a different position, so we could have better weight distribution. We moved the ultrasonic sensor to on top of the wheel, so that our robot doesn't turn too early. Our robot was also braced for better stability, but the tradeoff is our robot's width is longer.

| | |
|:---|:----|
|<img width="400" alt="a910624c-00f3-4ee8-b32c-c6e39e3e3a63" src="https://github.com/user-attachments/assets/b93fe076-9378-4c38-8162-7b4818c5dacb" />|<img width="400" alt="Messenger_creation_D390C557-0934-4E3C-927C-248851FC92B4" src="https://github.com/user-attachments/assets/20168924-10ae-49ed-8a9c-1ee4d92aadc4" />|
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

We used this wheel compared to others, because of the advantage that it's a single piece, which prevents the tire from slipping off unlike others. The tire is also made of hard rubber, which prevents significant friction amd allows the wheel to slide smoothly over the track, requiring less motor power to steer.

**2. Implementation of motor**

Our robot consists of two Lego Technic motors, one being medium angular motor (vertical) and the other being large angular motor (horizontal). The medium (vertical) motor faces inwards to further makes the robot compact to reduce overall length. This keeps our robot compact and saves space, reduces turning radius for sharper turns.

**3. Use of differential gear**

The differential gear is integrated into a drive axle that lets the wheels rotate at different speeds. This stops the wheels from locking up and helps the vehicle turn smoothly and stay stable. This is especially helpful when turning in both challenges.

<img width="25%" alt="Messenger_creation_2DD2113F-E5B7-4058-AAD0-0F9EC8B3905B" src="https://github.com/user-attachments/assets/cebe4825-b5ef-4ca0-b05a-08d2a869794f" />

**4. Engineering principle for steering**

This robot is designed using Ackermann steering geomatry.

<img width="50%" alt="Messenger_creation_A0C92DA4-F460-49D0-89A5-i" src="https://github.com/user-attachments/assets/9bf7b1c8-57c6-436e-9b63-53230b88d505" />

This setup turns the inside front wheel at a sharper angle than the outside one, so both wheels follow perfect turning circles. By stopping the tires from scrubbing sideways, it cuts down on friction and wear. This gives the vehicle better traction, more stability, and the ability to make sharp turns with great precision.

## 3.0 Power and sense management
This segment is dedicated to the vehicle's power supply and usage and its sensor systems. It covers each sensor's implementation and use along with information about the robot's power supply. There is also a wiring diagram given to illustrate the connection of sensors.
## 3.1 Power source
<img align="right" width="25%"  alt="Screenshot 2026-06-03 100053" src="https://github.com/user-attachments/assets/41e484c0-4b90-4e50-833b-3ca5c74df42b" />The robot is powered by the SPIKE Prime Hub, which features a 2,000 mAh rechargeable lithium-ion battery. It was selected due to the fact that it's easily replaced. It is also at peak voltage and performance for most of its run time, until it dies.




## 3.2 Sensors
**Camera**-The Openmv camera’s function is to detect the blocks and the parking lot. It detects red and green blocks while the robot is in motion and helps it turn left or right. It also helps find the magenta walls after the final turn to help in parking.

<img width="30%" alt="Messenger_creation_AD349745-06A4-4AB2-90DD-00" src="https://github.com/user-attachments/assets/f628ebe3-e2fc-4a4b-8981-00676a236fde" />



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
| <img width="50%" alt="1_1x" src="https://github.com/user-attachments/assets/f3cceb3e-253b-43ba-812d-94d8529ee561" />   |  <img width="50%" alt="2_1x" src="https://github.com/user-attachments/assets/0bd6561f-894e-4f64-bc83-675f4917325d" />|
|<img width="50%"  alt="3_1x" src="https://github.com/user-attachments/assets/413b0dee-93e2-4fd8-8dad-41786d5d16ff" />|<img width="50%"  alt="4_1x" src="https://github.com/user-attachments/assets/7addeef9-6667-4b4b-9354-3cbaf85d6dcd" />|
|<img width="50%"  alt="5_1x" src="https://github.com/user-attachments/assets/143b200c-cbdb-48f3-b3be-90abbd787416" />|<img width="50%"  alt="6_1x" src="https://github.com/user-attachments/assets/c36ddfa5-75a0-4929-87d9-ca7b0532607b" />|
|<img width="50%"  alt="7_1x" src="https://github.com/user-attachments/assets/f8e9fc2b-816a-4514-a795-741bc61bf91c" />|<img width="50%" alt="8_1x" src="https://github.com/user-attachments/assets/419fdb8b-06dd-4a49-aac2-c06aa4218bb3" />|
|<img width="50%" alt="9_1x" src="https://github.com/user-attachments/assets/9b65a57b-a64e-44ca-8dc6-35cb8eb9cf87" />|<img width="50%" alt="10_1x" src="https://github.com/user-attachments/assets/9502aabe-9f72-4ce6-8305-32c238abf729" />|
|<img width="50%" alt="11_1x" src="https://github.com/user-attachments/assets/6008cee4-e8c3-4524-a991-0a3e848fa3c2" />|<img width="50%" alt="12_1x" src="https://github.com/user-attachments/assets/26dd841a-fee4-4ba8-af47-9ba428c27f89" />|
|<img width="50%" alt="13_1x" src="https://github.com/user-attachments/assets/f0868a4b-7455-4998-8800-b32e6c61ca79" />|<img width="50%" alt="14_1x" src="https://github.com/user-attachments/assets/89007af1-ac86-4092-b8d8-bcb97dd0b8aa" />|
|<img width="50%" alt="15_1x" src="https://github.com/user-attachments/assets/548794b1-2450-48f6-abdf-1dfd8c1d9a87" />|<img width="50%" alt="16_1x" src="https://github.com/user-attachments/assets/bda688e5-bc83-4e8c-bee4-c0bf544bebe2" />|
|<img width="50%" alt="17_1x" src="https://github.com/user-attachments/assets/56371cb0-ceee-4bc4-a866-aab517a04a1f" />|<img width="50%" alt="18_1x" src="https://github.com/user-attachments/assets/f9603227-c89a-4b51-aa6d-004d32ff2114" />|
|<img width="50%" alt="19_1x" src="https://github.com/user-attachments/assets/335a94ed-ce04-4e60-8dbd-63184ddb53c6" />|<img width="50%" alt="20_1x" src="https://github.com/user-attachments/assets/9c3193be-5343-4221-b023-a3d6a444f534" />|
|<img width="50%" alt="21_1x" src="https://github.com/user-attachments/assets/a8d0fbec-5231-4f84-84a6-85af097baaca" />|<img width="50%" alt="22_1x" src="https://github.com/user-attachments/assets/26edbb12-e130-4cce-9e7a-a19e208e3b22" />|
|<img width="50%" alt="23_1x" src="https://github.com/user-attachments/assets/29cbf26c-e81b-45be-b7af-5d0248ad48d8" />|<img width="50%" alt="24_1x" src="https://github.com/user-attachments/assets/7e70396f-a26c-48df-8f95-27917264efb7" />|
|<img width="50%" alt="25_1x" src="https://github.com/user-attachments/assets/2d0ea535-cff9-4aea-8f3d-ce1327748931" />|<img width="50%" alt="26_1x" src="https://github.com/user-attachments/assets/97650260-3dd7-4d01-af07-ceffc7e59b43" />|
|<img width="50%" alt="27_1x" src="https://github.com/user-attachments/assets/5298ecaf-2bab-4169-ad5d-4e1754797fe6" />|<img width="50%" alt="28_1x" src="https://github.com/user-attachments/assets/555054a3-e8dc-4947-b389-8dc0af052ce7" />|
|<img width="50%" alt="29_1x" src="https://github.com/user-attachments/assets/42ecdd6b-7b0c-429b-9e00-5434efc0fdce" />|<img width="50%" alt="30_1x" src="https://github.com/user-attachments/assets/28927bce-1ba9-4f83-a28f-56fb6aa1eb5e" />|
|<img width="50%" alt="31_1x" src="https://github.com/user-attachments/assets/290cce00-b42c-42d2-818f-5caf03284bfa" />|<img width="50%" alt="32_1x" src="https://github.com/user-attachments/assets/d35322a1-4b14-4efd-8f95-4a9bb0aea48f" />|
|<img width="50%" alt="33_1x" src="https://github.com/user-attachments/assets/2ce8f0d1-ec4c-493a-9b17-9cea26e2bb7a" />|<img width="50%" alt="34_1x" src="https://github.com/user-attachments/assets/f484745b-69d0-411f-be69-e145dd4936ef" />|
|<img width="50%" alt="35_1x" src="https://github.com/user-attachments/assets/411da1a7-f278-40fa-b9ea-b021d5c0d4bd" />|<img width="50%" alt="36_1x" src="https://github.com/user-attachments/assets/0fd42c62-88e6-4e8f-8987-e5a9374c2fda" />|
|<img width="50%" alt="37_1x" src="https://github.com/user-attachments/assets/eea557c2-8e78-4549-96d1-454f220af569" />|<img width="50%" alt="38_1x" src="https://github.com/user-attachments/assets/8d16174f-8a0f-43ef-95a3-35805e910185" />|
|<img width="50%" alt="39_1x" src="https://github.com/user-attachments/assets/9ad226d6-b12f-499f-905c-21760d7976e1" />|<img width="50%" alt="40_1x" src="https://github.com/user-attachments/assets/92c0fb08-b193-4512-9a97-237ee0e10da6" />|
|<img width="50%" alt="41_1x" src="https://github.com/user-attachments/assets/12b81755-93bb-4323-8525-44b513dd113a" />|<img width="50%" alt="42_1x" src="https://github.com/user-attachments/assets/52c7ffd8-2a4f-4785-842e-105499ffccd3" />|
|<img width="50%" alt="43_1x" src="https://github.com/user-attachments/assets/67c41d2a-b586-4726-a2bb-25ead98bc629" />|<img width="50%" alt="44_1x" src="https://github.com/user-attachments/assets/fdcd9c89-2d68-4f72-a50a-5863b2effee3" />|
|<img width="50%" alt="45_1x" src="https://github.com/user-attachments/assets/e209323f-e9aa-49aa-af94-1294880c568b" />|<img width="50%" alt="46_1x" src="https://github.com/user-attachments/assets/06411233-d0d5-4f67-bccf-580b3c6f0898" />|
|<img width="50%" alt="47_1x" src="https://github.com/user-attachments/assets/ee355010-60d4-4a4c-9101-1122f1c3ad5a" />|<img width="50%" alt="48_1x" src="https://github.com/user-attachments/assets/164312d6-dfdf-4ff8-ab2a-69b2c3b85bb0" />|
|<img width="50%" alt="49_1x" src="https://github.com/user-attachments/assets/253402ff-48be-4bff-be56-fc78144b6bca" />|<img width="50%" alt="50_1x" src="https://github.com/user-attachments/assets/8f07aa41-e00e-4eaf-b5ea-ac1445ea494e" />|
|<img width="50%" alt="51_1x" src="https://github.com/user-attachments/assets/5797014b-0a01-46ab-a8b2-31f6b5a20cff" />|<img width="50%" alt="52_1x" src="https://github.com/user-attachments/assets/9a5155d8-7a8c-48ad-a5dc-5bd7b266fbfb" />|
|<img width="50%" alt="53_1x" src="https://github.com/user-attachments/assets/542aa0df-0072-4145-95e0-c0a92039bd6e" />|<img width="50%" alt="54_1x" src="https://github.com/user-attachments/assets/ba97fb5a-cb5d-43c2-b6a3-3c8a6637f19a" />|
|<img width="50%" alt="55_1x" src="https://github.com/user-attachments/assets/d4b13ea6-766f-4b16-b041-8bd1f392c775" />|<img width="50%" alt="56_1x" src="https://github.com/user-attachments/assets/6e211262-2499-441a-9dac-968172aab161" />|
|<img width="50%" alt="57_1x" src="https://github.com/user-attachments/assets/98771cfa-04b8-46a2-b1d5-032fff55efdb" />|<img width="50%" alt="58_1x" src="https://github.com/user-attachments/assets/c8a5b288-7551-4b52-addd-cb7f17899525" />|
|<img width="50%" alt="59_1x" src="https://github.com/user-attachments/assets/c6b32809-9bea-4814-9706-5e4a4e03a1ee" />|<img width="50%" alt="60_1x" src="https://github.com/user-attachments/assets/47cfa23e-1ecc-4185-9d64-176bcc59dc9b" />|
|<img width="50%" alt="61_1x" src="https://github.com/user-attachments/assets/9fa3f543-8f44-4d02-b6b2-2ace27a2bbda" />|<img width="50%" alt="62_1x" src="https://github.com/user-attachments/assets/0fc6684c-42b4-45dd-b53e-70292f1604ba" />|
|<img width="50%" alt="63_1x" src="https://github.com/user-attachments/assets/c712bf4d-a054-4712-9b4b-ad0c3966b520" />|<img width="50%" alt="64_1x" src="https://github.com/user-attachments/assets/ab061f8b-8441-467b-af72-df7ac1ad1e64" />|
|<img width="50%" alt="65_1x" src="https://github.com/user-attachments/assets/9b2dd4b7-f466-470b-93b1-f77c5c586db3" />|<img width="50%" alt="66_1x" src="https://github.com/user-attachments/assets/b49c5aed-9ec3-4fdf-b481-55c79c267973" />|
|<img width="50%" alt="67_1x" src="https://github.com/user-attachments/assets/9e6d4bcc-23e2-4c19-a00b-600dac0cd31c" />|<img width="50%" alt="68_1x" src="https://github.com/user-attachments/assets/bb9c61df-3bb1-464b-a72b-212dcc3ffd94" />|
|<img width="50%" alt="69_1x" src="https://github.com/user-attachments/assets/9586db77-9f9f-43d5-9253-e5d67233f421" />|<img width="50%" alt="70_1x" src="https://github.com/user-attachments/assets/66aef2ea-43d7-4b6f-973f-eadfd52747b5" />|
|<img width="50%" alt="71_1x" src="https://github.com/user-attachments/assets/a19bc04b-96b6-4e0b-bd22-572d19860625" />|<img width="50%" alt="72_1x" src="https://github.com/user-attachments/assets/5c7a4a88-c4ed-4269-9282-c895b33be342" />|
|<img width="50%" alt="73_1x" src="https://github.com/user-attachments/assets/70834dba-fe5c-4b8e-8fc0-f561a10ec1b6" />|<img width="50%" alt="74_1x" src="https://github.com/user-attachments/assets/a8e203b5-a207-4452-a11b-a5013531a463" />|
|<img width="50%" alt="75_1x" src="https://github.com/user-attachments/assets/a00a4772-b906-4ae4-a144-21ecea0e0f28" />|<img width="50%" alt="76_1x" src="https://github.com/user-attachments/assets/b7f2493a-8417-4603-b15f-5cd6651fec5a" />|
|<img width="50%" alt="77_1x" src="https://github.com/user-attachments/assets/88decec9-34d3-4b04-912f-32afff94d5f3" />|<img width="50%" alt="78_1x" src="https://github.com/user-attachments/assets/09ec11ba-5e75-40ce-90ba-95fac46a8d66" />|
|<img width="50%" alt="79_1x" src="https://github.com/user-attachments/assets/59eb1874-f87a-4de8-9bf1-55bd0f9c913e" />|<img width="50%" alt="80_1x" src="https://github.com/user-attachments/assets/040fb641-9d63-48a4-8023-c780bc4dbb71" />|
|<img width="50%" alt="81_1x" src="https://github.com/user-attachments/assets/fc7100b3-0e38-408f-96ba-ac1ca689edd7" />|<img width="50%" alt="82_1x" src="https://github.com/user-attachments/assets/0f8b5f60-5d4c-4038-8933-7b9647a18b64" />|
|<img width="50%" alt="83_1x" src="https://github.com/user-attachments/assets/59668013-6aea-4d55-85ee-97d0e3384456" />|<img width="50%" alt="84_1x" src="https://github.com/user-attachments/assets/f37aece1-b058-450b-8b91-b6b08e314702" />|
|<img width="50%" alt="85_1x" src="https://github.com/user-attachments/assets/a6bfb7c0-ab4f-4b58-bc29-b54c47a2d9c2" />|<img width="50%" alt="86_1x" src="https://github.com/user-attachments/assets/3d0bbbe0-4d92-40bf-883b-464983b91ad1" />|
|<img width="50%" alt="87_1x" src="https://github.com/user-attachments/assets/e6ae375f-94fd-43a0-859e-438726983073" />|<img width="50%" alt="88_1x" src="https://github.com/user-attachments/assets/810737e6-1b1e-426f-bf80-3ecf27692791" />|
|<img width="50%" alt="89_1x" src="https://github.com/user-attachments/assets/472847e2-5ffc-45b1-bd1c-7004e673e502" />|<img width="50%" alt="90_1x" src="https://github.com/user-attachments/assets/a3fd0004-0c45-4472-b402-481a840bc1e8" />|
|<img width="50%" alt="91_1x" src="https://github.com/user-attachments/assets/1f2ef701-34d1-480b-931f-bd0096719250" />|<img width="50%" alt="92_1x" src="https://github.com/user-attachments/assets/dc30a15e-4f69-4cb7-a55f-91eb6556c3e1" />|
|<img width="50%" alt="93_1x" src="https://github.com/user-attachments/assets/e3bf16e3-3b7d-4a3d-8cfc-3c3eb258b7e2" />|<img width="50%" alt="94_1x" src="https://github.com/user-attachments/assets/c460e8ac-235f-4123-8b9c-345e24b3def5" />|
|<img width="50%" alt="95_1x" src="https://github.com/user-attachments/assets/4438b42e-3eb6-4245-a254-09c3f6fd39ec" />|<img width="50%" alt="96_1x" src="https://github.com/user-attachments/assets/292e44aa-1800-444d-8fe6-a92a1379d11d" />|
|<img width="50%" alt="97_1x" src="https://github.com/user-attachments/assets/67d2e949-5c7d-41f8-891c-4fc327db4ce5" />|<img width="50%" alt="98_1x" src="https://github.com/user-attachments/assets/bfae834a-da40-4793-8dc5-d5b00f17ee77" />|
|<img width="50%" alt="99_1x" src="https://github.com/user-attachments/assets/ecaa6a98-ed52-48a5-85ba-43640dff91b1" />| |


**Build of Materials (BOM)**

<img width="681"  alt="1" src="https://github.com/user-attachments/assets/b8ed1118-b3d9-4f30-ab26-05c053ef5fe6" />

<img width="681"  alt="2" src="https://github.com/user-attachments/assets/7b2eadc5-2375-424f-855f-ea0d53e0bde7" />

<img width="681"  alt="3" src="https://github.com/user-attachments/assets/05a990f5-272b-48f6-9ee5-37110ae32c16" />

<img width="681"  alt="4" src="https://github.com/user-attachments/assets/ee085c06-05a4-4825-b020-9d09e5129dff" />

|Component |QTY |PHOTO|
|:---|:---|
|OpenMV H7 Plus Camera|1|<img width="30%" alt="490818226-ca1d665c-cde8-455f-a26d-12d1ab6409a1" src="https://github.com/user-attachments/assets/7c5ee0b8-33c3-4ba0-bd69-2e91ee1c7bdc" />|


## 5.0 Obstacle Management
For this project, we used Python as the coding language to operate the robot. Our robot uses two different codes that both utilize a camera to complete the open and obstacle challenge. This section includes the code overview and a flowchart for the open challenge and obstacle challenge.
## 5.1 Open Challenge
The robot uses two ultrasonic sensors on its left and right, an openmv camera to trigger the turning, and the in-built gyro sensor to assist its turns and face the correct direction. At the start of the challenge, the robot will use pid to navigate towards a corner. The robot has a roi (region of interest) at the middle of its camera to detect if it is getting close to a corner. When that roi detects black, the robot will execute a 90 degree turn by reversing. The robot determines the direction of the turn based on the difference of the left and right ultrasonic sensor. After completing a turn, a variable called “num_turn” will increase by one. The robot uses the absolute heading value to determine where to face after turning and during pid. While the variable is less than 12, the robot will continue to repeat the pid and turning code. When the variable reaches 12, the robot will continue to go forward for a few second with pid towards its starting section.

<img width="100%" alt="Untitled Diagram drawio_page-0001" src="https://github.com/user-attachments/assets/4eb3d02e-a88e-4a4b-9331-e58ade7158f8" />

For the code, our robot 


## 5.2 Obstacle Challenge
For the obstacle challenge, the robot uses a code similar to the open challenge. The camera has another roi that takes up half of the camera to detect an approaching obstacle and its color. The robot uses the ultrasonic sensors to determine which direction it needs to face after leaving the parking lot. The robot uses pid to face straight forward. When a block enters the roi, the camera also detects its color. By using the x coordinate and the color of the obstacle, the robot is able to maneuver away from the obstacle. To avoid detecting corner blocks that cause the robot to face the wrong direction, we utilized the orange and blue lines of the track. When the robot passes the orange or blue line, the robot will ignore the obstacle until it makes a turn. The robot will repeat this until num_turn is equal to 12. When num_trun reaches 12, the robot will align itself using the walls and execute a parallel parking.


<img width="100%"  alt="Untitled Diagram oc drawio_page-0001" src="https://github.com/user-attachments/assets/216c2a1c-e014-44f7-8426-ec3c51461759" />


## 6.0 Engineering Factor
The third-party factor that we used is the Openmv camera, which enables us to see the blocks and their color. It gives the robot the ability to see the blocks, and in turn, make the right decisions that are needed.

<img width="60%"  alt="Screenshot 2026-06-03 093710" src="https://github.com/user-attachments/assets/67ad3cf5-2a82-496e-ae5a-0b0daca77669" />

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
img_roi_corner = (155, 110, 10, 40)

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

## 7.0 Engineering decisions and Improvements
This segment explains the challenges, both in the build and programming, when making the robot and the improvements made to clear the challenges.

## 7.1 Robot Construction
The first problem we had was when our robot's ultrasonic sensors were infront of the wheels, which made the timing of the turn too early. To solve that, we made an entirely new design with the sensors on top of the wheels to help the robot turn better. We also switch from cantilever to braced for better stability on the wheels.

| | |
|:---|:---|
|<img width="50%" alt="a910624c-00f3-4ee8-b32c-c6e39e3e3a63" src="https://github.com/user-attachments/assets/db19628c-23a0-4470-924d-ea5b12803162" />|<img width="30%" alt="6" src="https://github.com/user-attachments/assets/3f1d291a-ebfb-4a48-8985-a5daa8f508fd" />|
|**OLD ROBOT**|**CURRENT ROBOT**|

## 7.2 Robot Programming
We had many problems in our old code and we made many improvements since then. We managed to complete the parking and made the AvoidBlocks command more consistent. 

https://drive.google.com/file/d/1IwvF9whDRN1dr9YeoEUpp2IKtazWew4s/view (old code)
