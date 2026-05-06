from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.robotics import Car
from pupremote_hub import PUPRemoteHub



hub = PrimeHub()

# Set up motors.
rear = Motor(Port.B, Direction.CLOCKWISE)
steer = Motor(Port.C, Direction.CLOCKWISE)

# Initialize the sensor
# eyes = UltrasonicSensor(Port.F)
eyesR = UltrasonicSensor(Port.F)
eyesL = UltrasonicSensor(Port.E)
eyesF = UltrasonicSensor(Port.A)

# OpenMV camera
p = PUPRemoteHub(Port.D)
p.add_command("msg", to_hub_fmt="repr", from_hub_fmt="repr")
p.add_channel('color', to_hub_fmt='f')  # char (0: none, 1: green, 2: red)
p.add_channel('bl_x', to_hub_fmt='f')  # center x of rectangle
p.add_channel('bl_y', to_hub_fmt='f')  # center y of rectangle
p.add_channel('cor', to_hub_fmt='f')  # presence of corner
# p.add_channel('lba', to_hub_fmt='b')  # integer presence of red block at left bottom area
# p.add_channel('rba', to_hub_fmt='b')  # integer presence of green block at right bottom area

hub.imu.reset_heading(0)
wait(500)

car = Car(steer, rear, 100)
wait(500)

# error => eyes.distance() - target

targetDist = 100
sum_error = 0
prev_error = 0

sum_Gerror = 0
prev_Gerror = 0
sum_Rerror = 0 
prev_Rerror = 0

sumS_error = 0
prevS_error = 0

distList = [0, 0, 0]
distListR = [0, 0, 0]
distListL = [0, 0, 0]
distListF = [0, 0, 0]

max_steer = 20 
# num_turn = 1
direction = 0

drive_power = 0.0
compensation = 0.0

def getMedian():
    i = 0
    while i < samples:
        distList[i] = eyes.distance()
        i += 1
    
    distList.sort()
    n = len(distList)
    mid = n // 2

    if n % 2 == 1:
        return distList[mid]
    else:
        return (distList[mid-1] + distList[mid]) / 2

def getMedianR(samples):
    i = 0
    while i < samples:
        distListR[i] = eyesR.distance()
        i += 1
    
    distListR.sort()
    n = len(distListR)
    mid = n // 2

    if n % 2 == 1:
        return distListR[mid]
    else:
        return (distListR[mid-1] + distListR[mid]) / 2

def getMedianL(samples):
    i = 0
    while i < samples:
        distListL[i] = eyesL.distance()
        i += 1
    
    distListL.sort()
    n = len(distListL)
    mid = n // 2

    if n % 2 == 1:
        return distListL[mid]
    else:
        return (distListL[mid-1] + distListL[mid]) / 2

def getMedianF(samples):
    i = 0
    while i < samples:
        distListF[i] = eyesF.distance()
        i += 1
    
    distListF.sort()
    n = len(distListF)
    mid = n // 2

    if n % 2 == 1:
        return distListF[mid]
    else:
        return (distListF[mid-1] + distListF[mid]) / 2

def UltrasonicPID_2Sensor(kp=0.05, ki=0.000001, kd=100000.0, gyro_angle_correct=25):

    global sum_error, prev_error, max_steer, direction, corner
    num_turn = 1

    drive_power = 0.0
    compensation = 0.0
    
    while num_turn <= 12:

        corner = p.call('cor')  # 0 -> no corner or 1 -> corner 

        error = getMedianR(3) - getMedianL(3)
        # print("error: ", error)
        # print("prev_error: ", prev_error)

        if hub.imu.heading() > gyro_angle_correct + (90 * (num_turn-1)):
            while hub.imu.heading() > gyro_angle_correct//2 + (90 * (num_turn-1)) and corner == 0:
                corner = p.call('cor')  # 0 -> no corner or 1 -> corner
                car.steer(-30)
                car.drive_speed(800)
            
        if hub.imu.heading() < -gyro_angle_correct - (90 * (num_turn-1)):
            while hub.imu.heading() < -gyro_angle_correct//2 -(90 * (num_turn-1)) and corner == 0:
                corner = p.call('cor')  # 0 -> no corner or 1 -> corner
                car.steer(30)
                car.drive_speed(800)
            
        if getMedianF(3) < 450 and corner == 1:  # corner wall
            car.drive_speed(0)
            hub.speaker.beep()
            # wait(250)

            if direction == 0:
                if getMedianL(3) > getMedianR(3): # left corner turn
                    direction = -1
                    
                elif getMedianL(3) < getMedianR(3): # right corner turn
                    direction = 1
                
            if direction == -1:
                car.steer(-75)  # left turn
            elif direction == 1:
                car.steer(75)   # right turn
            
            while abs(hub.imu.heading()) < (90 * num_turn - 10):
                car.drive_speed(800)

            num_turn += 1
            # print("num_turn: ", num_turn)
            car.drive_speed(0)
            car.steer(0)
            # wait(250)


        prop = kp * error # proportional
        
        sum_error += error
        integral = ki * sum_error # integral

        if integral > 10.0:
            integral = 10.0

        derivative = kd * (error - prev_error) # derivative

        compensation = prop + integral + derivative

        prev_error = error

        # negative steer = steer to the right 
        # positive steer = steer to the left
        
        if compensation > max_steer:
            compensation = max_steer
        elif compensation < -max_steer:
            compensation = -max_steer
            
        car.steer(compensation)
        # print("compensation: ", compensation)

        # drive power range => 300 - 800
        # compensation range => 0 - max_compensation
        maxP = 1100
        minP = 900
        max_error = 2000
        
        if error > max_error:
            error = max_error
        elif error < 0:
            error = 0

        drive_power = -(maxP - minP) * (error - max_error) / max_error + minP   
        # print("drive_power: ", drive_power)
        car.drive_speed(drive_power)
        wait(10)

    # finishing
    rear.reset_angle(0)
    while rear.angle() < 2250:
        UltrasonicPID_2Sensor_C(num_turn=13, kp=0.05, ki=0.000001, kd=10000, max_steer=20, gyro_angle_correct=25)

    car.drive_speed(0)

# open challenge
UltrasonicPID_2Sensor(kp=0.05, ki=0.000001, kd=100000.0, gyro_angle_correct=25)
