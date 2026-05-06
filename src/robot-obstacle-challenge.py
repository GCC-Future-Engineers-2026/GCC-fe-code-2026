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

def UltrasonicPID_2Sensor_C(num_turn, kp=0.05, ki=0.000001, kd=10000, max_steer=20, gyro_angle_correct=25):
    
    global sum_error, prev_error, direction, drive_power, compensation #, corner

    # corner = p.call('cor')  # 0 -> no corner or 1 -> corner

    error = getMedianR(3) - getMedianL(3)

    # print("IMU Heading: ", hub.imu.heading())
    
    if hub.imu.heading() > gyro_angle_correct + (90 * (num_turn-1)):
        while hub.imu.heading() > gyro_angle_correct//2 + (90 * (num_turn-1)): # and corner == 0:
            # corner = p.call('cor')  # 0 -> no corner or 1 -> corner
            car.steer(-30)
            car.drive_speed(800)
        
    if hub.imu.heading() < -gyro_angle_correct - (90 * (num_turn-1)):
        while hub.imu.heading() < -gyro_angle_correct//2 -(90 * (num_turn-1)): # and corner == 0:
            # corner = p.call('cor')  # 0 -> no corner or 1 -> corner
            car.steer(30)
            car.drive_speed(800)
        
    prop = kp * error # proportional
    
    sum_error += error
    integral = ki * sum_error # integral

    if integral > 10.0:
        integral = 10.0

    derivative = kd * (error - prev_error) # derivative

    compensation = prop + integral + derivative

    prev_error = error

    if compensation > max_steer:
        compensation = max_steer
    elif compensation < -max_steer:
        compensation = -max_steer
        
    car.steer(compensation)
    # print("compensation: ", compensation)

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


def avoidBlocks(color=0, kp=0.5, ki=0.000001, kd=0.1, bl_x=0.0, bl_y=0.0, max_steer_obs=50):

    global sum_Rerror, prev_Rerror, sum_Gerror, prev_Gerror

    if bl_x == 0.0:
        return

    # print("bl_x: ", bl_x)

    Rerror = 80 - bl_x
    Gerror = 260 - bl_x
    
    # print("Rerror: ", Rerror)

    if color == 2:  # red
        prop = kp * Rerror # proportional
        
        sum_Rerror += Rerror
        integral = ki * sum_Rerror # integral

        if integral > 10.0:
            integral = 10.0

        derivative = kd * (Rerror - prev_Rerror) # derivative

        compensation = prop + integral + derivative

        prev_Rerror = Rerror

        # negative steer = steer to the right 
        # positive steer = steer to the left
        
        if compensation > max_steer_obs:
            compensation = max_steer_obs
        elif compensation < -max_steer_obs:
            compensation = -max_steer_obs
        
        car.steer(-compensation)

        # assumed max power = 600
        # assumed min power = 150

        Rdrive_power = -(45 / 28) * (abs(Rerror) - 1120 / 3)
        car.drive_speed(Rdrive_power)

    elif color == 1:  # green
        prop = kp * Gerror # proportional
        
        sum_Gerror += Gerror
        integral = ki * sum_Gerror # integral

        if integral > 10.0:
            integral = 10.0

        derivative = kd * (Gerror - prev_Gerror) # derivative

        compensation = prop + integral + derivative

        prev_Gerror = Gerror

        # negative steer = steer to the right 
        # positive steer = steer to the left
        
        if compensation > max_steer_obs:
            compensation = max_steer_obs
        elif compensation < -max_steer_obs:
            compensation = -max_steer_obs
            
        # print("compensation: ", -compensation)
        car.steer(-compensation)

        # assumed max power = 600
        # assumed min power = 150
        Gdrive_power = -(45 / 28) * (abs(Gerror) - 1120 / 3)
        car.drive_speed(Gdrive_power)

def turn90_pid(num_turn, kp=35, ki=0.000001, kd=1.0):
    global sumS_error, prevS_error
    maxS = 1100
    minS = 400
    maxS_error = 90
    steer_error = (90 * (num_turn)) - hub.imu.heading()
    steer_power = (steer_error * (maxS - minS) / maxS_error + minS)
    prop = kp * steer_error # proportional
    
    sumS_error += steer_error
    integral = ki * sumS_error # integral

    if integral > 10.0:
        integral = 10.0

    derivative = kd * (steer_error - prevS_error) # derivative

    compensation = prop + integral + derivative

    prevS_error = steer_error
    if compensation < minS:
        compensation = minS
    elif compensation > maxS:
        compensation = maxS
    
    car.drive_speed(-compensation)

num_turn = 1
prev_block_x = 0.0
gyro_angle_correct = 25

while True:

    color = p.call('color')
    block_x = p.call('bl_x')
    block_y = p.call('bl_y')
    cor = p.call('cor')  # 0 -> no corner or 1 -> corner 
    
    # print("No. Turn: ", num_turn, "Color: ", color, "  Block X: ", block_x, "  Block Y: ", block_y, " corner:", cor)    
    
    if prev_block_x == 0.0 and block_x == 0.0: # no blocks
        UltrasonicPID_2Sensor_C(num_turn, kp=0.00015, ki=0.0, kd=0.5, max_steer=20, gyro_angle_correct=25)         
    
    elif prev_block_x != 0.0 and block_x == 0.0:  # just cleared a block

        hub.speaker.beep()
        print("IMU Heading: ", hub.imu.heading())
        
        if direction == 0:
            if hub.imu.heading() > 0: 
                # red, cw
                while hub.imu.heading() > gyro_angle_correct//4: # and corner == 0:
                    car.steer(-30)
                    car.drive_speed(800)
            elif hub.imu.heading() < 0: 
                # green, cw
                while hub.imu.heading() < gyro_angle_correct//4: # and corner == 0:
                    car.steer(30)
                    car.drive_speed(800)
        else:
            if hub.imu.heading() > (90 * (num_turn-1)) and direction == 1: 
                # red, cw
                while hub.imu.heading() > gyro_angle_correct//4 + (90 * (num_turn-1)): # and corner == 0:
                # while hub.imu.heading() > (90 * (num_turn-1)): # and corner == 0:
                    # corner = p.call('cor')  # 0 -> no corner or 1 -> corner
                    car.steer(-30)
                    car.drive_speed(800)
            elif hub.imu.heading() < (90 * (num_turn-1)) and direction == 1: 
                # green, cw
                while hub.imu.heading() < gyro_angle_correct//4 + (90 * (num_turn-1)): # and corner == 0:
                # while hub.imu.heading() > (90 * (num_turn-1)): # and corner == 0:
                    # corner = p.call('cor')  # 0 -> no corner or 1 -> corner
                    car.steer(30)
                    car.drive_speed(800)
                
            elif hub.imu.heading() < -(90 * (num_turn-1)) and direction == -1:
                # green, ccw
                while hub.imu.heading() < -gyro_angle_correct//4 - (90 * (num_turn-1)): # and corner == 0:
                # while hub.imu.heading() < -(90 * (num_turn-1)): # and corner == 0:
                    # corner = p.call('cor')  # 0 -> no corner or 1 -> corner
                    car.steer(30)
                    car.drive_speed(800)
            elif hub.imu.heading() > -(90 * (num_turn-1)) and direction == -1:
                # red, ccw
                while hub.imu.heading() > -gyro_angle_correct//4 - (90 * (num_turn-1)): # and corner == 0:
                # while hub.imu.heading() < -(90 * (num_turn-1)): # and corner == 0:
                    # corner = p.call('cor')  # 0 -> no corner or 1 -> corner
                    car.steer(-30)
                    car.drive_speed(800)

    else:
        avoidBlocks(color, bl_x=block_x, bl_y=block_y, max_steer_obs=50)
        
    if getMedianF(3) < 175 and cor == 1:  # corner wall
        hub.speaker.beep(100)
        car.drive_speed(0)
        car.steer(0)
        wait(250) 

        rear.reset_angle(0)
        while rear.angle() > -225:
            car.drive_speed(-800)

        if direction == 0:
            if getMedianL(3) > getMedianR(3): # left corner turn
                direction = -1
                    
            elif getMedianL(3) < getMedianR(3): # right corner turn
                direction = 1 
                
        if direction == -1:  # counterclockwise
            while hub.imu.heading() > -(90 * (num_turn)):
                car.steer(75)  # left turn
                turn90_pid(num_turn, kp=10, ki=0.000001, kd=1.0)
        
        elif direction == 1:
            while hub.imu.heading() < (90 * (num_turn)):
                car.steer(-75)   # right turn
                turn90_pid(num_turn, kp=10, ki=0.000001, kd=1.0)
        
        num_turn += 1
        car.steer(0)
        car.drive_speed(0)
        wait(250)
    
    # print("prev_block_x: ", prev_block_x, "block_x: ", block_x)
    
    prev_block_x = block_x
