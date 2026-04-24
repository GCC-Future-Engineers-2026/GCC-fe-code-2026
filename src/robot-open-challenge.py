from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.robotics import Car



hub = PrimeHub()

# Set up motors.
rear = Motor(Port.B, Direction.CLOCKWISE)
steer = Motor(Port.C, Direction.CLOCKWISE)

# Initialize the sensor
# eyes = UltrasonicSensor(Port.F)
eyesR = UltrasonicSensor(Port.F)
eyesL = UltrasonicSensor(Port.E)
eyesF = UltrasonicSensor(Port.A)

hub.imu.reset_heading(0)
wait(1000)

car = Car(steer, rear, 100)
wait(1000)

# error => eyes.distance() - target

targetDist = 100
sum_error = 0
prev_error = 0

kp = 0.035
ki = 0.00001
kd = 1000000.0

distList = [0, 0, 0]
distListR = [0, 0, 0]
distListL = [0, 0, 0]
distListF = [0, 0, 0]

max_steer = 20
num_turn = 1
direction = 0

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


def UltrasonicPID_RSensor():

    while True:

        # error = getMedianR(3) - getMedianL(3)
        error = getMedian(3) - targetDist
        
        #print("error: ", error)

        if error > 500:  # corner
            car.drive_speed(0)
            hub.speaker.beep()
            car.steer(0)
            # car.drive_speed(200)
            # wait(100)
            # car.drive_speed(0)
            car.steer(max_steer)
            while getMedian(3) > targetDist:
                car.drive_speed(300)
            
            car.steer(0)
            car.drive_speed(0)
            wait(250)


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

        # drive power range => 300 - 800
        # compensation range => 0 - max_compensation
        maxP = 700
        minP = 200
        max_error = 2000 - targetDist
        
        drive_power = -(maxP - minP) * (error - max_error) / max_error + minP   
        # print("drive_power: ", drive_power)
        car.drive_speed(drive_power)
        wait(10)


def UltrasonicPID_2Sensor():

    global kp, ki, kd, sum_error, prev_error, max_steer, num_turn, direction

    drive_power = 0.0
    compensation = 0.0
    
    while num_turn <= 12:

        error = getMedianR(3) - getMedianL(3)
        #print("error: ", error)

        while hub.imu.heading() > 20 + (90 * (num_turn-1)):
            car.steer(-30)
            car.drive_speed(600)
        
        while hub.imu.heading() < -20 - (90 * (num_turn-1)):
            car.steer(30)
            car.drive_speed(600)

        if getMedianF(3) < 275:  # corner wall
            car.drive_speed(0)
            hub.speaker.beep()
            wait(250)

            if direction == 0:
                if getMedianL(3) > getMedianR(3): # left corner turn
                    direction = -1
                    
                elif getMedianL(3) < getMedianR(3): # right corner turn
                    direction = 1
                
            if direction == -1:
                car.steer(-75)  # left turn
            elif direction == 1:
                car.steer(75)   # right turn
            
            while abs(hub.imu.heading()) < (90 * num_turn):
                car.drive_speed(400)

            num_turn += 1
            # print("num_turn: ", num_turn)
            car.drive_speed(0)
            car.steer(0)
            wait(500)


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

        # drive power range => 300 - 800
        # compensation range => 0 - max_compensation
        maxP = 1100
        minP = 800
        max_error = 2000
        
        if error > max_error:
            error = max_error
        elif error < 0:
            error = 0

        drive_power = -(maxP - minP) * (error - max_error) / max_error + minP   
        # print("drive_power: ", drive_power)
        car.drive_speed(drive_power)
        wait(10)

    car.drive_speed(0)

# UltrasonicPID_RSensor()
UltrasonicPID_2Sensor()

# while True:
    # print("LeftUS: ", getMedianL(3))
    # print("RightUS: ", getMedianR(3))
    # print("Error: ", getMedianR(3) - getMedianL(3))
    # print("LeftUS: ", eyesL.distance())
    # print("RightUS: ", eyesR.distance())
    # print("Error: ", eyesR.distance() - eyesL.distance())
    # wait(1000)





