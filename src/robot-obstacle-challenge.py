from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task
from pybricks.robotics import Car
from pupremote_hub import PUPRemoteHub
import gc

# --- Initialization ---
hub = PrimeHub()

rear = Motor(Port.B, Direction.CLOCKWISE)
steer = Motor(Port.C, Direction.CLOCKWISE)

eyesR = UltrasonicSensor(Port.F)
eyesL = UltrasonicSensor(Port.E)
colorF = ColorSensor(Port.A)

p = PUPRemoteHub(Port.D)
p.add_command("msg", to_hub_fmt="repr", from_hub_fmt="repr")
p.add_channel('cam', to_hub_fmt='bhhb')

hub.imu.reset_heading(0)
wait(500)
car = Car(steer, rear, 100)
wait(500)

# --- Global Variables ---
targetDist = 100
sum_error, prev_error = 0, 0
sum_Gerror, prev_Gerror = 0, 0
sum_Rerror, prev_Rerror = 0, 0
sumS_error, prevS_error = 0, 0
sumH_error, prevH_error = 0, 0
sump_error, prevp_error = 0, 0

distListR = [0, 0, 0]
distListL = [0, 0, 0]
distListF = [0, 0, 0]

num_turn = 1
max_steer = 20 
direction = 1
distance_wall = 0

drive_power = 0.0
compensation = 0.0
lineColor = 0
p_error = 0

can_sense_corner = False
can_sense_corner_CW = False
can_sense_corner_CCW = False
detect_color = True
is_sequencing = False
turn_left = False
turn_right = False
avoidance = False

rob_data = {
    "color": 0,
    "block_x": 0,
    "block_y": 0,
    "corner": 0,
    "heading": 0,
    "ultra_R": 0,
    "ultra_L": 0,
    "line_color": 0
}

steer_center, speed_center = 0, 0
steer_sequence, speed_sequence = 0, 0

# --- Core Sensor Functions ---
# Get samples to stabilize input from ultrasonic sensor
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

# Detects orange or blue lines in the corner section
async def getLineColor():
    global detect_color
    lineColor = 0
    hsv = await colorF.hsv()
    if detect_color:
        if hsv[0] < 20 and hsv[1] > 50: lineColor = 1  # orange
        elif hsv[0] > 200 and hsv[1] > 50: lineColor = 2  # blue
    else:
        lineColor = 0
    return lineColor

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

# --- Maneuver Functions ---

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

# use gyro to move straight forward
async def straight_forward():
    current_angle = hub.imu.heading()
    error = 0 - current_angle
    car.steer(error * 2.5)
    car.drive_power(800) 

# use gyro to move straight backward
async def straight_backward():
    current_angle = hub.imu.heading()
    error = 0 - current_angle
    car.steer(-(error * 2.5))
    car.drive_power(-800) 

# Gyro assist to make the turn as close to 90 degrees as possible
async def last_turn(forward=False, maxP=600, minP=150, kp=10.0, ki=0.0001, kd=55.0):
    global direction
    await hub.speaker.beep(600, 100)
    target_angle = 90 * direction
    car.steer(-75 * direction)
    car.drive_power(-200) 
    while abs(hub.imu.heading()) < abs(target_angle): 
        await wait(10)
        car.drive_power(-200)
        if (direction == 1 and hub.imu.heading() >= 84) or (direction == -1 and hub.imu.heading() <= -85):
            break

# this function uses car.steer and car.drive_power because it is outside of the multitask
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

# --- Block Avoidance & Recovery Functions ---

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

    return current_error

# Calculates steering PID correction to head back toward center target
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

# --- Navigation and Detection logic ---

# detecting corner, resetting gyro after each lap and initiating parking sequence
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

# Allows the robot to turn at corners
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

# combined function for the whole avoiding block and recovery process
async def avoid_blocks_and_return_center():
    global is_sequencing, steer_sequence, speed_sequence, detect_color, turn_left, turn_right
    global sum_Gerror, prev_Gerror, sum_Rerror, prev_Rerror, sump_error, prevp_error
    
    while True:
        if direction == 1: current_corner_active = can_sense_corner_CW
        elif direction == -1: current_corner_active = can_sense_corner_CCW
        else: current_corner_active = False

        if current_corner_active or (is_sequencing and rob_data['block_x'] == 0 and speed_sequence != 800):
            await wait(50)
            continue 

        if rob_data['block_x'] != 0.0 and rob_data['block_y'] > 100 and not is_sequencing:            
            is_sequencing = True
            avoidance = True
            sum_Gerror, prev_Gerror = 0, 0
            sum_Rerror, prev_Rerror = 0, 0
            sump_error, prevp_error = 0, 0
            
            # --- STEP 1: CAM LOOP ---
            while rob_data['block_x'] != 0.0:
                avoid_blocks(
                    color=rob_data['color'], 
                    bl_x=rob_data['block_x'], 
                    bl_y=rob_data['block_y'], 
                    max_steer_obs=50
                )
                await wait(10)
                if (direction == 1 and rob_data['line_color'] == 1) or (direction == -1 and rob_data['line_color'] == 2):
                    break
            await hub.speaker.beep(600,10)
            # --- STEP 2: CLEARANCE BUFFER ---
            current_rot = rear.angle()
            while rear.angle() < current_rot + 120:
                speed_sequence = 900
                await wait(10)

            # --- STEP 3: DIAGONAL RECOVERY ---
            await return_to_center_position(num_turn, direction, max_steer=45)
            
            # --- STEP 4: FLATTEN ALIGNMENT ---
            await return_center_fr_block(num_turn, direction, max_steer=45)
            
            # turn_left = False
            turn_right = False
            detect_color = True
            is_sequencing = False 
            avoidance = False
        else:
            pass

        await wait(20)

# --- Execution ---

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

run_task(main())
