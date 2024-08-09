from time import sleep
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import cv2
import numpy as np
from identify_circle import identificar_circulos_amarelos
from send_ned_velocity import send_ned_velocity

# Constants
TAKEOFF_ALTITUDE = 2
SPEED = 0.5  # Speed in m/s (adjust as needed)
SPEED_FIND_CENTER = 0.1 
SLEEP_INTERVAL = 0.1  # Interval to send commands

# Function to get IMU data
def get_imu_data(drone):
    message = drone.recv_match(type='HIGHRES_IMU', blocking=True)
    if message:
        message = message.to_dict()
        ax = message['xacc']
        ay = message['yacc']
        az = message['zacc']
        return ax, ay, az
    return None, None, None

# Function to capture frame
def capture_frame(cap):
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Failed to capture frame from camera.")
    return frame

# Function to process frame
def process_frame(frame):
    imagem, circulo, direcao = identificar_circulos_amarelos(frame)
    cv2.waitKey(1)
    return imagem, circulo, direcao

# Connect to drone
drone = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# Wait for a heartbeat
drone.wait_heartbeat()

# Inform user
print("Connected to system:", drone.target_system, ", component:", drone.target_component)

# Capture video from webcam
cap = cv2.VideoCapture(0)

# Ensure the drone is in GUIDED mode
print("Setting mode to GUIDED \n")
drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    dialect.MAV_CMD_DO_SET_MODE,
    0,
    1,  # Base mode: GUIDED
    4,  # Custom mode
    0, 0, 0, 0, 0
)

# Arm the motors
print("Setting motors armed \n")
drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    utility.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # Command
    0,
    1,  # Arm
    0,
    0, 0, 0, 0, 0
)

# Create takeoff command
takeoff_command = dialect.MAVLink_command_long_message(
    target_system=drone.target_system,
    target_component=drone.target_component,
    command=dialect.MAV_CMD_NAV_TAKEOFF,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=TAKEOFF_ALTITUDE
)

# Create land command
land_command = dialect.MAVLink_command_long_message(
    target_system=drone.target_system,
    target_component=drone.target_component,
    command=dialect.MAV_CMD_NAV_LAND,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

# Create stop the movement command
stop_command = dialect.MAVLink_set_position_target_local_ned_message(
    time_boot_ms=0,
    target_system=drone.target_system,
    target_component=drone.target_component,
    coordinate_frame=dialect.MAV_FRAME_LOCAL_NED,
    type_mask=0b111111111000,  # Stop movement
    x=0, y=0, z=0,
    vx=0, vy=0, vz=0,
    afx=0, afy=0, afz=0,
    yaw=0, yaw_rate=0
)

# Takeoff the drone
drone.mav.send(takeoff_command)
print("Sent takeoff command to drone \n")

# Check takeoff success using IMU data
while True:
    try:
        frame = capture_frame(cap)
        imagem, circulo, direcao = process_frame(frame)
        
        ax, ay, az = get_imu_data(drone)
        if ax is not None and ay is not None and az is not None:
            estimated_altitude = az * SLEEP_INTERVAL
            print("Estimated Altitude", estimated_altitude, "meters")
            if estimated_altitude >= TAKEOFF_ALTITUDE:
                print("Takeoff to", TAKEOFF_ALTITUDE, "meters is successful")
                break
    except RuntimeError as e:
        print(e)
        break

# Moving forward and looking for the platform
print("Moving forward and looking for the platform \n")

while True:
    try:
        frame = capture_frame(cap)
        imagem, circulo, direcao = process_frame(frame)
        if circulo:
            print("Platform located \n")
            break

        ax, ay, az = get_imu_data(drone)
        if ax is not None and ay is not None and az is not None:
            print(f"IMU data: ax={ax}, ay={ay}, az={az}")
            # Use IMU data to adjust the movement if necessary

        forward_command = dialect.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=0,
            target_system=drone.target_system,
            target_component=drone.target_component,
            coordinate_frame=dialect.MAV_FRAME_LOCAL_NED,
            type_mask=0b0000111111000111,  # Ignore position, only use velocity components
            x=0.0, y=0.0, z=0.0,
            vx=float(SPEED), vy=0.0, vz=0.0,
            afx=0.0, afy=0.0, afz=0.0,
            yaw=0.0, yaw_rate=0.0
        )
        drone.mav.send(forward_command)
        sleep(SLEEP_INTERVAL)
    except RuntimeError as e:
        print(e)
        break

drone.mav.send(stop_command)
print("Stopped forward movement \n")

# Centralizing the platform
print("Centralizing the platform \n")

while True:
    try:
        frame = capture_frame(cap)
        imagem, circulo, direcao = process_frame(frame)
        if circulo:
            if direcao == "Acima":
                send_ned_velocity(drone, SPEED_FIND_CENTER, 0, 0, 1)
                print("Sending forward command")
            elif direcao == "Abaixo":
                send_ned_velocity(drone, -SPEED_FIND_CENTER, 0, 0, 1)
                print("Sending back command")
            elif direcao == "Direita":
                send_ned_velocity(drone, 0, SPEED_FIND_CENTER, 0, 1)
                print("Sending right command")
            elif direcao == "Esquerda":
                send_ned_velocity(drone, 0, -SPEED_FIND_CENTER, 0, 1)
                print("Sending left command")
            elif direcao == "Centro":
                print("Platform centralized successfully")
                break
            sleep(1)
        
        ax, ay, az = get_imu_data(drone)
        if ax is not None and ay is not None and az is not None:
            print(f"IMU data: ax={ax}, ay={ay}, az={az}")
            # Use IMU data to adjust the movement if necessary
    except RuntimeError as e:
        print(e)
        break

drone.mav.send(stop_command)
print("Stopped movement \n")

# Land the drone                                                   
drone.mav.send(land_command)
print("Sent land command to drone")

# Check if landing is successful using IMU data
while True:
    try:
        frame = capture_frame(cap)
        imagem, circulo, direcao = process_frame(frame)
        
        ax, ay, az = get_imu_data(drone)
        if ax is not None and ay is not None and az is not None:
            # Estimate altitude using IMU data
            estimated_altitude = az * SLEEP_INTERVAL
            print("Estimated Altitude", estimated_altitude, "meters")
            if estimated_altitude < 1:
                break
    except RuntimeError as e:
        print(e)
        break

# Wait some seconds to ensure landing
sleep(2)
print("Landed successfully")
cap.release()
cv2.destroyAllWindows()
