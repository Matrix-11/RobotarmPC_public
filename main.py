import pygame
from spatialmath import SE3
import numpy as np
import com
from utils import trigger
import robotarmDH
import serial.tools.list_ports
import time

robot = robotarmDH.MYROBOT()

COM = "COM3"

print("Ready")


def ik(pose):
    sol = robot.ikine_LMS(pose)  # Inverse Kinematics
    if not sol.success:
        print("no IK solution found")
        return None, 1 # Returns error (1)

    print("IK solution found")
    pos = sol.q  # np.delete(sol.q, 0)
    pos = np.degrees(pos) # Convert radians to degrees
    # Real joints of Robot have different 0 positions this needs to be converted
    pos[1] += 90
    pos[2] = (pos[2] - 90) * -1
    pos[3] += 90
    pos[4] += 180
    pos = np.round(pos, 2)  # Rounds posion so no unecessary fractions are send
    pos = pos.tolist()  # Convert np array to py list
    print(pos)

    return pos, 0  # Returns joint pos and no error (0)


pygame.init()
print(pygame.joystick.get_count())
joystick = pygame.joystick.Joystick(0)
joystick.init()

#pos = 0
arduino_found = False
newData = False
lastSend = []
lastSpeed = 500
send_checksum = None  # Global checksum
received_checksum = None
#x = 0
#y = 0
#pitch = 0
#roll = 0
#con = False
check_val = True

soll_pose = [0, 0, 0, 0, 0]  # (x,y,z, knick, dreh)
curr_pose = [0, 0, 0, 0, 0]  # pose to fall back if ik is wrong
rotation = 0
home = False


if input() == "S":

    myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
    while not arduino_found:  # Waits till arduino is found on specified COM port
        myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
        print(myports)

        for port in myports:
            if COM in port:
                arduino_found = True
                break
        print("Arduino not found")
        time.sleep(0.5)

    print("Arduino found")
    arduino = serial.Serial(port=COM, baudrate=250000, timeout=0.1, write_timeout=0)
    time.sleep(2)
    arduino.write('S'.encode('utf-8'))  # Starts microcontroller which will start homing automatically

    while True:
        received_checksum = com.recieve(arduino)

        if received_checksum is not None:  # There is a received checksum (Arduino Responded)
            print(received_checksum)
            check_val = True  # Next command can be send
        if received_checksum is not None and received_checksum != send_checksum:  # There is a received checksum BUT it is incorrect
            print("Checksum not correct! Resending...")
            send_checksum = com.send(lastSend, lastSpeed, arduino)
            check_val = False

        if joystick.get_button(0):  # If controller button is pressed sends robot to home position
            home = True

        if check_val:  # New pose will only be generated and send if arduino is ready to receive a new one
            change = False
            button = pygame.event.get()
            deadzone = 0.11
            pose_impact = (0.0006, 0.0006, 0.5, 0.4, 0.4)  # How much an input changes the pose
            min_change = 0.1  # Min Change in [mm] which has to be achieved by input else robot won't move (to not clog the com)
            min_change *= 0.001  # convert to meters
            pose_change = (joystick.get_axis(0), -joystick.get_axis(1), trigger(joystick, deadzone), joystick.get_axis(2), joystick.get_axis(3))
            # Gets controller joystick position
            for i in range(0, len(pose_change)):
                round(pose_change[i], 6)
                if not (deadzone > pose_change[i] > -deadzone):  # Joystick input is never 0 because of potentiometers so a deadzone is necessary
                    soll_pose[i] += pose_impact[i] * pose_change[i]
                    change = True
                    print(f"Sollpose: {soll_pose}")

            if not (deadzone > pose_change[2] > - deadzone):  # Rotation of the Base is directly set with angles and not over ik
                rotation += pose_impact[2] * pose_change[2]
            if change or home:
                speed = int(2000 * abs(max(pose_change, key=abs, default=0.25)))  # Key=abs only runs for the function and does not change the value
                #  max speed is 2000 steps/s * the biggest analog controller input (float 0-1)
                if home:  # Home position of the robot
                    speed = 1000
                    soll_pose = [0, 0, 0, 0, 0]
                    rotation = 0
                    home = False

                pose = SE3(0.1 + soll_pose[0], 0, 0.2 + soll_pose[1]) * SE3.RPY([0, 90, 180], unit='deg', order='xyz') * SE3.Ry(soll_pose[3], 'deg') * SE3.Rz(soll_pose[4], 'deg')  # 0,90,180 offset for correct default orientation of the robot end

                pos, err = ik(pose)  # Converts cartesian (pose) to joint values (pos)
                if err == 0:
                    rotation = round(rotation, 2)
                    if rotation > 180:  # Prevents infinite turning of the robot
                        rotation = 180
                    elif rotation < -180:
                        rotation = -180
                    pos[0] = rotation  # Rotation which is not calculated over ik is now added to the pos which will be send
                    curr_pose = soll_pose
                    lastSend = pos  # So pos and speed can be resend
                    lastSpeed = speed
                    send_checksum = com.send(pos, speed, arduino)  # Sending data to arduino 
                    check_val = False
                else:  # if ik has found no solution
                    soll_pose = curr_pose
                    print(soll_pose)
