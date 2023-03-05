import pygame
from spatialmath import SE3
import numpy as np
import com
from utils import trigger
import robotarmDH
import serial.tools.list_ports
import time

robot = robotarmDH.MYROBOT()
# print(robot)

COM = "COM3"

print("Ready")


def ik(pose):
    sol = robot.ikine_LMS(
        pose)  # mask=[1, 1, 1, 0, 1, 1])  # For underactuated robot with 5Dof # Does not work with 5Dof with LMS

    if not sol.success:
        print("no IK solution found")
        return [], True

    print("IK solution found")
    pos = sol.q  # np.delete(sol.q, 0)
    pos = np.degrees(pos)
    # print(pos.tolist())
    pos[0] = 0  # -= 180
    pos[1] += 90
    pos[2] = (pos[2] - 90) * -1
    pos[3] += 90
    pos[4] += 180
    pos = np.round(pos, 2)
    pos = pos.tolist()
    print(pos)

    return pos, False


def baseik(x, z):
    v1 = pygame.math.Vector2(abs(x), 0)
    v2 = pygame.math.Vector2(x, z)
    # vxy = v1 - v2
    return v2.angle_to(v1), v2.length() - x  # - 0.1


pygame.init()
# print(pygame.joystick.get_count())
joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Controller connected")
pos = 0
arduino_found = False
newData = False
lastSend = []
lastSpeed = 500
send_checksum = None  # Global checksum
received_checksum = None
check_val = True

stepsperdeg = [4800 / 360, 14.925 * 4, 14.925 * 4, 12000 / 360, 12000 / 360]
joint_limits = [360, 180, 180, 180, 360]
soll_pose = [0.1, 0.2, 0, 0, 180]  # (x,y,z, knick, dreh)
curr_pose = [1, 1, 1, 1, 1]  # pose to fall back if ik is wrong
msg = [1, 1, 1, 1, 1, 1, 1]

base_rotation = 0
rotation_length = 0
waitTimes = [time.time(), time.time(), time.time(), time.time()]
releaseTime = time.time()

cmd = int(0)
gripper = 0
home = 0
rotate_wrist = False
match_speed = False
start_pos = 0

if input("Type S to start programm:") == "S":

    myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
    while not arduino_found:
        myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
        print(myports)
        # arduino_port = [port for port in myports if 'COM4' in port][0]
        # print(arduino_port)
        for port in myports:
            if COM in port:
                arduino_found = True
                break
        print("Arduino not found")
        time.sleep(0.5)

    print("Arduino found")
    arduino = serial.Serial(port=COM, baudrate=250000, timeout=0.1, write_timeout=0)
    time.sleep(2)
    arduino.write('S'.encode('utf-8'))

    while True:
        change = False
        received_checksum = com.recieve(arduino)

        if received_checksum:  # There is a received checksum (Arduino Responded)
            check_val = True  # Next command can be send

        if joystick.get_button(0) and time.time() >= waitTimes[0] + 1:  # A Button to activate/deactivate gripper
            if gripper == 1:
                gripper = 0
            else:
                gripper = 1
            change = True
            waitTimes[0] = time.time()
            releaseTime = time.time()

        if gripper == 1 and time.time() >= releaseTime + 15:
            gripper = 0
            change = True

        if joystick.get_button(3) and time.time() >= waitTimes[1] + 1:  # Y Button to activate/deactivate the automatic wrist roation
            if rotate_wrist:
                rotate_wrist = False
            else:
                rotate_wrist = True
            waitTimes[1] = time.time()

        if joystick.get_button(6) and time.time() >= waitTimes[2] + 1:  # Screenshot button to home
            time.sleep(5)
            pygame.event.get()
            if joystick.get_button(6):
                waitTimes[2] = time.time()
                start_pos = 1
                home = 1
                change = True

        if joystick.get_button(7) and time.time() >= waitTimes[3] + 1:  # Menu button to return start position
            waitTimes[3] = time.time()
            start_pos = 1
            change = True

        if check_val:  # checksum == send_checksum and check_val:
            button = pygame.event.get()
            deadzone = 0.12
            pose_impact = (0.0007, 0.0006, 0.0007, 0.4, 0.4)  # How much a input changes the pose
            pose_change = (joystick.get_axis(0), trigger(joystick), joystick.get_axis(1), joystick.get_axis(2), joystick.get_axis(3))

            for i in range(0, len(pose_change)):
                round(pose_change[i], 6)
                if not (deadzone > pose_change[i] > -deadzone):
                    soll_pose[i] += pose_impact[i] * pose_change[i]
                    change = True
                    print(f"Sollpose: {soll_pose}")

            if change:
                speed = int(2000 * abs(max(pose_change, key=abs, default=0.25)))
                if start_pos == 1:
                    soll_pose = [0.1, 0.2, 0, 0, 180]
                    speed = 700
                    match_speed = True
                    start_pos = 0

                # Key=abs only runs for the function and does not change the value
                base_rotation, rotation_length = baseik(soll_pose[0], soll_pose[2])

                print(f"Sollpose[0]: {soll_pose[0]}")
                pose = SE3(soll_pose[0] + rotation_length, 0, soll_pose[1]) * SE3.RPY([0, 90, 0], unit='deg', order='xyz') * SE3.Ry(
                    soll_pose[3], 'deg') * SE3.Rz(soll_pose[4],
                                                  'deg')  # 0,90,180 offset for correct default orientation of the robot end
                pos, err = ik(pose)

                if not err:
                    pos[0] = base_rotation
                    pos[4] = soll_pose[4]
                    if rotate_wrist:
                        print("rotating Wrist")
                        pos[4] += base_rotation # if the base is turned the wrist is also turned so it orientation stays the same
                    print(pos[0])
                    if -178 > pos[0] or pos[0] > 178:  # Base has to be tested extra
                        err = True
                        print(f"Catched error for Base Rotation {pos[0]}")
                    for i in range(1, len(pos)):
                        if 0 > pos[i] or pos[i] > joint_limits[i]:
                            err = True
                            print(f"Catched error for {i}: {pos[i]}")

                print(f"Error: {err}")
                if not err:
                    for i in range(len(pos)):
                        pos[i] *= stepsperdeg[i]
                    # match_speed = True
                    cmd = int(0)
                    cmd = cmd | (gripper << 0)
                    cmd = cmd | (home << 1)
                    cmd = cmd | (match_speed << 2)
                    print(f"CMD: {cmd}")  # Sends extra commands packaged as single bits in the int

                    msg = pos
                    msg.append(speed)
                    msg.append(cmd)
                    for i in range(0, len(msg)):  # converts msg to ints
                        round(msg[i], 0)
                        msg[i] = int(msg[i])
                    print(f"Sending: {msg}")
                    msg = com.int2bytes(msg)
                    print(f"Sending Bytes:{msg}")
                    arduino.write(msg)
                    curr_pose = soll_pose
                    check_val = False
                    match_speed = False
                    home = 0
                    start_pos = 0
                else:
                    print("no IK solution")
                    soll_pose = curr_pose
                    print(curr_pose)
                    print(soll_pose)
