# RobotarmPC_public

Script takes controller inputs via pygame to move a point in 3D Space.
Then the inverse Kinematics are used to calculate the joint angles of the robot to reach this point. 
The joint angeles and speed are then send via serial to an Arduino Mega which drives the stepper motors accordingly. 
The PC sends a string over serial with start and endmarkers and special format e.g. J1: for Joint1 etc. .

This project uses the Robotics Toolbox for Python for its inverse kinematics.
Thanks to its Contributors
https://github.com/petercorke/robotics-toolbox-python

This project is only for demonstration purposes and not in its final form
So this code is not optimized for readability or usage by others (but you certainly can)
