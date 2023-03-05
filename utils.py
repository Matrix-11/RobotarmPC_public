def trigger(joystick):  # Reads both analog triggers and returns the greater one
    trigL = (joystick.get_axis(4) + 1) / 2
    trigR = (joystick.get_axis(5) + 1) / 2
    if trigL >= trigR:  # Trigger Value from -1 to 1
        return -trigL
    else:
        return trigR  # Right Trigger has a neagtive value
