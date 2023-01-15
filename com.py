import time
import serial

startMarker = "<".encode()
endMarker = ">".encode()


def send(pos, speed, arduino):
    checksum = 0

    string = ','.join("J" + str(e) + ":" + str(pos[e]) for e in range(0, len(pos)))  # Converts position in formatted string which arduino can decode
    string = string + f",S:{str(speed)}"
    print(f" sending: {string}")
    for i in pos:  # Calculates checksum out of all position values added together
        checksum += i

    checksum = round(checksum, 2)
    print(f"Checksum:{checksum}")  # Arduino calculates same checksum and sends it back if they differ the wrong message has been send
    arduino.write(startMarker + string.encode('utf-8') + endMarker)
    # String format: <J0:pos[0],J1:pos[1],J2:pos[2],J3:pos[3],J4:pos[4],S:speed>
    # Start and Endmarkers are used to tell arduino when a message starts and ends

    return checksum


def recieve(arduino):
    # encoded = []
    bytes_to_read = arduino.inWaiting()
    if bytes_to_read > 0: # If serial message is avilable
        received = arduino.readline().decode()
        print(f"Arduino said: {received}")

        if received.find("<") != -1 and received.find(">") != -1:  # Arduino sends back checksum in format: <C:checksum>
            received = received.strip()
            received = received.strip("<>")  # Removes start and endmarker
            print("Found start")
            encoded = received.split(':')
            print(f"encoded: {encoded}")
            for i in range(0, len(encoded)):
                if encoded[i] == 'C':
                    encoded[i + 1] = round(float(encoded[i + 1]), 4)
                    print(encoded[i + 1])
                    return encoded[i + 1]  # Return checksum as an float

    return None
