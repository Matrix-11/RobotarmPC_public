import time
import serial

startMarker = "<".encode()
endMarker = ">".encode()


def int2bytes(x, i=0):
    if i == len(x) - 1:
        highByte = (x[i] >> 8) & 0xff
        lowByte = x[i] & 0xff
        return bytearray([highByte, lowByte])

    highByte = (x[i] >> 8) & 0xff
    lowByte = x[i] & 0xff
    i += 1
    res = bytearray([highByte, lowByte])
    res.extend(int2bytes(x, i))
    #print(res)
    return res


def recieve(arduino):
    bytes_to_read = arduino.inWaiting()
    if bytes_to_read > 0:
        received = arduino.readline().decode()
        print(f"Arduino said: {received}")

        if received.find("<") != -1 and received.find(">") != -1:
            received = received.strip()
            received = received.strip("<>")
            print(f"encoded: {received}")
            if received == "1":
                return True
    return False
