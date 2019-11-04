"""
coms.py

Followbot POE Projects

@author: Duncan Mazza
"""

import serial

def main(ser):
    while ser.isOpen():
        line = ser.readline().decode()
        

    pass

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0')  # open serial port
    main(ser)
    