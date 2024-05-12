#! /usr/bin/env python3

import sys

# import sensor_pb2 as spb
import serial
import serial.tools.list_ports
import test_pb2 as tpb

from cobs import cobs

def printSensor(data):
    recv = tpb.TestMessageWithoutOptions()
    recv.ParseFromString(data)
    print("yey")
    
    # sensor = spb.SensorReading()
    # sensor.ParseFromString(data)
    # print('Read from sensor reading number {}'.format(sensor.id))
    # print('    co2 level : {}'.format(sensor.co2))
    # print('    temperature : {:.{prec}f}'.format(sensor.temperature, prec=3))
    # print('    humidity : {:.{prec}f}'.format(sensor.humidity, prec=3))
    # print('')

def find_port():
    ports = serial.tools.list_ports
    devs = ports.comports()
    target = ""
    for dev in devs:
        try:
            name = str(dev.product)
            if "ROBOT" in name:
                target = dev.device
                break
        except Exception as e:
            print("Error: " + e)
    return target

def main():
    # For example : ./sensor_reading.py 19200
    port = find_port()
    baud = 115200
    tOut = None

    if port == '':
        exit()

    # if sys.argv == 4:
    #     tOut = int(sys.argv[3])

    with serial.Serial(port, baud, timeout=tOut) as ser:
        while True:
            data = []
            c = ser.read_until(b'/')
            # print(hex(ord(c)))
            b = c[:-1]
            printSensor(b)
            ser.close()
            if c == b'':
                break
            while c != b'\x00' and c != b'':
                data.append(c)
                c = ser.read()
            data = b''.join(data)
            printSensor(cobs.decode(data))

if __name__ == "__main__":
    main()