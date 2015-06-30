#!/usr/bin/python

import socket
import matplotlib.pyplot as plt
from threading import Thread
import time
import os
import fcntl
import struct
import numpy as np

TCP_IP = '192.168.1.147'
TCP_PORT = 5005
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"

class DataReceiver(Thread):
    def __init__(self, line):
        Thread.__init__(self)

        # Interval between sensing
        dt = 100

        self.line=line

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))

    def update(self, frameNum):
        data_name="acc_x"
        self.modify(self.line, data_name)

    def modify(self, line, data_name):
        #Plot only N_values
        length=200

        if(len(self.data_dict[data_name])>length):
            line.set_data(range(length),self.data_dict[data_name][-length:])
            line.axes.set_xlim(0, length)

            y_max=max([abs(x) for x in self.data_dict[data_name]])
            line.axes.set_ylim(-y_max, y_max)

    def run(self):
        def parse(bs, idx=0):
            """format: |cmd|length|data|data|
                       |fixed leng|
            """
            head_byte = 2
            length = ord(bs[idx+1])
            sub_bs = bs[idx: idx + head_byte + length]
            return len(sub_bs), sub_bs

        def gyroscope(y, p, r):
            print "gyrp: ", y, p, r
            gyro_yaw.append(y)
            gyro_pitch.append(p)
            gyro_roll.append(r)

        def accelerometer(x, y, z):
            print "acc: ", x, y, z
            accelerometer_x.append(x)
            accelerometer_y.append(y)
            accelerometer_z.append(z)

        fmt = struct.Struct('2B 3h')

        # data define
        self.gyro_yaw = []
        self.gyro_pitch = []
        self.gyro_roll = []
        self.accelerometer_x = []
        self.accelerometer_y = []
        self.accelerometer_z = []

        handler = {
            0x17: accelerometer,
            0x18: gyroscope,
        }

        # Start receiving loop
        while True:
            raw = ''
            try:
                raw = self.s.recv(BUFFER_SIZE)
                idx = 0
                while idx < len(raw):
                    length, data = parse(raw, idx)
                    idx += length
                    cmd, length, x, y, z = fmt.unpack(data)
                    handler[cmd](x, y, z)
                print "============="
            except Exception as e:
                print "resource not ready"
                continue

            x = gyro_yaw
            y = gyro_pitch
            z = gyro_roll
            a = accelerometer_x
            b = accelerometer_y
            c = accelerometer_z

            if x and y and z and a and b and c:
                #chart.plot(x, y, z, a, b, c)
                time.sleep(dt / 1000.0)
        self.s.close()

if __name__ == '__main__':
    fig = plt.figure(0)
    ax1=plt.subplot(331)
    plt.title("acc calibrated")
    acc_x, = ax1.plot([],[],label="acc_x")

    data_rc=DataReceiver(acc_x)
    data_rc.start()

    anim = animation.FuncAnimation(fig, data_rc.update, interval=50)
    plt.show()
