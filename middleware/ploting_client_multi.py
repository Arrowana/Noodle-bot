#!/usr/bin/python

import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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
    def __init__(self, lines):
        Thread.__init__(self)

        # Interval between sensing
        self.dt = 100

        #data_dict contains arrays of data
        self.data_dict={}
        self.lines=lines

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))

    def update(self, frameNum):
        for line in self.lines:
            data_name=line.get_label()
            if data_name in self.data_dict:
                self.modify(line, data_name)

    def modify(self, line, data_name):
        #Plot only N_values
        length=30

        if(len(self.data_dict[data_name])>length):
            line.set_data(range(length), self.data_dict[data_name][-length:])
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
            self.data_dict["gyro_x"].append(r)
            self.data_dict["gyro_y"].append(p)
            self.data_dict["gyro_z"].append(y)

        def accelerometer(x, y, z):
            print "acc: ", x, y, z
            self.data_dict["acc_x"].append(x)
            self.data_dict["acc_y"].append(y)
            self.data_dict["acc_z"].append(z)

        fmt = struct.Struct('2B 3h')

        # data define
        self.data_dict["gyro_x"] = []
        self.data_dict["gyro_y"] = []
        self.data_dict["gyro_z"] = []
        self.data_dict["acc_x"] = []
        self.data_dict["acc_y"] = []
        self.data_dict["acc_z"] = []

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

            time.sleep(self.dt / 1000.0)

        self.s.close()

if __name__ == '__main__':
    fig = plt.figure(0)

    ax1=plt.subplot(121)
    plt.title("acc calibrated")
    acc_x, = ax1.plot([],[],label="acc_x")
    acc_y, = ax1.plot([],[],label="acc_y")
    acc_z, = ax1.plot([],[],label="acc_z")

    ax2=plt.subplot(122)
    plt.title("gyro calibrated")
    gyro_x, = ax2.plot([],[], label="gyro_x")
    gyro_y, = ax2.plot([],[], label="gyro_y")
    gyro_z, = ax2.plot([],[], label="gyro_z")

    lines=[acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]

    data_rc=DataReceiver(lines)
    data_rc.daemon=True
    data_rc.start()

    anim = animation.FuncAnimation(fig, data_rc.update, interval=50)
    plt.show()
