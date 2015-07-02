#!/usr/bin/python

import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
import time
import os
import json
import fcntl
import struct
import numpy as np

TCP_IP = '192.168.1.140'
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
        length=100

        if(len(self.data_dict[data_name])>length):
            line.set_data(range(length), self.data_dict[data_name][-length:])
            line.axes.set_xlim(0, length)

            y_max=max([abs(x) for x in self.data_dict[data_name]])
            line.axes.set_ylim(-y_max, y_max)

    def run(self):
        def parse(data):
            print "Beginning parsing"
            start=data.index("{")
            stop=data.index("}")+1
            json_string = data[start:stop]

            print "parse data :", json_string, "type :", type(json_string)

            received_dict = json.loads(json_string)
            print "received_dict parsed"

            if received_dict:
                print "received_dict", received_dict, type(received_dict)

                for data_name in received_dict.keys():
                    print("add data to data_dict")
                    self.data_dict[data_name].append(received_dict[data_name])

        # data define
        self.data_dict["gyro_x"] = []
        self.data_dict["gyro_y"] = []
        self.data_dict["gyro_z"] = []
        self.data_dict["acc_x"] = []
        self.data_dict["acc_y"] = []
        self.data_dict["acc_z"] = []

        self.data_dict["acc_roll"] = []
        self.data_dict["acc_pitch"] = []
        self.data_dict["acc_yaw"] = []

        self.data_dict["gyro_roll"] = []
        self.data_dict["gyro_pitch"] = []
        self.data_dict["gyro_yaw"] = []

        self.data_dict["roll"]=[]
        self.data_dict["pitch"]=[]
        self.s.send("Hi")

        # Start receiving loop
        while True:
            try:
                data=self.s.recv(BUFFER_SIZE)
                print "received data:", data
            except Exception as e:
                print "resource not ready"
                continue

            if data:
                parse(data)

            time.sleep(0.01)

        self.s.close()

if __name__ == '__main__':
    fig = plt.figure(0)

    ax1=plt.subplot(221)
    plt.title("acc calibrated")
    acc_x, = ax1.plot([],[],label="acc_x")
    acc_y, = ax1.plot([],[],label="acc_y")
    acc_z, = ax1.plot([],[],label="acc_z")

    ax2=plt.subplot(222)
    plt.title("gyro calibrated")
    gyro_x, = ax2.plot([],[], label="gyro_x")
    gyro_y, = ax2.plot([],[], label="gyro_y")
    gyro_z, = ax2.plot([],[], label="gyro_z")

    ax3=plt.subplot(223)
    plt.title("roll")
    roll, = ax3.plot([],[], label="roll")

    ax5=plt.subplot(224)
    plt.title("pitch")
    pitch, = ax5.plot([],[], label="pitch")

    lines=[acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll, pitch]

    data_rc=DataReceiver(lines)
    data_rc.daemon=True
    data_rc.start()

    anim = animation.FuncAnimation(fig, data_rc.update, interval=50)
    plt.show()
