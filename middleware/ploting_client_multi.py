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

        print "Connected"

    def update(self, frameNum):
        for line in self.lines:
            data_name=line.get_label()
            if data_name in self.data_dict:
                if self.data_dict[data_name]:
                    self.modify(line, data_name)

    def modify(self, line, data_name):
        #Plot only N_values
        length=100
        length_data=len(self.data_dict[data_name])

        if(length_data<length):
            length=length_data

        line.set_data(range(length), self.data_dict[data_name][-length:])
        line.axes.set_xlim(0, length)

        y_max=max([abs(x) for x in self.data_dict[data_name]])
        #line.axes.set_ylim(-y_max, y_max)

    def parse(self, data):
        while len(data)>0:
            try:
                start=data.index("{")
                stop=data.index("}")+1
            except:
                print "{ not found in data"
                return

            if start != 0:
                print "Unexpected location of {"
                return
            # Parsed json string 
            json_string = data[start:stop]
            # Remove found json string from data
            data=data[stop:]

            print "parse data :", json_string, "type :", type(json_string)
            received_dict = json.loads(json_string)
            if received_dict:
                #print "received_dict", received_dict, type(received_dict)
                for data_name in received_dict.keys():
                    self.data_dict[data_name].append(received_dict[data_name])

    def run(self):
        print "Thread started, run function"
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
                self.parse(data)

            time.sleep(0.01)

        self.s.close()

if __name__ == '__main__':
    fig = plt.figure(0)

    ax1=plt.subplot(331)
    plt.title("acc calibrated")
    acc_x, = ax1.plot([],[],label="acc_x")
    acc_y, = ax1.plot([],[],label="acc_y")
    acc_z, = ax1.plot([],[],label="acc_z")

    ax2=plt.subplot(332)
    plt.title("gyro calibrated")
    gyro_x, = ax2.plot([],[], label="gyro_x")
    gyro_y, = ax2.plot([],[], label="gyro_y")
    gyro_z, = ax2.plot([],[], label="gyro_z")

    ax3=plt.subplot(333)
    plt.title("acc_roll, gyro_roll and roll")
    acc_roll, = ax3.plot([],[], label="acc_roll")
    gyro_roll, = ax3.plot([],[], label="gyro_roll")

    ax5=plt.subplot(334)
    plt.title("acc_pitch, gyro_pitch and pitch")
    acc_pitch, = ax5.plot([],[], label="acc_pitch")
    gyro_pitch, = ax5.plot([],[], label="gyro_pitch")
    ax5.autoscale_view(True, 'y')

    ax6=plt.subplot(335)
    plt.title("roll")
    roll, = ax6.plot([],[], label="roll")

    ax7=plt.subplot(336)
    plt.title("pitch")
    pitch, = ax7.plot([],[], label="pitch")

    ax9=plt.subplot(338)
    plt.title("gyro_yaw")
    gyro_yaw, = ax9.plot([],[], label="gyro_yaw")

    lines=[acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, acc_pitch, acc_roll, gyro_roll,
     gyro_pitch, gyro_yaw, roll, pitch]

    data_rc=DataReceiver(lines)
    data_rc.daemon=True
    data_rc.start()

    anim = animation.FuncAnimation(fig, data_rc.update, interval=50)
    plt.show()
