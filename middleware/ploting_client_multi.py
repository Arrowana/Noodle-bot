#!/usr/bin/env python
import rospy

import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
import time
import os
import json
import math as m

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

TCP_IP = '192.168.1.147'
TCP_PORT = 5005
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"

class DataReceiver(Thread):
    def __init__(self, axes):
        Thread.__init__(self)
        rospy.init_node('DataRecever')

        # Interval between sensing
        self.dt = 100
        self.data_dict={}
        self.axes=axes

        self.cleaning=True

        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))

        print "Connected"

    def update(self, frameNum):
        # For each axis iterate through each line to set the data
        for axe in self.axes:
            y_max_list=[]
            isData=False

            for line in axe.get_lines():
                data_name=line.get_label()

                if data_name in self.data_dict:
                    if self.data_dict[data_name]:
                        y_max=self.modify(line, data_name)
                        y_max_list.append(y_max)

                        isData=True
            if isData:
                # Set ylim to the max of y data in the graph
                global_max=max(y_max_list)
                axe.set_ylim(-global_max, global_max)

        if self.cleaning:
            self.clean_data_dict() 

    def modify(self, line, data_name):
        #Plot only N_values
        length=100
        length_data=len(self.data_dict[data_name])

        if(length_data<length):
            length=length_data

        line.set_data(range(length), self.data_dict[data_name][-length:])
        line.axes.set_xlim(0, length)
        y_max=max([abs(x) for x in self.data_dict[data_name]])
        return y_max

    def clean_data_dict(self):
        for key in self.data_dict.keys():
            length=len(self.data_dict[key])
            if(length>500):
                del self.data_dict[key][:150]

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

            #print "parse data :", json_string, "type :", type(json_string)
            received_dict = json.loads(json_string)
            if received_dict:
                #print "received_dict", received_dict, type(received_dict)
                for data_name in received_dict.keys():
                    self.data_dict[data_name].append(received_dict[data_name])

                    if data_name in ["roll", "pitch", "gyro_yaw"]:
                        print data_name, ":", received_dict[data_name]

                    if all(len(self.data_dict[name])>0 for name in ["roll", "pitch", "gyro_yaw"]):
                        self.publish_imu()


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
                #print "received data:", data
            except Exception as e:
                print "resource not ready"
                continue

            if data:
                self.parse(data)

            time.sleep(0.01)

        self.s.close()

    def publish_imu(self):
        imu_msg=Imu()
        imu_msg.header.stamp=rospy.Time.now()
        imu_msg.header.frame_id="map"
        quat=quaternion_from_euler(-m.pi*self.data_dict["roll"][-1]/180, m.pi*self.data_dict["pitch"][-1]/180, m.pi*self.data_dict["gyro_yaw"][-1]/180)
        imu_msg.orientation.x=quat[0]
        imu_msg.orientation.y=quat[1]
        imu_msg.orientation.z=quat[2]
        imu_msg.orientation.w=quat[3]

        self.imu_pub.publish(imu_msg)

if __name__ == '__main__':
    fig = plt.figure(0)

    ax_acc=plt.subplot(331)
    plt.title("acc calibrated")
    ax_acc.plot([],[],label="acc_x")
    ax_acc.plot([],[],label="acc_y")
    ax_acc.plot([],[],label="acc_z")

    ax_gyro=plt.subplot(332)
    plt.title("gyro calibrated")
    ax_gyro.plot([],[], label="gyro_x")
    ax_gyro.plot([],[], label="gyro_y")
    ax_gyro.plot([],[], label="gyro_z")

    ax_roll_comb=plt.subplot(333)
    plt.title("acc_roll, gyro_roll and roll")
    ax_roll_comb.plot([],[], label="acc_roll")
    ax_roll_comb.plot([],[], label="gyro_roll")

    ax_pitch_comb=plt.subplot(334)
    plt.title("acc_pitch, gyro_pitch and pitch")
    ax_pitch_comb.plot([],[], label="acc_pitch")
    ax_pitch_comb.plot([],[], label="gyro_pitch")

    ax_roll=plt.subplot(335)
    plt.title("roll")
    ax_roll.plot([],[], label="roll")

    ax_pitch=plt.subplot(336)
    plt.title("pitch")
    ax_pitch.plot([],[], label="pitch")

    ax_yaw=plt.subplot(338)
    plt.title("gyro_yaw")
    ax_yaw.plot([],[], label="gyro_yaw")

    axes=[ax_acc, ax_gyro, ax_roll_comb, ax_pitch_comb, ax_roll, ax_pitch, ax_yaw]

    data_rc=DataReceiver(axes)
    data_rc.daemon=True
    data_rc.start()

    anim = animation.FuncAnimation(fig, data_rc.update, interval=50)
    plt.show()
