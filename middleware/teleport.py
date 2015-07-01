#!/usr/bin/python

import fcntl
import os
import sys
import struct
import SocketServer
import json
from signal import signal, SIGINT, SIG_DFL, SIGTERM
import math as m
import threading
import time

import rospy
from robot.msg import *

TCP_IP = '0.0.0.0'
TCP_PORT = 5005

a={"test" : 1}
b={"test2" : str(2.0012345)}
data_queue = [a, b, a]

class MyTCPServer(SocketServer.ThreadingTCPServer):
    allow_reuse_address = True

class MyTCPServerHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        while True:
            try:
                if data_queue:
                    print data_queue[0]
                    self.request.sendall(json.dumps(data_queue[0]))
                    data_queue.pop(0)
            except Exception, e:
                print "Exception wile sending message: ", e

class DataProcessor:          
    def __init__(self):
        self.calibrated=False
        self.data_dict={}
        self.setup_dict()

        self.s_acc=1
        self.s_gyro=4*131*180/m.pi # sensivity+ deg to rad
        self.N_sample=50
        self.alpha=0.98

        self.low_pass=False

    def setup_dict(self):
        self.data_dict["AcX"]=[]
        self.data_dict["AcY"]=[]
        self.data_dict["AcZ"]=[]

        self.data_dict["GyX"]=[]
        self.data_dict["GyY"]=[]
        self.data_dict["GyZ"]=[]


    def on_msgSPI(self, msg):
        self.AcX=msg.imu_acc_x
        self.AcY=msg.imu_acc_y
        self.AcZ=msg.imu_acc_z

        self.GyX=msg.imu_roll
        self.GyY=msg.imu_pitch
        self.GyZ=msg.imu_yaw

        if self.calibrated == False:
            self.data_dict["AcX"].append(self.AcX)
            self.data_dict["AcY"].append(self.AcY)
            self.data_dict["AcZ"].append(self.AcZ)

            self.data_dict["GyX"].append(self.GyX)
            self.data_dict["GyY"].append(self.GyY)
            self.data_dict["GyZ"].append(self.GyZ)

        self.compute()

    def compute(self):
        if self.calibrated==False and len(self.data_dict["AcX"])>self.N_sample+1:
            self.calibrate()

            self.gyro_x=0
            self.gyro_y=0
            self.gyro_z=0

            self.gyro_roll=0
            self.gyro_pitch=0
            self.gyro_yaw=0

            self.roll=0
            self.pitch=0
            self.yaw=0

        if self.calibrated==True:
            dt=0.05

            self.acc_x=(self.AcX-self.acc_x_cal)/self.s_acc
            self.acc_y=(self.AcY-self.acc_y_cal)/self.s_acc
            self.acc_z=(self.AcZ-self.acc_z_cal)/self.s_acc
            
            self.gyro_x=(self.GyX-self.gyro_x_cal)/self.s_gyro
            self.gyro_y=(self.GyY-self.gyro_y_cal)/self.s_gyro
            self.gyro_z=(self.GyZ-self.gyro_z_cal)/self.s_gyro

            # Roll and pitch from accelerometer
            self.acc_roll=180.*m.atan2(self.acc_y, self.acc_z)/m.pi
            self.acc_pitch=-180.*m.atan2(self.acc_x, self.acc_z)/m.pi

            # Roll, pitch and yaw from gyroscope
            self.gyro_roll+=180.*dt*self.gyro_x/m.pi
            self.gyro_pitch+=180.*dt*self.gyro_y/m.pi
            self.gyro_yaw+=180.*dt*self.gyro_z/m.pi

            self.roll=self.alpha*(self.roll+180.*self.gyro_x*dt/m.pi)+(1-self.alpha)*self.acc_roll
            self.pitch=self.alpha*(self.pitch+180.*self.gyro_y*dt/m.pi)+(1-self.alpha)*self.acc_pitch

            self.send_computed_data()

    def calibrate(self):
        print("CALIBRATING===========================")
        self.acc_x_cal=sum(self.data_dict["AcX"][0:self.N_sample])/self.N_sample
        self.acc_y_cal=sum(self.data_dict["AcY"][0:self.N_sample])/self.N_sample
        self.acc_z_cal=0

        self.gyro_x_cal=sum(self.data_dict["GyX"][0:self.N_sample])/self.N_sample
        self.gyro_y_cal=sum(self.data_dict["GyY"][0:self.N_sample])/self.N_sample
        self.gyro_z_cal=sum(self.data_dict["GyZ"][0:self.N_sample])/self.N_sample

        self.calibrated=True
        print("CALIBRATED")

    def send_computed_data(self):
        #print "gyro : ", self.gyro_x, self.gyro_y, self.gyro_z
        print "rpy : ", self.roll, self.pitch, self.gyro_yaw

        acc_data = {"acc_x" : self.acc_x, "acc_y" : self.acc_y, "acc_z" : self.acc_z}
        gyro_data = {"gyro_x" : self.gyro_x, "gyro_y" : self.gyro_y, "gyro_z" : self.gyro_z}
        #rpy_data = {"roll" : self.roll, "pitch" : self.pitch, "gyro_yaw" : self.gyro_yaw}

        self.send(acc_data)
        self.send(gyro_data)
        self.send(rpy_data)

    def send(self, data):
        data_queue.append(data)

def shutdown(signum=None, frame=None):
    print "shutdown", signum, frame
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('teleport')
    signal(SIGINT, shutdown)
    signal(SIGTERM, shutdown)

    data_proc=DataProcessor()

    server = MyTCPServer(('0.0.0.0', 5005), MyTCPServerHandler)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()

    rospy.Subscriber("/msg_SPI", msg_SPI, data_proc.on_msgSPI)

    time.sleep(2)
    data_queue.append({"late" : 3})

    print "Initialization passed"
    rospy.spin()
