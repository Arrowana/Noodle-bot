#!/usr/bin/python
import rospy
from robot.msg import *

import fcntl
import os
import sys
import socket
import json
from signal import signal, SIGINT, SIG_DFL, SIGTERM
import math as m
from threading import Thread
import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

TCP_IP = '0.0.0.0'
TCP_PORT = 5005
BUFFER_SIZE = 1024
publish_imu = False

data_queue = []

class DataProcessor:          
    def __init__(self):
        rospy.init_node('compute_imu')
        self.calibrated=False
        self.data_dict={}

        self.last_update = rospy.Time.now()

        self.s_acc=1
        self.s_gyro=1/131. # sensivity+ deg to rad
        self.N_sample=100 #200
        self.alpha=0.98

        self.lp_alpha=0.99 
        self.low_pass=False


        rospy.Subscriber("/msg_SPI", msg_SPI, self.on_msgSPI)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

        self.setup_dict()

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
        if self.calibrated==False and len(self.data_dict["AcX"])<self.N_sample+1:
            print "Remaining values needed for calibration:", self.N_sample-len(self.data_dict["AcX"])

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

            acc_x_nf=(self.AcX-self.acc_x_cal)*self.s_acc
            if self.low_pass:
                self.acc_x=self.lp_alpha*self.acc_x+(1-self.lp_alpha)*acc_x_nf
            else:
                self.acc_x=acc_x_nf

            acc_y_nf=(self.AcY-self.acc_y_cal)*self.s_acc
            if self.low_pass:
                self.acc_y=self.lp_alpha*self.acc_y+(1-self.lp_alpha)*acc_y_nf
            else:
                self.acc_y=acc_y_nf

            acc_z_nf=(self.AcZ-self.acc_z_cal)*self.s_acc
            if self.low_pass:
                self.acc_z=self.lp_alpha*self.acc_z+(1-self.lp_alpha)*acc_z_nf
            else:
                self.acc_z=acc_z_nf
            
            self.gyro_x=(self.GyX-self.gyro_x_cal)*self.s_gyro
            self.gyro_y=(self.GyY-self.gyro_y_cal)*self.s_gyro
            self.gyro_z=(self.GyZ-self.gyro_z_cal)*self.s_gyro

            # Roll and pitch from accelerometer
            self.acc_roll=180.*m.atan2(self.acc_y, self.acc_z)/m.pi
            self.acc_pitch=-180.*m.atan2(self.acc_x, self.acc_z)/m.pi

            # Roll, pitch and yaw from gyroscope
            self.gyro_roll+=dt*self.gyro_x
            self.gyro_pitch+=dt*self.gyro_y
            self.gyro_yaw+=dt*self.gyro_z
            # Keep gyro_yaw within -180;180
            if self.gyro_yaw > 185:
                self.gyro_yaw = self.gyro_yaw - 360
            elif self.gyro_yaw < -180:
                self.gyro_yaw = self.gyro_yaw + 360

            self.roll=self.alpha*(self.roll+self.gyro_x*dt)+(1-self.alpha)*self.acc_roll
            self.pitch=self.alpha*(self.pitch+self.gyro_y*dt)+(1-self.alpha)*self.acc_pitch

            self.publish_imu()
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
        acc_data = {"acc_x":self.acc_x, "acc_y":self.acc_y,"acc_z":self.acc_z}
        acc_rpy_data = {"acc_roll":self.acc_roll,"acc_pitch":self.acc_pitch}
        gyro_data = {"gyro_x":self.gyro_x,"gyro_y":self.gyro_y,"gyro_z":self.gyro_z}
        gyro_rpy_data = {"gyro_roll":self.gyro_roll,"gyro_pitch":self.gyro_pitch}
        rpy_data = {"roll":self.roll,"pitch":self.pitch,"gyro_yaw":self.gyro_yaw}

        # Lower update of values
        if (rospy.Time.now()-self.last_update).to_sec()>0.2:
            print "roll:",self.roll,"pitch:",self.pitch,"gyro_yaw", self.gyro_yaw

            self.send(acc_data)
            self.send(acc_rpy_data)
            self.send(gyro_data)
            self.send(gyro_rpy_data)
            self.send(rpy_data)

            self.last_update = rospy.Time.now()

    def send(self, data):
        data_queue.append(data)

    def publish_imu(self):
        imu_msg=Imu()
        imu_msg.header.stamp=rospy.Time.now()
        imu_msg.header.frame_id="map"
        quat=quaternion_from_euler(-m.pi*self.roll/180, m.pi*self.pitch/180, m.pi*self.gyro_yaw/180)
        imu_msg.orientation.x=quat[0]
        imu_msg.orientation.y=quat[1]
        imu_msg.orientation.z=quat[2]
        imu_msg.orientation.w=quat[3]

        self.imu_pub.publish(imu_msg)

class DataSender(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((TCP_IP, TCP_PORT))
        server_socket.listen(1)

        self.conn, addr = server_socket.accept()
        print 'Connection address:', addr
        #Handshake
        data = self.conn.recv(BUFFER_SIZE)
        print data

        del data_queue[:]

        while True:
            if data_queue:
                data_to_send = json.dumps(data_queue[0])
                print "data_sent :", data_to_send
                self.conn.send(data_to_send)
                data_queue.pop(0)

                print "length of queue:", len(data_queue)
            #Cap speed
            #time.sleep(0.05)

def shutdown(signum=None, frame=None):
    print "shutdown", signum, frame
    sys.exit(0)

if __name__ == '__main__':
    signal(SIGINT, shutdown)
    signal(SIGTERM, shutdown)

    data_proc=DataProcessor()

    data_sender = DataSender()
    data_sender.daemon=True
    data_sender.start()


    print "Initialization passed"
    rospy.spin()
