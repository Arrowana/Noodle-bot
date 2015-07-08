#!/usr/bin/python
import rospy
from robot.msg import *

from DataProcessor import DataProcessor
import socket
import json
import math as m
from threading import Thread
import time

TCP_IP = '0.0.0.0'
TCP_PORT = 5005
BUFFER_SIZE = 1024
publish_imu = False

class ImuReceiver:
    def __init__(self, dataProcessor):
        self.dataProcessor=dataProcessor

    def on_msgSPI(self, msg):
        self.AcX=msg.imu_acc_x
        self.AcY=msg.imu_acc_y
        self.AcZ=msg.imu_acc_z

        self.GyX=msg.imu_roll
        self.GyY=msg.imu_pitch
        self.GyZ=msg.imu_yaw

        self.dataProcessor.data_dict["AcX"].append(self.AcX)
        self.dataProcessor.data_dict["AcY"].append(self.AcY)
        self.dataProcessor.data_dict["AcZ"].append(self.AcZ)

        self.dataProcessor.data_dict["GyX"].append(self.GyX)
        self.dataProcessor.data_dict["GyY"].append(self.GyY)
        self.dataProcessor.data_dict["GyZ"].append(self.GyZ)

        self.dataProcessor.compute()

class DataSender(Thread):
    def __init__(self, data_queue):
        Thread.__init__(self)

        self.data_queue=data_queue

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

        del self.data_queue[:]

        while True:
            if data_queue:
                data_to_send = json.dumps(data_queue[0])
                print "data_sent :", data_to_send
                self.conn.send(data_to_send)
                self.data_queue.pop(0)

                print "length of queue:", len(self.data_queue)
            #time.sleep(0.05)

if __name__ == '__main__':
    rospy.init_node('compute_imu')
    data_queue = []

    data_proc=DataProcessor(data_queue)
    imu=ImuReceiver(data_proc)


    data_sender = DataSender(data_queue)
    data_sender.daemon=True
    data_sender.start()


    print "Initialization passed"
    rospy.spin()
