#!/usr/bin/python
import rospy
from robot.msg import *

from DataProcessor import DataProcessor
import os
import sys
import socket
import json
from signal import signal, SIGINT, SIG_DFL, SIGTERM
import math as m
from threading import Thread
import time

TCP_IP = '0.0.0.0'
TCP_PORT = 5005
BUFFER_SIZE = 1024
publish_imu = False

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
            #time.sleep(0.05)

def shutdown(signum=None, frame=None):
    print "shutdown", signum, frame
    sys.exit(0)

if __name__ == '__main__':
    signal(SIGINT, shutdown)
    signal(SIGTERM, shutdown)

    data_queue = []

    data_proc=DataProcessor(data_queue)

    data_sender = DataSender()
    data_sender.daemon=True
    data_sender.start()


    print "Initialization passed"
    rospy.spin()
