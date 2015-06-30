#!/usr/bin/python

import fcntl
import os
import sys
import struct
import socket
from signal import signal, SIGINT, SIG_DFL, SIGTERM

import rospy
from robot.msg import *



TCP_IP = '0.0.0.0'
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setblocking(0)
s.bind((TCP_IP, TCP_PORT))
# fcntl.fcntl(s, fcntl.F_SETFL, os.O_NONBLOCK)
s.listen(1)

while True:
    try:
        conn, addr = s.accept()
        break
    except Exception as e:
        print e
        continue
print 'Connection address:', addr

fmt = struct.Struct('2B 3h')
message = [0] * 5

def on_msgSPI(msg):
    message[0] = 0x17
    message[1] = 6
    message[2] = msg.imu_acc_x
    message[3] = msg.imu_acc_y
    message[4] = msg.imu_acc_z
    conn.send(fmt.pack(*message))  # echo

    message[0] = 0x18
    message[1] = 6
    message[2] = msg.imu_yaw
    message[3] = msg.imu_pitch
    message[4] = msg.imu_roll
    conn.send(fmt.pack(*message))  # echo

def shutdown(signum=None, frame=None):
    print "shutdown", signum, frame
    s.close()
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('teleport')
    signal(SIGINT, shutdown)
    signal(SIGTERM, shutdown)
    rospy.Subscriber("/msg_SPI", msg_SPI, on_msgSPI)
    rospy.spin()
