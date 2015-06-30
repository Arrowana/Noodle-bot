#!/usr/bin/python

import socket
import matplotlib.pyplot as plt
import time
import os
import fcntl
import struct
import numpy as np

class MedianRingBuffer(object):
    def __init__(self, size):
        self.buffer = np.ones(size, dtype=np.float)
        self.idx = 0

    def append(self, item):
        if self.idx >= len(self.buffer):
            self.idx = 0
        self.buffer[self.idx] = item
        self.idx += 1

    def pop(self):
        return np.median(self.buffer)

class Chart(object):

    def __init__(self):
        self.range = 20000
        self.senses = 0
        self.sb, self.xbuf, self.ybuf, self.zbuf, self.abuf, self.bbuf, self.cbuf  = 0, 0, 0, 0, 0, 0, 0
        plt.ion()
        self.fig = plt.figure(1, figsize=(19, 6))
        plt.ylim([self.range * -1, self.range])
        plt.xlim([0, 2300])

    def plot(self, x, y, z, a, b, c):
        self.senses += 1
        plt.plot([self.sb, self.senses], [self.xbuf, x], color='r', label='X')
        plt.plot([self.sb, self.senses], [self.ybuf, y], color='g', label='Y')
        plt.plot([self.sb, self.senses], [self.zbuf, z], color='b', label='Z')
        plt.plot([self.sb, self.senses], [self.abuf, a], color='#FF00FF', label='A')
        plt.plot([self.sb, self.senses], [self.bbuf, b], color='#006400', label='B')
        plt.plot([self.sb, self.senses], [self.cbuf, c], color='#00008B', label='C')

        self.fig.canvas.draw()
        self.sb, self.xbuf, self.ybuf, self.zbuf, self.abuf, self.bbuf, self.cbuf = self.senses, x, y, z, a, b, c

TCP_IP = '192.168.1.147'
TCP_PORT = 5005
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"

# Interval between sensing
dt = 100

# Number of senses
TotalToSense = 11250

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

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

chart = Chart()
amountSensed = 0

# data define
gyro_yaw = MedianRingBuffer(20)
gyro_pitch = MedianRingBuffer(20)
gyro_roll = MedianRingBuffer(20)
accelerometer_x = MedianRingBuffer(20)
accelerometer_y = MedianRingBuffer(20)
accelerometer_z = MedianRingBuffer(20)

handler = {
    0x17: accelerometer,
    0x18: gyroscope,
}

while amountSensed <= TotalToSense:
    raw = ''
    try:
        raw = s.recv(BUFFER_SIZE)
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

    x = gyro_yaw.pop()
    y = gyro_pitch.pop()
    z = gyro_roll.pop()
    a = accelerometer_x.pop()
    b = accelerometer_y.pop()
    c = accelerometer_z.pop()

    if x and y and z and a and b and c:
        chart.plot(x, y, z, a, b, c)
        time.sleep(dt / 1000.0)
        amountSensed += 1
s.close()
