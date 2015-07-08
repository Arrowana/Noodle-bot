#!/usr/bin/env python
import rospy

from DataProcessor import DataProcessor
from ImuSerialReceiver import ImuSerialReceiver

import serial
import time

SERIALPORT="/dev/ttyUSB0"

if __name__ == '__main__':
    rospy.init_node('IMU_publisher', anonymous=True)

    data_queue=[]

    dataProcessor=DataProcessor(data_queue)

    #Create data container
    try:
        serialReceiver=ImuSerialReceiver(SERIALPORT, dataProcessor)
        serialReceiver.daemon=True
        serialReceiver.start()
    except(KeyboardInterrupt, SystemExit):
        print('\n! Received keyboard interrupt, quitting threads.\n')

    rospy.spin()

   


