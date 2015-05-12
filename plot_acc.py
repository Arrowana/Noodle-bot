import serial

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from threading import Thread, Event
import time             

class DataContainer(Thread):
    def __init__(self):
        Thread.__init__(self)
        self._stop = Event()
        
        maxData=0
        self.ser = serial.Serial(2, 9600)
        self.data_dict={}
        self.a0_list=[]
        self.a1_list=[]

    def stop(self):
        self._stop.set()

    def update(self, frameNum, a0, a1):
        #self.pop_data(self.a0_list)
        #self.pop_data(self.a1_list)
        
        a0.set_data(range(len(self.a0_list)), self.a0_list)
        #a1.set_data(range(len(self.a1_list)), self.a1_list)

        self.stop()

    def run(self):
        while(True):
            print("Data received")
            data_raw=self.ser.readline()
            data=data_raw.decode("utf-8")
            self.process(str(data))
            
            #time.sleep(1)

    def process(self, data):
        if "data:" in data:
            header, data_name,=data.split(":")

            # Read value associated to data_name
            value_raw=self.ser.readline()
            value=int(value_raw.decode("utf-8"))

            # Store in dictionnary and add key if doesn't exist
            if "data_name" in self.data_dict:
                self.data_dict["data_name"].append(value)
            else:
                self.data_dict["data_name"]=[value]

    def pop_data(self, data_list):
        data_list.append(random.randint(0,100))

def main():
    x_max=1000
    y_max=20000
    data=DataContainer()
    data.start()

    # set up animation
    fig = plt.figure()
    ax = plt.axes(xlim=(0, x_max), ylim=(0, y_max))
    a0, = ax.plot([], [])
    a1, = ax.plot([], [])
    anim = animation.FuncAnimation(fig, data.update, 
                                 fargs=(a0, a1), 
                                 interval=50)
    plt.show()

if __name__ == '__main__':
    main()


