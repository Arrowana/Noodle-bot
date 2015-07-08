#!/usr/bin/python
import socket
import json
from threading import Thread

TCP_IP = '0.0.0.0'
TCP_PORT = 5005
BUFFER_SIZE = 1024

class DataTCPSender(Thread):
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