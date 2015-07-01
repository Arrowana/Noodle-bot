import SocketServer
import json

a={"test" : 1}
b={"test2" : str(2.0012345)}
data_queue = [a, b, a]

class MyTCPServer(SocketServer.ThreadingTCPServer):
    allow_reuse_address = True

class MyTCPServerHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        for i in range(10):
            try:
                if data_queue:
                    #print type(json.dumps(data_queue[0]))
                    #print json.dumps(data_queue[0])
                    self.request.sendall(json.dumps(data_queue[0]))
                    data_queue.pop(0)
            except Exception, e:
                print "Exception wile sending message: ", e

server = MyTCPServer(('0.0.0.0', 5005), MyTCPServerHandler)
server.serve_forever()