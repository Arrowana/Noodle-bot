import socket
import json

data = {'message':'hello world!', 'test':123.4}

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('192.168.1.145', 5005))
s.send(json.dumps(data))
result = json.loads(s.recv(1024))
print result
s.close()