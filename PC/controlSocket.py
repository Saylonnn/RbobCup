import socket
import time

class ControlSocket:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.ev3_ip = ev3_ip
        self.STOP = False
         

    #connect to EV3 or wait and retries
    def connectSocket(self):
        self.sock.connect(('192.168.226.1', 8484))
        self.connection = self.sock.makefile('wb')
        
        


    def fw(self):
        self.sock.sendall(b"FW")

    def bw(self):
        self.sock.sendall(b"BW")

    def rotateAngle(self, angle):
        x = "angle "+ str(angle)
        self.sock.sendall(bytes(x, 'utf-8'))

    def hold(self):
        self.sock.sendall(b"hold")

    def exit_All(self):
        self.sock.sendall(b"exit")
        self.sock.close()
        self.STOP = True
        exit()
    
    def set_speed(self, x):
        self.sock.sendall(bytes("set_speed ", x))
    
    def shoot(self):
        self.sock.sendall(b"shoot")
    
    def grab(self):
        self.sock.sendall(b"catch")

    def run(self):
        try:
            self.connectSocket()
        except ConnectionRefusedError:
            raise IOError("Could not connect to IP")
        