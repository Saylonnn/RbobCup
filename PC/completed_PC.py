import socket
import time
from controlSocket import ControlSocket
import threading
import numpy as np
import math


class PcController:
    def __init__(self):
        # global Variables
        self.IS_CONNECTED = False
        self.EV3_DEV = 1
        self.EvFrontCor = None
        self.EvBackCor = None
        self.BallCor = None
        self.GAME_RUNNING = True
        self.GOAL_1_COR = None
        self.GOAL_2_COR = None
        self.RUNNING = True
        self.GAME_STARTED = None
        self.sock = None

    def drive_control(self):
        self.drive_to_start()

        '''
        degree = self.calculateAngle([3, 1], [3,0], [3,7])
        print(degree)

        for x in range(10):
            print(self.BallCor)
            time.sleep(2)



        x=1
        print("drive_control started")
        #self.driveToStart()
        while True:
            print("distance: ", self.messureDistance(self.EvFrontCor, self.BallCor))
            angle = self.calculateAngle(self.EvFrontCor, self.EvBackCor, self.BallCor)
            print("angle: ", angle)
            time.sleep(5)
            self.sock.rotateAngle(angle)
            if x == 100:
                break

        
        while self.GAME_RUNNING == True:
            print("lets start")

            degree = self.calculateAngle(self.EvFrontCor, self.EvBackCor, self.BallCor)
            self.sock.rotateAngle(degree)
            
            while self.messureDistance(self.EvFrontCor, self.BallCor) > 5 and abs(self.calculateAngle(self.EvFrontCor, self.EvBackCor, self.BallCor)) < 30:
                self.sock.fw()
            self.sock.hold()
            if self.messureDistance(self.BallCor, self.EvFrontCor) < 5:
                self.sock.grab()
                degree = self.calculateAngle(self.EvFrontCor, self.EvBackCor, self.GOAL_2_COR)
                self.sock.rotateAngle(degree)
                while self.messureDistance(self.EvFrontCor, self.GOAL_2_COR) > 200:
                    self.sock.fw()
                self.sock.shoot()
                self.sock.hold()
        self.driveToStart()
        #self.driveControl()
        '''

    def drive_to_start(self):
        print("drive to start")
        if self.EV3_DEV == 1:
            start_pos = (self.GOAL_1_COR[0]+100, self.GOAL_1_COR[1])
            self.sock.hold()
            self.calculate_angle(self.EvFrontCor, self.EvBackCor, start_pos)
            self.sock.fw()
            while self.messure_distance(self.EvFrontCor, start_pos) > 0:
                time.sleep(10)
            self.sock.hold()
            degree = self.calculate_angle(self.EvFrontCor, self.EvBackCor, (512, 393))
            self.sock.rotateAngle(degree)

        if self.EV3_DEV == 2:
            start_pos = (self.GOAL_2_COR[0]-100, self.GOAL_2_COR[1])
            self.sock.hold()
            self.calculate_angle(self.EvFrontCor,self.EvBackCor, start_pos)
            self.sock.fw()
            while self.messure_distance(self.EvFrontCor, start_pos) > 0:
                time.sleep(10)
            self.sock.hold()
            degree = self.calculate_angle(self.EvFrontCor, self.EvBackCor, (512, 393))
            self.sock.rotateAngle(degree)

    @staticmethod
    def messure_distance(self, ev3_front, ball_cor):
        # print("messure distance")
        distance = math.sqrt((ev3_front[0] - ball_cor[0])**2 + (ev3_front[1] - ball_cor[1])**2)
        return distance

    @staticmethod
    def calculate_angle(self, ev3_vorne, ev3_hinten, ball_koor):
        # print("calculate Angle")
        ev3_vorne = np.array(ev3_vorne)
        ev3_hinten = np.array(ev3_hinten)
        ball_koor = np.array(ball_koor)
        ev3_mittig = np.array([ev3_vorne[0]-(ev3_vorne[0] - ev3_hinten[0])/2, ev3_vorne[1]-(ev3_vorne[1] -
                                                                                            ev3_hinten[1])/2])
        ev3_vec = ev3_vorne - ev3_mittig
        to_ball_vec = ball_koor - ev3_mittig

        # calculate_Angle
        # Skalarprodukt --> a_1 * b_1 +.... a_n + b_n
        skalar_produkt = ev3_vec.dot(to_ball_vec)

        # berechne vectoren länge
        # länge_ev3_vec =  sqrt(a[0]^2 + ... + a[n]^2)
        x = 0
        for i in range(0, ev3_vec.size):
            x = x + ev3_vec[i]**2
        length_ev3_vec = math.sqrt(x)
       # print("length_ev3_vec: ", length_ev3_vec)
       
        x = 0
        for i in range(0, to_ball_vec.size):
            x = x + to_ball_vec[i]**2
        length_ball_vec = math.sqrt(x)
        # print("length_ball_vec: ", length_ball_vec)

        degree = math.degrees(math.acos(skalar_produkt / (length_ball_vec * length_ev3_vec)))
        if degree > 180:
            degree = -(360 - degree)
        return int(degree)

    def start_game(self):
        self.GAME_STARTED = True
    
    def run_socket(self):
        self.sock = ControlSocket()
        self.sock.run()
        print("control Socket started")



    def connectPosManager(self):
        print("connect to position Manager")
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('192.168.226.230', 8485))
        #connection = self.client_socket.makefile('wb')
        self.IS_CONNECTED = True
        while True:
            data = self.client_socket.recv(2048)
            data = data.decode('utf-8')
            data = data.split(",")
            #print(data)
            #Ball, EV3_1.Vorne, EV3_1.Hinten, EV3_2.Vorne, EV3_2.Hinten, Tor1_rechts, Tor1_links, Tor2_rechts, Tor2_links

            self.BallCor = [int(data[0]), int(data[1])]
            if self.EV3_DEV == 1:
                self.EvFrontCor = [int(data[2]), int(data[3])]
                self.EvBackCor = [int(data[4]), int(data[5])]
            if self.EV3_DEV == 2:
                self.EvFrontCor = [int(data[6]), int(data[7])]
                self.EvBackCor = [int(data[8]), int(data[9])]
            self.GOAL_1_COR = ((int(data[10]) + int(data[12]))/2, (int(data[11]) + int(data[13]))/2)
            self.GOAL_2_COR = ((int(data[14]) + int(data[16]))/2, (int(data[15]) + int(data[17]))/2)
            self.CornersCor1 = [int(data[18]), int(data[19])]
            self.CornersCor2 = [int(data[20]), int(data[21])]
            self.CornersCor3 = [int(data[22]), int(data[23])]
            self.CornersCor4 = [int(data[24]), int(data[25])]
            self.GAME_RUNNING = data[26]
            #print(self.GAME_RUNNING)
            #print("coordinates updated")

    def run(self):
            ################################################################
            #           def Threads 
            ################################################################
            ev3_conn = threading.Thread(target=self.run_socket)
            driveController = threading.Thread(target=self.driveControl)
            update_cord = threading.Thread(target=self.connectPosManager())

            ################################################################
            #           start Threads 
            ################################################################
            ev3_conn.start()
            update_cord.start()
            time.sleep(5)
            driveController.start()
            
            while self.RUNNING == True:
                time.sleep(100)


ctrl = PC_Controller()
ctrl.run()

