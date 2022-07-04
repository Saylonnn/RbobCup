import socket
import keyboard
import time
from controlSocket import ControlSocket
import cv2
import threading
import numpy as np
import math
import pickle

class PC_Controller:
    def __init__(self):
        self.IS_CONNECTED = False
        self.EV3_DEV = 1
        self.EvFrontCor = None
        self.EvBackCor = None
        self.BallCor = None
        self.GAME_RUNNING = True
        self.GOAL_1_COR = None
        self.GOAL_2_COR = None
        self.RUNNING = True
    
    
       
    def driveControl(self):
        print("test Angle")

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

        '''
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

    def driveToStart(self):
        print("drive to start")
        if self.EV3_DEV == 1:
            startPos = (self.GOAL_1_COR[0]+100, self.GOAL_1_COR[1])
            self.sock.hold()
            self.calculateAngle(self.EvFrontCor, self.EvBackCor, startPos)
            self.sock.fw()
            while self.messureDistance(self.EvFrontCor, startPos) > 0:
                time.sleep(10)
            self.sock.hold()
            degree = self.calculateAngle(self.EvFrontCor, self.EvBackCor, (512, 393))
            self.sock.rotateAngle(degree)

        if self.EV3_DEV == 2:
            startPos = (self.GOAL_2_COR[0]-100, self.GOAL_2_COR[1])
            self.sock.hold()
            self.calculateAngle(self.EvFrontCor,self.EvBackCor, startPos)
            self.sock.fw()
            while self.messureDistance(self.EvFrontCor, startPos) > 0:
                time.sleep(10)
            self.sock.hold()
            degree = self.calculateAngle(self.EvFrontCor, self.EvBackCor, (512, 393))
            self.sock.rotateAngle(degree)


 

    def messureDistance(self, ev3Front, ballCor):
        #print("messure distance")
        distance = math.sqrt((ev3Front[0] - ballCor[0])**2 + (ev3Front[1] - ballCor[1])**2)
        return distance

    def calculateAngle(self, ev3_vorne, ev3_hinten, ball_koor):
        #print("calculate Angle")
        ev3_vorne = np.array(ev3_vorne)
        ev3_hinten = np.array(ev3_hinten)
        ball_koor = np.array(ball_koor)
        ev3_mittig = np.array([ev3_vorne[0]-(ev3_vorne[0] - ev3_hinten[0])/2, ev3_vorne[1]-(ev3_vorne[1] - ev3_hinten[1])/2])

        ev3_vec = ev3_vorne - ev3_mittig
        toBall_vec = ball_koor - ev3_mittig


        #calculate_Angle
        #Skalarprodukt --> a_1 * b_1 +.... a_n + b_n
        skalar_produkt = ev3_vec.dot(toBall_vec)


        #berechne vectoren länge
        #länge_ev3_vec =  sqrt(a[0]^2 + ... + a[n]^2)
        x = 0
        for i in range(0, ev3_vec.size):
            x = x + ev3_vec[i]**2
        length_ev3_vec = math.sqrt(x)
       # print("length_ev3_vec: ", length_ev3_vec)
       
        x = 0
        for i in range(0, toBall_vec.size):
            x = x + toBall_vec[i]**2
        length_ball_vec = math.sqrt(x)
        #print("length_ball_vec: ", length_ball_vec)

        degree = math.degrees(math.acos(skalar_produkt / (length_ball_vec * length_ev3_vec)))
        if degree > 180:
            degree = -(360 - degree)
        return int(degree)
        

    def startGame(self):
        self.GAME_STARTED = True
    
    def run_socket(self):
        self.sock = ControlSocket()
        self.sock.run()
        print("control Socket started")


    '''
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
    '''
            
            
       
    def updateCoords(self):
        cap = cv2.VideoCapture("rtsp://141.46.137.93:8554/mystream")
        # initialize green ball
        green_1 = cv2.imread('PatternPics\GreenBall.png', -1)
        # initialize red ball
        red = cv2.imread('ballRotNeu.png', -1)
        # initialize yellow ball
        yellow = cv2.imread('PatternPics/YellowBall.png', -1)
        # initialize blue ball
        blue = cv2.imread('PatternPics/echterBallBlau.png', -1)
        # initialize black ball
        black = cv2.imread('PatternPics/BlackBall.png', -1)
        while (cap.isOpened()):
            ret, frame = cap.read()
            # ...resize the image by a half
            # frame = cv2.resize(frame,(0,0),fx=0.5, fy=0.5)
            cv2.imshow('frame', frame)

            # green ball tracking
            resGreen = cv2.matchTemplate(frame, green_1, 5)

            cv2.imshow('PatternPics\GreenBall.png', green_1)

            green_min_val, green_max_val, green_min_loc, green_max_loc = cv2.minMaxLoc(resGreen)

            green_top_left = green_max_loc
            green_bottom_right = (green_top_left[0] + 37, green_top_left[1] + 37)
            green_top_left_final = (green_top_left[0] + 17, green_top_left[1] + 17)

            # Koordinaten der Mitte des Balls
            print("Green Ball: ", green_top_left_final)

            cv2.rectangle(frame, green_top_left_final, green_bottom_right, 255, 2)

            # red ball tracking

            resRed = cv2.matchTemplate(frame, red, 5)
            red_min_val, red_max_val, red_min_loc, red_max_loc = cv2.minMaxLoc(resRed)

            red_top_left = red_max_loc
            red_bottom_right = (red_top_left[0] + 35, red_top_left[1] + 32)
            red_top_left_final = (red_top_left[0] + 15, red_top_left[1] + 12)

            # Koordinaten der Mitte des Balls
            print("Red Ball: ", red_top_left_final)

            if red_top_left_final[0] > 56 & red_top_left_final[0] < 908 & red_top_left_final[1] > 105 & \
                    red_top_left_final[1] < 638:
                print("red ball inside")
            else:
                print("red ball outside")

            cv2.rectangle(frame, red_top_left_final, red_bottom_right, 255, 2)

            # yellow ball tracking

            resYellow = cv2.matchTemplate(frame, yellow, 5)
            yellow_min_val, yellow_max_val, yellow_min_loc, yellow_max_loc = cv2.minMaxLoc(resYellow)

            yellow_top_left = yellow_max_loc
            yellow_bottom_right = (yellow_top_left[0] + 39, yellow_top_left[1] + 40)
            yellow_top_left_final = (yellow_top_left[0] + 19, yellow_top_left[1] + 20)

            # Koordinaten der Mitte des Balls
            print("Yellow Ball: ", yellow_top_left_final)

            cv2.rectangle(frame, yellow_top_left_final, yellow_bottom_right, 255, 2)

            # black ball tracking

            resBlack = cv2.matchTemplate(frame, black, 5)
            black_min_val, black_max_val, black_min_loc, black_max_loc = cv2.minMaxLoc(resBlack)

            black_top_left = black_max_loc
            black_bottom_right = (black_top_left[0] + 33, black_top_left[1] + 34)
            black_top_left_final = (black_top_left[0] + 13, black_top_left[1] + 14)

            # Koordinaten der Mitte des Balls
            print("Black Ball: ", black_top_left_final)

            cv2.rectangle(frame, black_top_left_final, black_bottom_right, 255, 2)

            # blue ball tracking

            resBlue = cv2.matchTemplate(frame, blue, 5)
            blue_min_val, blue_max_val, blue_min_loc, blue_max_loc = cv2.minMaxLoc(resBlue)

            blue_top_left = blue_max_loc
            blue_bottom_right = (blue_top_left[0] + 33, blue_top_left[1] + 35)
            blue_top_left_final = (blue_top_left[0] + 13, blue_top_left[1] + 15)

            # Koordinaten der Mitte des Balls
            print("Blue Ball: ", blue_top_left_final)

            cv2.rectangle(frame, blue_top_left_final, blue_bottom_right, 255, 2)

            cv2.rectangle(frame, (56, 105), (908, 638), 255, 2)

            cords_1 = [str(red_top_left_final[0]), ",", str(red_top_left_final[1]), ",", str(black_top_left_final[0]),
                       ",", str(black_top_left_final[1]), ",", str(blue_top_left_final[0]), ",",
                       str(blue_top_left_final[1]), ",", str(yellow_top_left_final[0]), ",",
                       str(yellow_top_left_final[1]), ",",
                       str(green_top_left_final[0]), ",", str(green_top_left_final[1]), ",", str(57), ",", str(286),
                       ",", str(57), ",", str(461), ",", str(947), ",", str(283), ",", str(947), ",", str(458), ",",
                       str(56), ",", str(105), ",", str(56), ",", str(638), ",", str(908), ",", str(105), ",", str(908),
                       ",", str(638), ",", str(True)]

            s = ''.join(cords_1)
            data = s.split(",")
            self.BallCor = [int(data[0]), int(data[1])]
            if self.EV3_DEV == 1:
                self.EvFrontCor = [int(data[2]), int(data[3])]
                self.EvBackCor = [int(data[4]), int(data[5])]
            if self.EV3_DEV == 2:
                self.EvFrontCor = [int(data[6]), int(data[7])]
                self.EvBackCor = [int(data[8]), int(data[9])]
            self.GOAL_1_COR = ((int(data[10]) + int(data[12])) / 2, (int(data[11]) + int(data[13])) / 2)
            self.GOAL_2_COR = ((int(data[14]) + int(data[16])) / 2, (int(data[15]) + int(data[17])) / 2)
            self.CornersCor1 = [int(data[18]), int(data[19])]
            self.CornersCor2 = [int(data[20]), int(data[21])]
            self.CornersCor3 = [int(data[22]), int(data[23])]
            self.CornersCor4 = [int(data[24]), int(data[25])]
            self.GAME_RUNNING = data[26]

            cv2.imshow("Video", frame)  # Anzeige des Videoframes
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()  # Close all windows

    def run(self):
            ################################################################
            #           def Threads 
            ################################################################
            #ev3_conn = threading.Thread(target=self.run_socket)
            driveController = threading.Thread(target=self.driveControl)
            update_cord = threading.Thread(target=self.updateCoords)

            ################################################################
            #           start Threads 
            ################################################################
            #ev3_conn.start()
            update_cord.start()
            time.sleep(5)
            #driveController.start()
            
            while self.RUNNING == True:
                time.sleep(100)


ctrl = PC_Controller()
ctrl.run()

