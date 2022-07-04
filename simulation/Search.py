import sim  # V-rep library
import sys
from MashineVisual import *
import numpy as np


from MyMasterBotControl import MyBotDownControl


#cv2.namedWindow("result")
class Control():

    def __init__(self,RF):
        if RF:
            sim.simxFinish(-1)  # just in case, close all opened connections

            clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

            if clientID != -1:  # check if client connection successful
                print('Connected to remote API server')

            else:
                print('Connection not successful')
                sys.exit('Could not connect')

            self.clientId = clientID
            self.MB = MyBotDownControl(clientID)
            #self.MC = MyKopterBot(clientID)
            self.ReOfWork=RF##если правда то модель иначе реальны
        else:
            self.ReOfWork=RF

            #вставить необходимые команды
            print('Real')

    def chassisMove(self, x, y, z):
        if self.ReOfWork:
            self.MB.chassisMove(x, y, z)
        else:
            print('movereal')

    def chassisSetSpeed(self,Vx,Vy,Vz):
        if self.ReOfWork:
            self.MB.chassisSetSpeed(Vx,Vy,Vz)
        else:
            print('Setvelositureal')



    def roboticArmMoveTo(self,x,y):
        if self.ReOfWork:
            self.MB.roboticArmMoveTo(x,y)
        else:
            print('moverealManipulator')

    def roboticArmMove(self,dx,dy):# перемещение манипулятора на +х + у
        if self.ReOfWork:
            self.MB.roboticArmMove(dx,dy)
        else:
            print('moverealManipulator')

    def roboticArmRecenter(self):
        if self.ReOfWork:
            self.MB.roboticArmRecenter()
        else:
            print('Realrecenter')

    def roboticArmOptimalForward(self):
        if self.ReOfWork:
            self.MB.roboticArmOptimalForward()
        else:
            print('Realrecenter')


    def roboticArmStop(self):
        if self.ReOfWork:
            self.MB.roboticArmStop()
        else:
            print('RealStop')


    def roboticArmReturnPosition(self):
        if self.ReOfWork:
            self.MB.roboticArmReturnPosition()
        else:
            print('RealReturnPos')

    def roboticGripperClose(self,force):
        print('returnpos')

    def roboticGripperOpen(self,force):
        print('returnpos')

    def roboticGripperStatus(self):
        print('returnpos')

BLOBSIZE = 10

def checkSize(w, h):
    if w * h > BLOBSIZE:
        return True
    else:
        return False


K = 5600


if __name__ == '__main__':
    Co=Control(True)

    # поиск мяча
    while True:
        frame = Co.MB.botGetImage()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, (0, 0, 0), (8, 255, 255))

        result = cv2.bitwise_and(frame, frame, mask=mask)

        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            if checkSize(w, h):
                break
            else:
                Co.chassisSetSpeed(0, 0, 10)


    Co.chassisSetSpeed(0.5,0,0)
    #Co.roboticArmMove(0,0)
    print(type(Co.MB.botGetImage()))
    while True:

        frame = Co.MB.botGetImage()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, (0, 0, 0), (8, 255, 255))

        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('result', result)

        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            if checkSize(w, h):
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                #cv2.imshow('frame', Co.MB.botGetImage())

                # определим расстояние до мяча
                Lm = (w + h) / 2
                length = K / Lm

                # определим угол
                if length != 0:
                    sin = (K / (x + w / 2 - 160)) / length
                    sin = abs(sin)
                    if sin < 1:
                        angle = np.arcsin(sin)
                        angle = angle * 180 / 3.14159265359
                        print(length, angle)
                    else:
                        print(length)




        cv2.waitKey(0)
        cv2.destroyAllWindows()
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break


