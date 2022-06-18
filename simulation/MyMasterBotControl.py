import sim  # V-rep library
import sys
import time  # used to keep track of time
import math
from array import array
from PIL import Image as I
from PIL import ImageGrab as Ig
import  MashineVisual as MV
import numpy

class MyBotDownControl(object):

    def __init__(self,clientID) -> object:
        self.clientId=clientID

        errorCode, self.Body = sim.simxGetObjectHandle(clientID, 'MasterBot', sim.simx_opmode_oneshot_wait)  ##BodyHandle
        errorCode, self.Target= sim.simxGetObjectHandle(clientID, 'MYBotTargetPos', sim.simx_opmode_oneshot_wait)
        errorCode, self.TargetM = sim.simxGetObjectHandle(clientID, 'TargetM', sim.simx_opmode_oneshot_wait)
        errorCode, self.TargetOri = sim.simxGetObjectHandle(clientID, 'Orie', sim.simx_opmode_oneshot_wait)

        errorCode, self.q1 = sim.simxGetObjectHandle(clientID, 'RJq1', sim.simx_opmode_oneshot_wait)  ##BackLeftHandle
        errorCode, self.q2 = sim.simxGetObjectHandle(clientID, 'RJq2', sim.simx_opmode_oneshot_wait)  ##BackLeftHandle
        errorCode, self.q3 = sim.simxGetObjectHandle(clientID, 'RJq3', sim.simx_opmode_oneshot_wait)  ##BackLeftHandle
        errorCode, self.q4l = sim.simxGetObjectHandle(clientID, 'PJLq4', sim.simx_opmode_oneshot_wait)  ##BackLeftHandle
        errorCode, self.q4r = sim.simxGetObjectHandle(clientID, 'PJRq4', sim.simx_opmode_oneshot_wait)  ##BackLeftHandle

        errorCode, self.Camera = sim.simxGetObjectHandle(clientID, 'RobKam', sim.simx_opmode_oneshot_wait)
        err, self.resolution, self.image = sim.simxGetVisionSensorImage(clientID, self.Camera, 0,sim.simx_opmode_streaming)

        errorCode, q2 = sim.simxGetJointPosition(self.clientId, self.q2, sim.simx_opmode_streaming)

        errorCode, q1 = sim.simxGetJointPosition(self.clientId, self.q1,sim.simx_opmode_streaming)
        errorCode, q2 = sim.simxGetJointPosition(self.clientId, self.q2,sim.simx_opmode_streaming)




        errorCode, self.maingripp = sim.simxGetObjectHandle(clientID, 'Tq3', sim.simx_opmode_oneshot_wait)  ##BackLeftHandle

        self.l1=0.125
        self.l2=0.12

        self.r=0.1
        self.R=0.14

        if (self.Body==0) or (self.Target==0) or (self.TargetM==0):
            print('Error of initialization: Bad Handles')
            sys.exit('Error of initialization: Bad Handles')
        else:
            print('Good Handles')

        wreal = sim.simxGetObjectVelocity(self.clientId, self.Body, sim.simx_opmode_streaming)

        ##errorCode, PosTg = sim.simxGetObjectPosition(clientID, Target, -1, sim.simx_opmode_streaming)

    def botGetImage(self):

        err, self.resolution, self.image = sim.simxGetVisionSensorImage(self.clientId, self.Camera, 0,sim.simx_opmode_buffer)
        image_byte_array = array('b', (self.image))
        image_byte_array = bytes(image_byte_array)
        image_buffer = I.frombuffer("RGB", (self.resolution[0], self.resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
        image_buffer = image_buffer.rotate(180)
        image_buffer = image_buffer.transpose(I.FLIP_LEFT_RIGHT).convert('RGB')
        image_buffer.save('VidSrobotaCopelia.png')

        open_cv_image = numpy.array(image_buffer)
        open_cv_image = open_cv_image[:, :, ::-1].copy()
        return open_cv_image

    def visLoadimage(self):
        image_buffer=I.open('VidSrobotaCopelia.png')
        return(image_buffer)


    def chassisSetSpeed(self,Vx,Vy,Vz):
        ori=sim.simxGetObjectOrientation(self.clientId,self.Target,-1,sim.simx_opmode_oneshot)
        sim.simxSetObjectParent(self.clientId,self.Target,self.Body,0,sim.simx_opmode_oneshot)
        ori=ori[1]
        ori[1]=ori[1]+Vz
        sim.simxSetObjectPosition(self.clientId,self.Target,self.Body,[0,Vy,-Vx],sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(self.clientId,self.TargetOri,self.Body,ori,sim.simx_opmode_oneshot)


    def roboticArmMove(self,x,y):
        pos=sim.simxGetObjectPosition(self.clientId,self.TargetM,self.Body,sim.simx_opmode_oneshot_wait)
        xr=pos[1][1]
        yr=-pos[1][3]
        self.botSetM(xr+x,yr+y)

    def chassisMove(self,x,y,z):#перемещает вперед на х, вправо га у и поворачивается вправо на z
        sim.simxSetObjectPosition(self.clientId, self.Target, self.Body, [0, y, -x], sim.simx_opmode_oneshot)
        sim.simxSetObjectParent(self.clientId,self.Target,-1,0,sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(self.clientId, self.TargetOri, self.Body, [z,0,0], sim.simx_opmode_oneshot)
        sim.simxSetObjectParent(self.clientId, self.TargetOri, -1, 0, sim.simx_opmode_oneshot)

    def roboticArmMoveTo(self,x,y):
        x=x+0.07
        sim.simxSetObjectPosition(self.clientId,self.TargetM,self.Body,[x,0,-y],sim.simx_opmode_oneshot)


    def roboticArmRecenter(self):
        self.roboticArmMoveTo(0.125,0.12)

    def roboticArmOptimalForward(self):
        self.roboticArmMoveTo(0, 0.18)

    def roboticArmStop(self):
        pos=sim.simxGetObjectPosition(self.clientId,self.TargetM,self.Body,sim.simx_opmode_oneshot_wait)
        sim.simxSetObjectPosition(self.clientId, self.TargetM, self.Body, pos[1], sim.simx_opmode_oneshot)

    def roboticArmReturnPosition(self):#Otladit
        eror,pos=sim.simxGetObjectPosition(self.clientId, self.maingripp, self.q1,sim.simx_opmode_oneshot_wait)
        x=pos[0]
        y=pos[1]
        return x,y

