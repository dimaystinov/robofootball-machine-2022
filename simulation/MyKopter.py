import sim  # V-rep library
import sys
import time  # used to keep track of time
import math
import numpy
import cv2
from array import array
from PIL import Image as I

class MyKopterBot(object):

    def __init__(self,clientID):
        self.clientId = clientID
        print(self.clientId )
        errorCode, self.Body = sim.simxGetObjectHandle(self.clientId, 'Kopter', sim.simx_opmode_oneshot_wait)  ##BodyHandle
        print(self.Body)
        errorCode, self.Camera = sim.simxGetObjectHandle(self.clientId, 'KopS', sim.simx_opmode_oneshot_wait)


        err, self.resolution, self.image = sim.simxGetVisionSensorImage(clientID, self.Camera, 0, sim.simx_opmode_streaming)
        print(self.resolution)

        self.Vx=0.1
        self.Vy=0.1
        self.Vz=0.1
        self.Wz=0.1
        time.sleep(1)

        if (self.Body==0) or (self.Camera==0):
            print('Error of initialization: Bad Handles')
            sys.exit('Error of initialization: Bad Handles')
        else:
            print('Good Handles')

    def kopMove(self,X,Y,Z,A):
        operation = True
        while operation:
            position = sim.simxGetObjectPosition(self.clientId, self.Body, -1, sim.simx_opmode_oneshot_wait)
            # print('position-',position)
            x = position[1][0]
            y = position[1][1]
            z = position[1][2]
            angle=1
            alpha = self.kopOrientation()
            t1 = sim.simxGetLastCmdTime(self.clientId)

            time.sleep(0.1)
            self.kopOrientation()
            t2 = sim.simxGetLastCmdTime(self.clientId)

            arealP=self.opAngleConvert(A,alpha[angle])
            dx=X-x
            dy=Y-y
            dz=Z-z
            da=A-arealP

            if abs(dx)<self.Vx:
                Vx=dx
            else:
                Vx=(self.Vx)*(dx/abs(dx))

            if abs(dy)<self.Vy:
                Vy=dy
            else:
                Vy=(self.Vy)*(dy/abs(dy))

            if abs(dz)<self.Vz:
                Vz=dz
            else:
                Vz=(self.Vz)*(dz/abs(dz))

            if abs(da) < self.Wz:
                Wz = -da
            else:
                Wz = (self.Wz) * (-da / abs(da))


            X2 = x + Vx
            Y2 = y + Vy
            Z2 = z + Vz
            angle2=orientation = sim.simxGetObjectOrientation(self.clientId, self.Body, self.Body, sim.simx_opmode_oneshot_wait)
            print(angle2)
            pa=angle2[1]
            pa[0] =Wz
            sim.simxSetObjectPosition(self.clientId,self.Body,-1,[X2, Y2, Z2],sim.simx_opmode_oneshot_wait)
            sim.simxSetObjectOrientation(self.clientId,self.Body,self.Body,pa,sim.simx_opmode_oneshot_wait)

            if (abs(Vx)<0.05) and (abs(Vy)<0.05) and (abs(Vz)<0.05) and (abs(Wz)<0.1):
                operation=False


    def kopMoveOtn(self,x,y,z,A):
        x=-x
        alpha=self.kopOrientation()
        alpha=alpha[1]+A
        eroror,pos=sim.simxGetObjectPosition(self.clientId, self.Body, -1, sim.simx_opmode_oneshot_wait)
        alphab2=alpha+math.pi/2
        X=x*math.sin(alphab2)+y*math.cos(alphab2)
        Y=x*math.cos(alphab2)+y*math.sin(alphab2)
        Xn=pos[0]+X
        Yn=pos[1]+Y
        Zn=pos[2]+z
        self.kopMove(Xn,Yn,Zn,alpha)

    def opAngleConvert(self,atr,areal):

        if (areal>0) and (atr>0):
            aout=areal
        elif (areal<0) and (atr<0):
            aout=areal
        else :
            if abs(atr-areal)<math.pi:
                aout = areal
            else:
                if areal>0:
                    aout = atr - math.pi * 2
                elif areal<0:
                    aout = atr + math.pi * 2
                else:
                    aout = areal
        #print(aout)
        return aout

    def kopOrientation(self):
        orientation = sim.simxGetObjectOrientation(self.clientId, self.Body, -1, sim.simx_opmode_oneshot_wait)
        return orientation[1]

    def kopGetImage(self):
        err, self.resolution, self.image = sim.simxGetVisionSensorImage(self.clientId, self.Camera, 0,sim.simx_opmode_buffer)
        image_byte_array = array('b', (self.image))
        image_byte_array = bytes(image_byte_array)
        image_buffer = I.frombuffer("RGB", (self.resolution[0], self.resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
        image_buffer=image_buffer.rotate(180)
        image_buffer = image_buffer.transpose(I.FLIP_LEFT_RIGHT)
        image_buffer.save('VidSrobotaCopelia.png')
        return image_buffer

    def visLoadimage(self):
        image_buffer=I.open('VidSKopteraCopelia.png')
        return(image_buffer)

    def visGetAngleLine(self):
        image_buffer = self.kopGetImage()
        img2 = numpy.asarray(image_buffer)
        ret=self.visSearchDown(img2,'Green')

        if ret:
            cv2.cv2.circle(img2, (ret[1], ret[0]), 2, (0xff, 0xf4, 0x0d), 2)


        image_buffer = I.fromarray(img2)
        image_buffer.save('DownT.png')


    def visCentrObject(self,Color):
        image_buffer = self.kopGetImage()
        imgcv = numpy.asarray(image_buffer)

        if Color=='Green':
            ret = self.track_green_object(imgcv)

        elif Color=='Red':
            ret=self.track_red_object(imgcv)
        elif Color=='Blue':
            ret=self.track_blue_object(imgcv)
        else:
            ret=[0,0]

        if ret:
            cv2.rectangle(imgcv, (ret[0] - 15, ret[1] - 15), (ret[0] + 15, ret[1] + 15), (0xff, 0xf4, 0x0d), 1)

        # img2 = img2.ravel()
        image_buffer = I.fromarray(imgcv)
        if Color=='Green':
            image_buffer.save('VidSrobotaCopeliaGreenObject.png')

        elif Color=='Red':
            image_buffer.save('VidSrobotaCopeliaRedObject.png')
        elif Color=='Blue':
            image_buffer.save('VidSrobotaCopeliaBlueObject.png')




    def visSearchDown(self,Image, color):
        I = self.resolution[0]
        J = self.resolution[1]
        centr = 0
        set = 0
        if color == "Green":
            Mask = self.visGreenMask(Image)

        for j in range(0, J - 1):
            thisRay= False
            centrc = 0
            setc = 0
            for i in range(0, I):
                if (Mask[J - j - 1][i] > 0):
                    thisRay = True
                    centrc = centrc + i
                    setc = setc + 1

            if thisRay:
                centrc = int(centrc / setc)
                setc = J - j - 1
                Colomn = centrc
                Row = setc
                break

        return Row, Colomn

    def opvisAngle(self,Colomn):
        k = (8 / 60)# угловое разрешение камеры
        xmidle = self.resolution[0] / 2
        dx = Colomn - xmidle


        da = dx * k
        return da

    def visGreenMask(self,image):


        # Convert BGR to RGB
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Threshold the HSV image for only green colors
        lower_green = numpy.array([1, 100, 1])
        upper_green = numpy.array([100, 300, 100])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)


        # Blur the mask
        bmask = cv2.GaussianBlur(mask, (5, 5), 0)
        return (bmask)


    def kopVisualSensing(self):
        image_buffer=self.kopGetImage()
        img2 = numpy.asarray(image_buffer)

        ret = self.track_green_object(img2)

        if ret:
            cv2.rectangle(img2, (ret[0] - 15, ret[1] - 15), (ret[0] + 15, ret[1] + 15), (0xff, 0xf4, 0x0d), 1)

        #img2 = img2.ravel()
        image_buffer=I.fromarray( img2)



    def track_green_object(self,image):

        # Blur the image to reduce noise
        blur = cv2.GaussianBlur(image, (5, 5), 0)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image for only green colors
        lower_green = numpy.array([40, 70, 70])
        upper_green = numpy.array([80, 200, 200])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Blur the mask
        bmask = cv2.GaussianBlur(mask, (5, 5), 0)

        # Take the moments to get the centroid
        moments = cv2.moments(bmask)
        m00 = moments['m00']
        centroid_x, centroid_y = None, None
        if m00 != 0:
            centroid_x = int(moments['m10'] / m00)
            centroid_y = int(moments['m01'] / m00)

        # Assume no centroid
        ctr = None

        # Use centroid if it exists
        if centroid_x != None and centroid_y != None:
            ctr = (centroid_x, centroid_y)
        return ctr

    def track_red_object(self,image):


        blur = cv2.GaussianBlur(image, (5, 5), 0)

        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lower_red = numpy.array([100, 40,40])
        upper_red = numpy.array([300, 80, 80])


        mask = cv2.inRange(hsv, lower_red, upper_red)
        bmask = cv2.GaussianBlur(mask, (5, 5), 0)

        moments = cv2.moments(bmask)
        m00 = moments['m00']
        centroid_x, centroid_y = None, None
        if m00 != 0:
            centroid_x = int(moments['m10'] / m00)
            centroid_y = int(moments['m01'] / m00)

        ctr = None

        if centroid_x != None and centroid_y != None:
            ctr = (centroid_x, centroid_y)
        return ctr

    def track_blue_object(self,image):

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        lower_blue = numpy.array([1, 1,1])
        upper_blue = numpy.array([255, 50, 50])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        bmask = cv2.GaussianBlur(mask, (5, 5), 0)

        moments = cv2.moments(bmask)
        m00 = moments['m00']
        centroid_x, centroid_y = None, None
        if m00 != 0:
            centroid_x = int(moments['m10'] / m00)
            centroid_y = int(moments['m01'] / m00)

        ctr = None

        if centroid_x != None and centroid_y != None:
            ctr = (centroid_x, centroid_y)
        return ctr

    def otob(self,mask):
        cv2.namedWindow("result")
        cv2.imshow('result', mask)
        while 1:
            ch = cv2.waitKey(1)

    def dronUp(self,l):
        self.kopMoveOtn(0,0,l,0)

    def dronDown(self,l):
        self.kopMoveOtn(0,0,-l,0)

    def dronLeft(self,l):
        self.kopMoveOtn(0, -l, 0,0)

    def dronRight(self,l):
        self.kopMoveOtn(0, l, 0,0)

    def dronFoward(self,l):
        self.kopMoveOtn(l, 0, 0,0)

    def dronBack(self,l):
        self.kopMoveOtn(-l, 0, 0,0)

    def dronRotateRight(self,l):
        self.kopMoveOtn(0,0,0,l)

    def dronRotateLeft(self,l):
        self.kopMoveOtn(0,0,0,-l)



