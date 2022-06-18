import numpy
import cv2
from array import array
from PIL import Image as I


def otob(mask):
    cv2.namedWindow("result")
    cv2.imshow('result', mask)
    while 1:
        ch = cv2.waitKey(1)

def visLoadimage():
    image_buffer = I.open('ImageReal3.png')
    return (image_buffer)

def visSaveimage(image_buffer):
     image_buffer.save('DownT3.png')

def potctroika():
    print('0')
    img = Ig.grab((0, 300, 950, 800))
    img.save('buffer.png')


def getRealImageBot():
    img = Ig.grab((0,300, 950, 800))
    img.save('bufferBot.png')
    return(img)

def getRealImageKopter():
    img = Ig.grab((0,300, 950, 800))
    img.save('bufferKoP.png')
    return(img)




def visGetAngleLineBot(Image,color):#Возврает координаты до нижнего вхождения линии цвета Color
    image_buffer = Image
    resolution = Image.size
    img2 = numpy.asarray(image_buffer)
    ret = visSearchDown(img2,resolution, color)
    ret0 = visSearchUP(img2,resolution, color)

    if ret:
        cv2.circle(img2, (ret[1], ret[0]), 2, (0xff, 0xf4, 0x0d), 2)

    if ret0:
        cv2.circle(img2, (ret0[1], ret0[0]), 2, (0, 0xf4, 0x0d), 2)

    Colomn=ret[1]
    k = (7 / 60)  # угловое разрешение камеры
    xmidle = resolution[0] / 2
    dx = Colomn - xmidle
    da = dx * k
    Colomn=ret0[1]

    k = (7 / 60)  # угловое разрешение камеры
    xmidle = resolution[0] / 2
    dx = Colomn - xmidle
    da2 = dx * k
    if Colomn == 0:
        da2=0

    image_buffer = I.fromarray(img2)
    image_buffer.save('DownT2.png')
    return(da,da2)



def visGetAngleCentrObBot(Image,color):
    image_buffer=Image
    resolution=Image.size
    img2 = numpy.asarray(image_buffer)

    ret,l = track_centr_object(img2,color)
    if (ret == None):
        return (None,None)

    if ret:
        cv2.rectangle(img2, (ret[0] - 15, ret[1] - 15), (ret[0] + 15, ret[1] + 15), (0xff, 0xf4, 0x0d), 1)
        cv2.circle(img2, (int(resolution[0]/2), int(resolution[1]-1)), 2, (0xff, 0xf4, 0x0d), 2)

    Colomn=ret[0]
    k = (7 / 60)  # угловое разрешение камеры
    xmidle = resolution[0] / 2
    dx = Colomn - xmidle
    print(dx)
    da = dx * k
    image_buffer = I.fromarray(img2)
    image_buffer.save('CentrObject.png')
    return da,l

def track_centr_object(image,color):
    if color == "Green":
        bmask = visGreenMask(image)
    elif color == "Red":
        bmask = visRedMask(image)



    # Take the moments to get the centroid
    moments = cv2.moments(bmask)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10'] / m00)
        centroid_y = int(moments['m01'] / m00)

    if (centroid_x == None):
        return (None, None)
    h=500-centroid_y
    k=0.925/250
    l=h*k


    # Assume no centroid
    ctr = None

    # Use centroid if it exists
    if centroid_x != None and centroid_y != None:
        ctr = (centroid_x, centroid_y)
    return ctr,l

def visSearchDown(Image,resolution, color):
    Row=0
    Colomn=0
    I = resolution[0]
    J = resolution[1]
    centr = 0
    set = 0
    if color == "Green":
        Mask = visGreenMask(Image)
    elif color == "Red":
        Mask = visRedMask(Image)


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

def visSearchUP(Image,resolution, color):
    Row=0
    Colomn=0
    I = resolution[0]
    J = resolution[1]
    centr = 0
    set = 0
    if color == "Green":
        Mask = visGreenMask(Image)
    elif color == "Red":
        Mask = visRedMask(Image)


    for j in range((int(J*0.1)), J - 1):
        thisRay= False
        centrc = 0
        setc = 0
        for i in range(0, I-1):
            if (Mask[j][i] > 0):
                thisRay = True
                centrc = centrc + i
                setc = setc + 1

        if thisRay:
            centrc = int(centrc / setc)
            setc = j
            Colomn = centrc
            Row = setc
            break

    return Row, Colomn

def visGreenMask(image):
   # Convert BGR to RGB
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Threshold the HSV image for only green colors
    lower_green = numpy.array([10, 40, 0])
    upper_green = numpy.array([50, 70,0])

        # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)


        # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5, 5), 0)
    return (bmask)

def visRedMask(image):
   # Convert BGR to RGB
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Threshold the HSV image for only green colors
    lower_red = numpy.array([1, 1, 100])
    upper_red = numpy.array([100, 100, 300])

        # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_red, upper_red)


        # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5, 5), 0)
    return (bmask)



def FULAngleLineBot(Color):
    Img=visLoadimage()
    res = visGetAngleLineBot(Img, Color)
    return res

def FULAngleCentrObBot(Color):
    Img=visLoadimage()
    res,l = visGetAngleCentrObBot(Img, Color)
    if (res == None):
        return (None)
    return -res

def FULDistanceCentrObBot(Color):
    Img=visLoadimage()
    res,l = visGetAngleCentrObBot(Img, Color)
    if (res == None):
        return (None)
    return l






