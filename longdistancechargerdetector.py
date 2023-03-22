
from pydoc import doc
import cv2 as cv
import numpy as np 
import imutils
from imutils.video import VideoStream

#this is for the intel stereo depth camera

import pyrealsense2 as rs
import time

#this is for the adafruit servo and motor controler board

from board import SCL, SDA #may not be needed anymore
import busio
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import adafruit_bno055
import multiprocessing as mp
from multiprocessing import Value
import geopy.distance
import json
from geographiclib.geodesic import Geodesic

#temporarily commented out the bno055 may be fried if it was writted to insted of  the pca9685

i2c = I2C(8) #the imu will use i2c bus 8 (in order to be closer to the front) and the motor controller will use i2c bus 1- a lot is going to need to be changed for this
#this is the setup code for the adafruit imu
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.NDOF_MODE
last_val = 0xFFFF
# the value function allows your to share variables within processes
locationlat = Value('d',0.0)
locationlon = Value('d',0.0)
#this reads the json file with the location of the charger
with open('/home/gilblankenship/Projects/PythonCode/env/main/Charge/longdistance/chargerposition.json') as json_file:
    chargerdict = json.load(json_file)
chargerlat = chargerdict['latitude']
chargerlon = chargerdict['longetude']
chargerlocation = (chargerlat,chargerlon) 

# locationlat.value,locationlon.value = chargerlocation 

#in the other code i do not set the bus, it only takes the scl and sda inputs from the PCA 9685 class, so here because i am dealing woith two diffrent devices i am individually setting them - May not work
i2c2= I2C(1)#busio.I2C(SCL, SDA) #
print(i2c)
print(i2c2)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c2) #i2c2)
pca.frequency = 50
#setting the turning and motor servos as servo 1 and servo 2 
servo1 = servo.Servo(pca.channels[0], actuation_range=10) #this will be how I deal with turning, 0 is right, 5 is straight, 10 is left
servo2 = servo.Servo(pca.channels[1],min_pulse = 400,max_pulse=2400)#same as before write 80 to intialize/set motors to offf, 100 seems about right for driving if it is jnot i will make the range smaller
servo2.angle = 80
time.sleep(4)
print("It should beep")
#functions that tell the adafruit servo controller how to command the motors, now you have to specify motor direction and wheel direction seperatly.
def Left():
    servo1.angle = 10
def Right():
    servo1.angle = 0
def Straight():
    servo1.angle = 5
def Forward():
    servo2.angle = 100
def Back():
    servo2.angle = 60
def Stop():
    servo2.angle = 80

#these values are what the latitude and longetude are set to as, they have to be initalized this this, so they can be passed in between multiprocessing, Value is from either
#multiprocessing or threading library but allows numbers to be passed between multiprocessing functions as global variables do not work
xx = Value('d',0.0)
yy = Value('d',0.0)

def begintrack(): #this is reading the lat and lon points from the ntrip serial code

    while True:
        try:
            with open('/home/gilblankenship/Projects/PythonCode/env/main/Charge/longdistance/location.json') as json_file: 
                locationdict = json.load(json_file)
            json_file.close()
            x = locationdict['latitude']
            xx.value = float(x)
            #print(x)
            y = locationdict['longetude']
            yy.value = float(y)
            
            #print(y)
            
            # current = (xx, yy)
            # distance = geopy.distance.distance(current,final).m
            
        except json.decoder.JSONDecodeError as err:
            #print(err)
            time.sleep(.01)

track = mp.Process(target = begintrack) 

#below reads the bearing values from the imu
yaw = Value('d',0.0)
currTime = time.time()
print_count = 0

def direction():
    global yangle
    global zangle
    global yaw
    while True:
        yaw.value, yangle, zangle = sensor.euler
        
        yaw.value = yaw.value - 10 #this accounts for magnetic vs true north  
        if yaw.value < 0:
            yaw.value = yaw.value + 360
        #print("yaw =",yaw.value)
        #time.sleep(0.1)

dir = mp.Process(target = direction)
distance = Value('d',0.0)
def howfar():
    while True:
        global distance
        final = (locationlat.value, locationlon.value)#needs to be changed to the read values at beginning
        current = (xx.value, yy.value)
        distance.value = geopy.distance.distance(current,final).m
        #print(distance.value)
        #print("distance=",distance.valu

def Gotocharger():
    while distance.value > .75:
        #print(distance.value)
        X = xx.value
        Y = yy.value
        bearing = Geodesic.WGS84.Inverse(X, Y, locationlat.value, locationlon.value)['azi1']
        #print(bearing)
        if bearing <0:
            bearing = bearing + 360
        if bearing >360:
            bearing = bearing - 360
        #print("bearing=",bearing)
        #print("yaw=",yaw.value)
        bearinglow = bearing - 15
        if bearinglow < 0:
            bearinglow = bearinglow + 360
        bearinghigh = bearing + 15
        if bearinghigh > 360:
            bearinghigh = bearinghigh - 360
        print("trying to write")
        if bearinglow < bearinghigh:
            if yaw.value >= bearinglow and yaw.value <= bearinghigh:
                Straight()
                Forward()
            elif yaw.value < bearinglow:
                Forward()
                Right()
            elif yaw.value > bearinghigh:
                Forward()
                Left()
                # time.sleep(1)
            else:
                pass
        elif bearinglow > bearinghigh:
            if yaw.value >= bearinghigh and yaw.value <= bearinglow:
                Straight()
                Forward()
            elif yaw.value < bearinghigh :
                Forward()
                Right()
            elif yaw.value > bearinglow:
                Forward()
                Left()
            else:
                pass
        time.sleep(.1)
    #I believe that it breaks on its own and finished when it is within .75m    
    #this is also going to use ntrip client august 4th








def Finalapproach(show):
    # VIDEO CAPTURE
    #cap_video = cv.VideoCapture(0)
    #setting color limits for the mask
    blueLower = (100, 150, 100)
    blueUpper = (110, 255, 255)
    vs = VideoStream(src=0).start()
    time.sleep(2.0)
    #setting up the stereo deapth camera
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution for stereo depth
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    #setting up the servo and motor controller
    #all below is done above, should not be needed
    #i2c = I2C(1)
    # i2c = busio.I2C(SCL, SDA)
    # # Create a simple PCA9685 class instance.
    # pca = PCA9685(i2c)
    # pca.frequency = 50
    # servo1 = servo.Servo(pca.channels[0], actuation_range=10) #this will be how I deal with turning, 0 is right, 5 is straight, 10 is left
    # servo2 = servo.Servo(pca.channels[1],min_pulse = 400,max_pulse=2400)#same as before write 80 to intialize/set motors to offf, 100 seems about right for driving if it is jnot i will make the range smaller
    servo2.angle = 80
    time.sleep(4)
    # RUNS FOREVER
    while(1):
        #reading the normal camera
        frame = vs.read()
        framers = pipeline.wait_for_frames()
        depth_frame = framers.get_depth_frame()
        #same way it is done for the ball, applies a mask and makes it smaller
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, blueLower, blueUpper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        #this is to test for the size of the frame
        # height,width =frame.shape[:2]
        # print("height=",height,"Width=",width),
        # time.sleep(20)


        
        

        # CANNY EDGE DETECTION, I am not currently using this 

        frameG = cv.cvtColor(frame,cv.COLOR_RGB2GRAY)
        edges = cv.Canny(frameG,200,200)
        thresh = cv.threshold(frameG,127,255,cv.THRESH_BINARY)[1]



        # CALLING SHAPE DETECTION FUNCTION
        try:
            shapes,x,y,shape = shapeDetector(mask,frame.copy())
            #below pulls the center of the shape and takes the average deapth of the 10x10 pixle area around it  
            XL = createList(x-5,x+5)
            YL = createList(y-5,y+5)
            zDepthtot = 0
            avedepth = 0
            divisor = 0
            #zDepth = depth_frame.get_distance(int(x),int(y))
            for ix in XL:
                for iy in YL:
                    zDepthtot =zDepthtot+ depth_frame.get_distance(int(ix),int(iy))
                    if depth_frame.get_distance(int(ix),int(iy)) != 0:
                        divisor = divisor +1
            avedepth = zDepthtot/divisor
        except:
            pass#this ofen happens when something is too close, cause this to set distance to .8<?

        # if (show):
        #     # DISPLAY ORIGINAL
        #     cv.imshow('Original Image',frame)

            
        #     # DISPLAY SHAPE OUTPUT
        #     try:
        #         cv.imshow('Shapes',shapes)
        #     except:
        #         pass
                #print("no shape in view")
        #this is where the movement code and instructions will go, starting assuming that the robot is in front of it and can see it
        if avedepth <.8:
            servo2.angle = 80
            servo1.angle = 5
            break
        else:# avedepth <2:
            if avedepth <1.5: #this is my final approach code, needs to be dialed in
                if 240<=x<=440 and shape =='square':
                    servo1.angle = 5
                    servo2.angle = 100
                else:
                    servo1.angle = 5
                    servo2.angle = 70 #hopefully this makes it back up
            else:
                if avedepth >1.5: #this if shouldent be needed as it is contained in the else
                    print("its trying to write")
                    if 240<=x<=440:
                        servo1.angle = 5
                        servo2.angle = 100
                    elif 0<=x<=240:
                        servo1.angle = 10
                        servo2.angle = 100
                    else:# 440< x <640:
                        servo1.angle = 0
                        servo2.angle = 100



       
        
        cv.waitKey(5)
    #killing pipelines and videos
    vs.stop()
    pipeline.stop()
    # cv.destroyAllWindows()
#simple function to crate a list from number r1 to number r2
def createList(r1, r2):
    return list(range(r1, r2+1))

def shapeDetector(image,origimage):
    
    # RESIZING THE IMAGE
    resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])
    
    # SETTING A THRESHOLD TO CONVERT IT TO BLACK AND WHITE

    # FINDING CONTOURS IN THE B/W IMAGE
    contours = cv.findContours(resized.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)[0]
    
    if len(contours)>0:
        #this only takes the largest contour, we do not want to deal with other small contours messing us up
        cn = max(contours, key = cv.contourArea)

        # for cntour in contours:
            # CALCULATING THE CENTERgray = cv.cvtColor(resized, cv.COLOR_BGR2GRAY)
        
        shape = detect(cn)#tour)
        #finds the center, resizes, and draws the contours on the image, could lose some of this as it is not for viewing
        M = cv.moments(cn)#tour)
        if (M["m00"] == 0):
            cX = 0
            cY = 0
        else:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
        cntour = cn.astype("float")
        cn = cn*ratio
        cn = cn.astype("int")
        cv.drawContours(origimage,[cn],-1,(34,0,156),2)
        cv.putText(origimage, shape, (cX, cY), cv.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 255, 255), 2)
        return(origimage,cX,cY,shape)
    else:
        pass

    # NEEDS TO BE REPLACED BY K MEANS CLUSTERING INSTEAD OF CONTOUR MAPPING the last person  that wrote this said this, i am not sure how it would be applied, probably not applicable using the mask method i am.


#this determines the shape based on the points
def detect(c):
    shape = "unidentified"
    peri = cv.arcLength(c,True)
    approx = cv.approxPolyDP(c, 0.04 * peri, True)
    if len(approx) == 3:
        shape = "triangle"

    elif len(approx) == 4:
        (_, _, w, h) = cv.boundingRect(approx)
        ar = w / float(h)
        shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

    elif len(approx) == 5:
        shape = "pentagon"

    else:
        shape = "circle"

    return shape


if __name__ == "__main__":
    far = mp.Process(target = howfar)
    track.start()
    print("tracking")
    time.sleep(3)
    dir.start()
    input("please move robot around in all directions, press enter when done")
    #time.sleep(5)
    print("finding direction")
    h = sensor.calibration_status
    print(h)
    far.start()
    time.sleep(5)
    print("measuring distance")

    Gotocharger()
    Stop() #- unsure about this and the next one, make sure they are needed.
    Straight()
    Finalapproach(1)