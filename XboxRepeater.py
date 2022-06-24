# -*- coding: utf-8 -*-
"""
Created on Wed Jun  8 19:16:12 2022

@author: Andrew Ng
"""

from inputs import get_gamepad
import math
import threading
import matplotlib.pyplot as plt

import serial
import struct
import time


def userToServo(): # use with programmed bed
    st = int(input("Steer:"))
    th = input("Throttle:")
    if th == "":
        th = 1000
    th = int(th)
    return [st, th]

def convertBytes(b):
    allData = struct.unpack('<ff?hhh?ffffB',b)
    if allData[-1] == 96:
        return allData[0:-1]
    else:
        return []
    
def sendBytes(data,s):
    
    
    p = struct.pack('<Bff?hhh?B', 69, data[0], data[1], data[2], data[3], data[4], data[5], data[6], 96)
    
    s.write(p)
    
    
def getData(scomm):
    startByte = scomm.read()
    if startByte != b'':
        if startByte[0] == 69:
            body = scomm.read(33) # 18 before return IMU data (17), 34 after (33)
            data = convertBytes(body)
            
            return [body,data]

def fixFileErrors(fname):
    f = open(fname+'.txt','r')
    ff = open(fname + '_fixed.txt','w')
    ls = f.readlines()
    for i in range(len(ls)):
        if ls[i].count('\t') != 10:
            print(i)
            print(ls[i])
        else:
            ff.write(ls[i])
    f.close()
    ff.close()


class TestbedController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        
        self.MaxThrottle = 90
        self.MaxSteering = 30.0
        

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0
        
        self.DPadSumX = 0
        self.DPadSumY = 0
        self.BumperSum = 0
        self.ManualDriveMode = True # True - manual driving, False - programmed manouver

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def getServoValues(self): # return the buttons/triggers that you care about in this methode
        steer = round(90 - self.LeftJoystickX*self.MaxSteering,3)
        throttle = round(90 + self.RightTrigger*self.MaxThrottle - self.LeftTrigger*self.MaxThrottle,3)
        
        return [steer,throttle]
    
    def convertToServo(self, steering, throttle):
        steer = round(90 - steering*self.MaxSteering,3)
        throttle = round(90 + throttle*self.MaxThrottle,3)
        
        return [steer,throttle]
    
    def printControllerData(self):
        print("LJX\t\tLJY\t\tLT\t\tRT\t\tLB\t\tRB\t\tA\t\tX\t\tY\t\tB\t\tSLCT\tdL\t\tdR\t\tdU\t\tdD")
        cList = [self.LeftJoystickX,self.LeftJoystickY,self.LeftTrigger,self.RightTrigger,self.LeftBumper,self.RightBumper,self.A,self.X,self.Y,self.B,self.Back,self.LeftDPad,self.RightDPad,self.UpDPad,self.DownDPad]
        
        for val in cList:
            print(str(round(val,1)) + "\t\t", end="")
        print(self.DPadSumX, self.DPadSumY, self.BumperSum, self.ManualDriveMode)
        
    def getProgPars(self):
        if self.A == 1:
            aBool = True
        else:
            aBool = False
        
        controlSpeed = (self.RightTrigger - self.LeftTrigger)/1
        # if self.BumperSum != 0 & self.ManualDriveMode == True:
        #     controlSpeed = 0.02*self.BumperSum
            
        if self.BumperSum != 0:
            MaxS = 0.02*self.BumperSum
            MinS = -1*MaxS
            if controlSpeed > MaxS:
                controlSpeed = MaxS
            elif controlSpeed < MinS:
                controlSpeed = MinS
            
        SC = 4
        controlSteer = self.LeftJoystickX + 0.02*SC
        if self.DPadSumX != 0:
            controlSteer = 0.02*self.DPadSumX
            
        progPars = [controlSteer, controlSpeed, self.ManualDriveMode,self.DPadSumX,self.DPadSumY,self.BumperSum,aBool]
        return progPars
        
        

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / TestbedController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / TestbedController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / TestbedController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / TestbedController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / TestbedController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / TestbedController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    if self.LeftBumper == 0:
                        self.BumperSum -= 1
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    if self.RightBumper == 0:
                        self.BumperSum += 1
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_WEST':
                    self.X = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_START':
                    if self.Back == 0:
                        self.ManualDriveMode = not(self.ManualDriveMode)
                    self.Back = event.state
                elif event.code == 'BTN_SELECT':
                    self.Start = event.state
                elif event.code == 'ABS_HAT0X':
                    if event.state < 0: # left dpad
                        if self.LeftDPad == 0: # press down
                            self.DPadSumX -= 1
                        self.LeftDPad = 1
                    elif event.state > 0:
                        if self.RightDPad == 0: # press down
                            self.DPadSumX += 1
                        self.RightDPad = 1
                    else:
                        self.RightDPad = 0
                        self.LeftDPad = 0
                elif event.code == 'ABS_HAT0Y':
                    if event.state < 0: # up dpad
                        if self.UpDPad == 0:
                            self.DPadSumY += 1
                        self.UpDPad = 1
                    elif event.state > 0:
                        if self.DownDPad == 0:
                            self.DPadSumY -= 1
                        self.DownDPad = 1
                    else:
                        self.UpDPad = 0
                        self.DownDPad = 0
                elif event.code == 'BTN_TRIGGER_HAPPY8':
                    self.DownDPad = event.state



startTime = 0
lastTime = 0
intervalTime = 0.01

ys = []
ts = []
# if shit is laggy, close the whole kernal, interrupt doesn't help

startFlag = ""

ALLDATA = []

if __name__ == '__main__':
    joy = TestbedController()
    
    serialcomm = serial.Serial(port = 'COM5',baudrate=115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,)
    serialcomm.timeout = 0.1
    while True:
        
        # cData = joy.getServoValues()
        if joy.Start:
            break
        
        if startFlag == "1":
            ProgMan = joy.getProgPars()
            
            if joy.Y: # Save log flag
                if len(ALLDATA) > 0:
                    print("SAVE DATA")
                    fname = str(time.time()) + ".txt"
                    f = open(fname,'w')
                    f.write("steer\tthrottle\tmanflag\tdsx\tdsy\tbs\taflag\tt\tGr\tAx\tAy\n")
                    for d in ALLDATA:
                        for i in d:
                            f.write(str(i))
                            if i != d[-1]: # Prevent extra end tab
                                f.write("\t")
                        f.write("\n")
                    f.close()
                    ALLDATA = []
            
            
            if joy.A: # Datalog Flag
                rData = getData(serialcomm)
                    
                if rData != None:
                    # print(rData[-1])
                    ALLDATA.append(rData[-1])
                else:
                    # print(ProgMan)
                    pass
            
            if joy.ManualDriveMode:
                if time.time() - lastTime > 1:
                    joy.printControllerData()
                    lastTime=time.time()
                    
                if joy.A and rData != None: # control loop
                    yawrate = rData[-1][-3]
                    if  yawrate != 0: # if there is some yaw rate
                        print(yawrate*2*3.14159/360)
                        if abs(yawrate*2*3.14159/360) > joy.DPadSumY*0.1: # if over the limit
                            print("CUT: \t" + str(yawrate*2*3.14159/360))
                            ProgMan[1] = ProgMan[1]*0.1
                            
                sendBytes(ProgMan,serialcomm)
            else:
                
                if joy.A != 1:
                    print(ProgMan)
                    sendBytes(ProgMan,serialcomm)
                    # rData = getData(serialcomm)
                    
                    # if rData != None:
                    #     print(rData)
                    # else:
                    #     # print(ProgMan)
                    #     pass
                else:
                    sendBytes(ProgMan,serialcomm)
                    
                    
            
            if time.time() - startTime > 20000:
                break
            # cData = userToServo()
            
            # print(cData)
            # if startTime == 0:
            #     startTime = time.time()
            # t = time.time() - startTime
            
            
            # steer = math.sin(t*math.pi*2)
            # throttle = 0
            
            # cData = joy.convertToServo(steer,throttle)
            
            # # print(cData)
            
            # if t - lastTime > intervalTime:
            #     serialcomm.write((str(cData) + '\n').encode())
            #     lastTime = t
        else:
            if joy.A == 1:
                startFlag = "1"
                startTime = time.time()
                print("started")
            pass
    
        
        
        
    serialcomm.close()
            
