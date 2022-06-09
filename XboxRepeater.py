# -*- coding: utf-8 -*-
"""
Created on Wed Jun  8 19:16:12 2022

@author: Andrew Ng
"""

from inputs import get_gamepad
import math
import threading

import serial
import time

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

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def getServoValues(self): # return the buttons/triggers that you care about in this methode
        x = round(90 - self.LeftJoystickX*self.MaxSteering,3)
        z = round(90 + self.RightTrigger*self.MaxThrottle - self.LeftTrigger*self.MaxThrottle,3)
        
        return [x,z]


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
                # elif event.code == 'BTN_TL':
                #     self.LeftBumper = event.state
                # elif event.code == 'BTN_TR':
                #     self.RightBumper = event.state
                # elif event.code == 'BTN_SOUTH':
                #     self.A = event.state
                # elif event.code == 'BTN_NORTH':
                #     self.X = event.state
                # elif event.code == 'BTN_WEST':
                #     self.Y = event.state
                # elif event.code == 'BTN_EAST':
                #     self.B = event.state
                # elif event.code == 'BTN_THUMBL':
                #     self.LeftThumb = event.state
                # elif event.code == 'BTN_THUMBR':
                #     self.RightThumb = event.state
                # elif event.code == 'BTN_SELECT':
                #     self.Back = event.state
                # elif event.code == 'BTN_START':
                #     self.Start = event.state
                # elif event.code == 'BTN_TRIGGER_HAPPY1':
                #     self.LeftDPad = event.state
                # elif event.code == 'BTN_TRIGGER_HAPPY2':
                #     self.RightDPad = event.state
                # elif event.code == 'BTN_TRIGGER_HAPPY3':
                #     self.UpDPad = event.state
                # elif event.code == 'BTN_TRIGGER_HAPPY4':
                #     self.DownDPad = event.state



# waitTime = 0
# lastTime = time.time()

# if shit is laggy, close the whole kernal, interrupt doesn't help

if __name__ == '__main__':
    joy = TestbedController()
    
    serialcomm = serial.Serial('COM4',115200)
    serialcomm.timeout = 0.00
    while True:
        cData = joy.getServoValues()
        serialcomm.write((str(cData) + '\n').encode())
    
        print(cData[1])
        
        try:
            xx = serialcomm.readline()
        except:
            print("ok")
            pass
    serialcomm.close()
            