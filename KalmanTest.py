# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 15:22:41 2022

@author: Andrew Ng
"""

import numpy
import control
from numpy import dot
from numpy import transpose
from numpy.linalg import inv
import csv
import matplotlib.pyplot as plt

# numpy.dot(x,numpy.linalg.inv(y))


# Make a kalman filter which uses pre-determined gains and schedules them for appropriate speed - Scheduled Kalman Filter, SKF

class SKF:
    
    def __init__(self, speedList):
        self.speedList = speedList
        self.dt = 0.005
        self.SC = -0.08
        
        self.cf = 180.0
        self.cr = 180.0- 30
        
        self.m = 45.92/9.81
        self.I = 0.093
        
        self.a = 0.164
        self.b = 0.147
        
        
        self.state = numpy.array([[0],[0]])
        self.preState = self.state
        self.stateHistory = []
        self.betaHistory = []
        self.d = []
        
        self.lastu = 0
        
        self.covarShape = numpy.array([[1,0],[0,1]])
        

        self.Q = 1*self.covarShape
        self.R = 0.1*self.covarShape
        
        self.A = {} 
        self.B = {}
        self.C = {}
        self.D = {}
        self.Adt = {}
        self.Bdt = {}
        self.Cdt = {}
        self.Ddt = {}
        self.K = {}
        self.ss = {}
        self.ss_dt = {}
        
        self.beta = 0
        
        # populate system matrices, both continuous and discrete
        for speedControl in speedList:
            u = round(speedControl*1.4258/0.12,2)
            
            self.A[u] = numpy.array([[-1*(self.cf+self.cr)/(self.m*u), -1*u - (self.a*self.cf-self.b*self.cr)/(self.m*u)],
                                     [-1*(self.a*self.cf-self.b*self.cr)/(self.I*u), -1*((self.a**2)*self.cf + (self.b**2)*self.cr)/(self.I*u)]])
            
            self.B[u] = numpy.array([[self.cf/self.m],
                                     [self.a*self.cf/self.I]])
             
            self.C[u] = numpy.array([[1,0],[0,1]])
             
            self.D[u] = numpy.array([[0],[0]])
            
            self.ss[u] = control.StateSpace(self.A[u],self.B[u],self.C[u],self.D[u])
            
            self.ss_dt[u] = control.c2d(self.ss[u], self.dt)
            
            self.Adt[u] = self.ss_dt[u].A
            self.Bdt[u] = self.ss_dt[u].B
            self.Cdt[u] = self.ss_dt[u].C
            self.Ddt[u] = self.ss_dt[u].D
            
            # iterate to get Kalman gain
            # set initial conditions
            self.X_k0 = numpy.array([[0],[0]])
            self.X_k1 = numpy.array([[0],[0]])
            self.X_kp1 = numpy.array([[0],[0]])
            
            P_k0 = 0.5*self.covarShape
            for i in range(50):
                
                P_kp = dot(self.Adt[u], dot(P_k0, transpose(self.Adt[u]))) + self.Q
                
                
                K = dot(dot(P_kp, transpose(self.Cdt[u])), inv(dot(self.Cdt[u], dot(P_kp, transpose(self.Cdt[u]))) + self.R))
                
                
                P_k0 = dot(numpy.eye(2) - dot(K, self.Cdt[u]), P_kp)
                
                
                
            self.K[u] = K
            
    def resetSKF(self):
        self.X_k0 = numpy.array([[0],[0]])
        self.X_k1 = numpy.array([[0],[0]])
        self.X_kp1 = numpy.array([[0],[0]])
        
        
    def predictNextState(self, speedControl, steer, measurement, onTime): # measurement is [r, ay]
        # s = (-0.311/1.08)*(steer + self.SC)
        dVar = lambda j:None
        dVar.speedControl = speedControl
        dVar.steer = steer
        dVar.t = onTime
        dVar.r = measurement[0]
        dVar.Ay = measurement[1]
        
        self.d.append(dVar)
        
        
        
        s = (-0.311/1.08)*(steer + self.SC)
        if self.lastu != 0:
            midSpeed = (speedControl + self.lastu)/2
            self.lastu = speedControl
        else:
            midSpeed = speedControl
            
        if midSpeed == 0:
            midSpeed = 0.12
        u = round(midSpeed*1.4258/0.12,2)
        
        if u == 0:
            self.X_k1 = self.X_k1*1
            self.beta = self.beta*1
            self.stateHistory.append(self.X_k1)
            self.betaHistory.append(self.beta)
            return self.X_k1
        
        v_measured = (measurement[1] - measurement[0]*u - self.A[u][0][1]*measurement[0] - self.B[u][0][0]*s)/self.A[u][0][0]
        
        self.X_kp1 = dot(self.Adt[u], self.X_k0) + self.Bdt[u]*s
        
        self.X_k1 = self.X_kp1 + dot(self.K[u], numpy.array([[v_measured],[measurement[0]]]) - dot(self.Cdt[u], self.X_kp1))
        
        
        
        self.X_k0 = self.X_k1
        
        self.beta = self.X_k1[0]/u
        
        self.stateHistory.append(self.X_k1)
        self.betaHistory.append(self.beta)
        
        return self.X_k1
    
    def simFile(self,fname): # simulate predicting through file timeseries
        f = open(fname,'r')
        cf = csv.reader(f, delimiter = '\t')
        d = [] # [speed, steer, r, ay]
        for i in cf:
            dVar = lambda j:None
            dVar.speedControl = round(float(i[1]),2)
            dVar.steer = float(i[0])
            dVar.t = float(i[7])
            dVar.r = float(i[8])*2*numpy.pi/360.0
            dVar.Ay = -1*float(i[10])
            dVar.Ax = float(i[10])
            
            d.append(dVar)
        f.close()
        self.d = d
        
        stateHistory = []
        for i in d:
            hh = self.predictNextState(i.speedControl,i.steer,[i.r, i.Ay])
            stateHistory.append([hh[0][0], hh[1][0], self.beta])
            
        return [d,stateHistory]
    
    def showResults(self):
        t = []
        Ay = []
        r = []
        k_v = []
        k_r = []
        for i in self.d:
            t.append(i.t)
            Ay.append(i.Ay)
            r.append(i.r)
            
        for state in self.stateHistory:
            k_v.append(state[0][0])
            k_r.append(state[1][0])
            

            
        plt.plot(t,Ay,'rx', label = "M_ay")
        plt.plot(t,r,'bx', label = "M_r")
        plt.plot(t,k_r, label = "K_r")
        plt.plot(t, k_v, label = "K_v")
        plt.plot(t, self.betaHistory, label = "K_b")
        plt.grid(True)
        plt.legend(loc = "upper left")
                
        
       

        
    # def newMeasurement(self, y, U, speed):
        
def makeSpeedRange(start,stop,step):
    r = []
    s = (stop-start)//step + 2
    for i in range(int(s)):
        r.append(round(start+step*i,2))
        
    return r

#s = round(1.4258*(6/6), 2)
#speeds = makeSpeedRange(0.01,1,0.01)
#
#testBed = SKF(speeds)
#h = testBed.simFile("circle_speedup_fixed.txt")
#vs = []
#rs = []
#bs = []
#for i in h[1]:
#    vs.append(i[0])
#    rs.append(i[1])
#    bs.append(i[2])
#    
#testBed.showResults()


    
    