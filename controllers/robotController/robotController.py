from controller import Robot
import numpy as np
import struct
import random
import time
import math

class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


class slave(Robot):


    Mode = Enumerate('MOVE SEARCH STOP')
    mode = Mode.MOVE
    timeStep = 32
    max_speed = 6.28
    motors = []
    distanceSensors = []
    #Initilise bestFit to -1
    bestFit = 10**10
    pBest = [] 
    globalBest = 10**9
    gBest = []
    cancelTime = 100
    
    fitness_average = []
    
    
    def __init__(self):
        """
        This is called when the robot is created and initialises 
        the robot and its devices.
    
        """
        super(slave,self).__init__()

        self.motors.append(self.getDevice("motor1"))
        self.motors.append(self.getDevice("motor2"))
        
        self.motors[0].setPosition(float('inf'))
        self.motors[0].setVelocity(0.0)
        
        self.motors[1].setPosition(float('inf'))
        self.motors[1].setVelocity(0.0)  
           
        self.receiver = self.getDevice('receiver')
        self.receiver.enable(self.timeStep)
        self.name = self.getName()
        
        
        #Every robot name will be set to "RobotX" so string split on a 't' will return 
        #just the robot number and this can be used for the receiver channel each time 
        #This is so each robot can receive their own co-ords and fitness for calculations
        channel = [int(i) for i in self.name.split("t") if i.isdigit()]
        self.receiver.setChannel(channel[0])      
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timeStep)        
        for dsnumber in range(0, 2):
            self.distanceSensors.append(self.getDevice('ds' + str(dsnumber)))
            self.distanceSensors[-1].enable(self.timeStep)
    
    
    def positionCalculator(self, pBest, gBest):
        """
        This functions purpose is to 
        1)calculate the inertia by adding the current
        Speed vector to the current position. 
        2)calculate the angle between the best global position (gBest)
        the best local position (pBest) and the current position (c)
        In the form gBest - c - pBest
        """
        self.velocity = self.gps.getSpeedVector()
        self.currentPos = self.gps.getValues()
        del(self.currentPos[2])
        self.projectedPos = [self.currentPos[0] + (self.velocity[0] * 10), self.currentPos[1] + (self.velocity[1] * 10)]
        
        #g_m1 is current and projected
        m1 = (self.projectedPos[1] - self.currentPos[1])/(self.projectedPos[0] - self.currentPos[0])
        #g_m2 is current and gBest
        g_m2 = (gBest[1] - self.currentPos[1])/(gBest[0] - self.currentPos[0])
        
        #p_m2 is current and pBest
        p_m2 = (pBest[1] - self.currentPos[1])/(pBest[0] - self.currentPos[0])
        
        #Angle - Tan^-1((m1-m2)/1+(m1)(m2))
        self.gCalc = (m1 - g_m2) / (1 + (m1*g_m2))
        self.pCalc = (m1 - p_m2)/ (1 + (m1*p_m2))
        self.G_angle = np.arctan(self.gCalc)
        self.P_angle = np.arctan(self.pCalc)
        
        cg = 2
        cp = 1.5
        rg = random.uniform(0,1)
        rp = random.uniform(0,1)
        #Angle of change
        if self.iteration < 50:
            self.aoc = (rg*cg*self.G_angle/2) + (cp*rp*self.P_angle/8)
        else:
            self.aoc = self.G_angle/2 + self.P_angle/5
        print("The angle of change for", self.name, "is", (self.aoc * (180/math.pi)))
       # print(self.iteration)
        self.mode = self.Mode.SEARCH
        print("Change mode = Search", self.name)
        
                

        
    def run(self):
        speed = self.max_speed
        while True:
            if self.mode == self.Mode.MOVE:
                self.motors[0].setVelocity(speed)
                self.motors[1].setVelocity(speed)
                
                delta = self.distanceSensors[0].getValue() - self.distanceSensors[1].getValue()
                if delta != 0:
                    self.motors[0].setVelocity(((speed+0.01*delta) + 0.01)/2)
                    self.motors[1].setVelocity(((speed-0.01*delta) + 0.01)/2)
            
            elif self.mode == self.Mode.SEARCH:
                velocity = self.gps.getSpeedVector()
                self.newProjected = self.gps.getValues() + (velocity*10)

                m1 =  (self.projectedPos[1] - self.currentPos[1])/(self.projectedPos[0] - self.currentPos[0])
                m2 = (self.newProjected[1] - self.currentPos[1])/(self.newProjected[0] - self.currentPos[0])
                calc = (m1 - m2) / (1 + (m1*m2))
                newProjectedAngle = round(np.arctan(calc),2)
                #print(newProjectedAngle, self.aoc/3, self.name)
                #if newProjectedAngle != self.angle:
                #print("AOC: ", self.aoc, "current angle", newProjectedAngle)
                #print(self.aoc-0.10, "<=", newProjectedAngle, "<=" ,self.aoc+0.10, self.name)
                if self.aoc-0.01 <= newProjectedAngle <= self.aoc+0.01:
                    self.mode = self.Mode.MOVE
                    #print("Change Mode Move", self.name)
                else:
                    self.motors[0].setVelocity(self.max_speed)
                    self.motors[1].setVelocity(self.max_speed/2)
                    #print(self.gCalc, calc)
                    #print("Target Angle = ",self.angle, "newProjectedAngle = ", newProjectedAngle)
                    

                
            if self.receiver.getQueueLength() > 0:
                message = self.receiver.getData().decode('utf-8')
                messages = message.split("/")
                gBest =  [float(messages[0]), float(messages[1])]
                pBest = [float(messages[2]), float(messages[3])]
                self.iteration = int(messages[4])
                self.positionCalculator(pBest, gBest)
                self.receiver.nextPacket()
                
            if self.bestFit <= 4:
                print("Object has been found at", self.pBest)
                                
            if self.step(self.timeStep) == -1:
                break
                
                
if __name__ == "__main__":
     controller = slave()
     controller.run()