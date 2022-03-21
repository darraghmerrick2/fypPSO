from controller import Supervisor, GPS
import sys
import numpy as np
import struct
import time
from math import dist
import threading
import matplotlib.pyplot as plt
import random


class Driver(Supervisor):

    Robots = []
    pBestFitness = []
    pBestPos = []
    #Have gBest obscenely high so that in the first check it will always be lower
    gBest = -1
    gBestPos = []
    translation_fields = []
    translation_values = []
    fitness_list = []
    timeStep = 128
    Flag = False
    iteration = 0
    fitness_average = []


    def randomisePositions(self, robots):
        """
        This method randomises the positions of all robots within the arena area.
        This is to ensure fair testing of the algorithm in place
        """
        
        x = random.randint(-1000, 1000)
        z = random.randint(-25,25)
        self.translationField.setSFVec3f(x, z,0)

       
    def __init__(self):
        super(Driver, self).__init__()
        print("Initilisation of Supervisor")
        for i in range(0, 6):
            self.Robots.append(self.getFromDef("ROBOT" + str(i)))
            self.translation_fields.append(self.Robots[i].getField('translation'))
            x = random.randint(-2000, 2000)
            z = random.randint(-2000,2000)
            self.translation_values.append(self.translation_fields[i].setSFVec3f([x,z,0]))
        
        self.target = self.getFromDef("Target")
        self.target_field = self.target.getField('translation')
        a = random.randint(-2000, 2000)
        b = random.randint(-2000,2000)
        self.target_field.setSFVec3f([a,b,0])
        self.target_co_ords = self.target_field.getSFVec3f()

    
        self.emitter = self.getDevice("emitter")
        
        #Set the initial fitness values to 100
        for i in range(len(self.Robots)):
            self.fitness_list.append(100)
        for i in range(len(self.Robots)):
            self.pBestFitness.append(1000)    
            self.pBestPos.append([0,0])
        
    def delay(self, arg):
        print("delay")
        time.sleep(arg)
        
    def sendInfo(self, bestPos):
        """
        Function is used to send location and fitness information     
        about the robots and the fittest robot to all robots in the simulation
        The global best needs to be send to all robots and the pBest for each individual robot
        needs to be sent to the corrosponding robots
        Loop through channels 
        """
        gBestString = str(self.gBestPos[0]) + "/"+ str(self.gBestPos[1])
        
        #Also send best coordinates so that all robots can share the same global best
        #print("bestFitness: ", bestFit)
        for i in range(len(self.Robots)):
            pBestString = str(self.pBestPos[i][0]) + "/" + str(self.pBestPos[i][1])
            self.emitter.setChannel(i)
            message = gBestString + "/" + pBestString + "/" + str(self.iteration)
            self.emitter.send(message.encode('utf-8'))
            
        
    def checkGlobalBest(self, position, fitness):
        if self.iteration == 0:
            self.gBest = fitness
            self.gBestPos = position
        elif fitness < self.gBest:
            self.gBest = fitness
            self.gBestPos = position
            print("new global best has been found at", position, "with a fitness of", fitness)
        else:
            pass
    
    def graphAverage(self, average):
        plt.title("Average Fitness")
        plt.plot(averageFit, color="red")
        plt.show()
        
        
    def getFitness(self, values):
        #Get Fitness of each robot in the 'Robot[]' array
        #Fitness is the squared distance betweent he robot and the target
        #lower is better  
        #Need to work with positions [0] and [1] to get distances from the target
        
        #Euclid distance is [(x2 - x1)^2 + (y2 - y1)]^(1/2)
        #(x1,x2) - target 
        #(y1,y2) - loops each time to get each robots fitness
        #print("Calculating Fitness")
        x1 = self.target_co_ords[0]
        y1 = self.target_co_ords[1]
        distance = []
        for i in range(len(self.Robots)):
            #run calculation each time in this loop and append to fitness list
            x2 = values[i][0]
            y2 = values[i][1]
            fitness = round(dist(self.target_co_ords[0:2], values[i][0:2]), 4)
            self.fitness_list[i] = fitness**2
            distance.append(fitness)
            
            del(values[i][2])
        bestPos = self.fitness_list.index(min(self.fitness_list))
        print(len(self.fitness_list))
        for j in range(len(self.pBestFitness)):
            if self.iteration == 0:
                self.pBestFitness[j] = self.fitness_list[j]
                self.pBestPos[j] = values[j]
                self.checkGlobalBest(self.pBestPos[j], self.pBestFitness[j])
            elif self.fitness_list[j] <= self.pBestFitness[j]:
                self.pBestFitness[j] = self.fitness_list[j]
                self.pBestPos[j] = values[j]
                #print("new local best found at ", self.pBestPos[j], j)
                self.checkGlobalBest(self.pBestPos[j], self.pBestFitness[j])
            else:
                pass
        self.fitness_average.append(fitness)
        #Send fitness to corrosponding robots
        self.sendInfo(bestPos)
        #self.check_localFitness(self.fitness_list) 
        pass
        
        
    
    def run(self):
        #x = random.randint(-25, 25)
        #z = random.randint(-25,25)
        #self.translationField.setSFVec3f(x, z,0)
        t = 0
        previous_message = ""
        while True:
            
            for i in range(len(self.Robots)):
                #overwrite values each time
                self.translation_fields[i] = self.Robots[i].getField('translation')
                self.translation_values[i] = self.translation_fields[i].getSFVec3f()
            message = self.translation_values    
            
            if self.iteration < 50:
                if t%600 == 0:
            #May just call function here and have it return nothing
            #Find values and send information all in one go
                    self.getFitness(message)
                    self.iteration += 1
                    print("Best fitness was", self.gBest, "for iteration", self.iteration)

                #print("Iteration: ", self.iteration)
            elif 50 < self.iteration  < 150:
                if t%300 == 0 :
                    self.getFitness(message)
                    self.iteration += 1
                    print("Best fitness was", self.gBest, "for iteration", self.iteration)

            else:
                if t%150 == 0 :
                    self.getFitness(message)
                    self.iteration += 1
                    print("Best fitness was", self.gBest, "for iteration", self.iteration)
            t += 1
            
            
            if self.step(self.timeStep) == -1:
                self.graphAverage(fitness_average)

                break
                
    
controller = Driver()
controller.run()