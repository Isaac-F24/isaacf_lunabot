import sys
import numpy
import math
import random
import time
from Map_maker import map_generate

import tkinter

#Draw Init

root = tkinter.Tk()

canvasHeight = 700
canvasWidth = 700

canvas = tkinter.Canvas(root, bg="white", height=canvasHeight, width=canvasWidth)
canvas.pack()

robotRad = 0.05

#end Init

def translate(node,arena): #Translate from node to canvas space
    x = node[1]/len(arena[0]) * canvasWidth *.95 + (canvasWidth * 0.025)
    y = node[0]/len(arena) * canvasHeight * .95 + (canvasHeight * 0.025)

    return (x,y)


def circle(color,xpos,ypos,radius): #Draw a circle on canvas

    x = xpos*canvasWidth
    y = ypos*canvasHeight
    horizontalRad = radius * canvasWidth
    verticalRad = radius * canvasHeight

    canvas.create_oval(x-horizontalRad, y+verticalRad, x+horizontalRad, y-verticalRad, fill=color, outline="")


def drawDir(node,arena,angle,vel,itera):

    if (itera == 1):
        color = "red"
    elif (itera == 2 ):
        color = "orange"
    elif (itera == 3):
        color = "green"
    elif (itera == 4):
        color = "blue"
    elif (itera == 5):
        color = "magenta"
    elif (itera == 6):
        color = "cyan"
    elif (itera == -1):
        color = "gold"

    length = vel * 1.4

    xytuple = translate(node,arena)
    x = xytuple[0]
    y = xytuple[1]

    x2 = x + math.cos((angle)) * length
    y2 = y + math.sin((angle)) * length

    canvas.create_line(x,y,x2,y2, fill = color, width = 4)
    

def update_goal(current_map, goal): #however map is be updated
    
    doneEditing = True

    newGoal = goal.copy()

    def click_to_update(event):
        if (not doneEditing):
            xpos=event.x
            ypos=event.y

            nonlocal newGoal
            prevGoal = newGoal

            x = int((xpos / canvasWidth * 1.05) * len(current_map[0]))
            y = int((ypos / canvasHeight * 1.05) * len(current_map))


            newGoal = [y,x]
            
            circle("white",current_map, prevGoal, 10)

            circle("green",current_map, newGoal, 10)

    canvas.bind("<Button-1>", click_to_update)

        
    doneEditing = False
    input("Editing: Press Enter to continue")
    doneEditing = True

    return newGoal

def drawstart(rocks,start,goal):

    for rock in rocks:
        xpos = rock[0]
        ypos = rock[1]
        rad = rock[2]

        circle("gray", xpos, ypos, rad)

    circle("blue", start[1], start[0], robotRad)
    circle("green", goal[1], goal[0], robotRad)

class MPC:

    timeStepTime = 0.2 #Time per timestep (in seconds?)

    initStandardDeviation = 100.0

    minVelocity = -5 #Units per second (0.025 m = 1 unit)
    maxVelocity = 5

    minAngularVelocity = -3.49066 #Theta per second (rad)
    maxAngularVelocity = 3.49066

    #Arena- 200 x 272 units 5 x 6.8 m

    #Correct arena size: height 200 width 272

    def __init__(self, timesteps, sampleCount, bestSamples, iterations):
        self.timesteps = timesteps #Number of timesteps per sample
        self.sampleCount = sampleCount #Number of samples to create
        self.bestSamples = bestSamples #Number of best samples to take the mean/SD of
        self.iterations = iterations #Number of iterations that the MPC runs for

        self.itera = 1 #Temp variable used for drawing

        self.means = numpy.full((self.timesteps,2), 0.0) #Initial Means (All 0); Column 0: Velocity; Column 1: AngularV
        self.stdDevs = numpy.full((self.timesteps, 2), self.initStandardDeviation) #Init StdDevs (All set to value) Column 0: Vel; Column 1: AngV

    '''
    Generates random samples based on the current means and standard deviation values (normal dist)
    and clamps them to the possible maximum values 
    '''
    def generateSamples(self):
        
        samples = numpy.empty((self.timesteps, 2, self.sampleCount)) #3d matrix: Each row: timestep, left/right column: velocity/angular velocity, each layer is a sample
 
        for i in range(self.timesteps):
            #For each timestep, a velocity and angularVelocity is generated * all samples based on the means/stdDevs of that timestep

            # Velocities 
            sampleRow = numpy.random.normal(self.means[i][0], self.stdDevs[i][0], self.sampleCount) #Pick mean from means:current row and left column; stdDevs from stdDevs, length of Samplecount
            sampleRow = numpy.clip(sampleRow, self.minVelocity, self.maxVelocity) #Clamp to min and max values
            samples[i][0] = sampleRow

            #Angular Velocities
            sampleRow = numpy.random.normal(self.means[i][1], self.stdDevs[i][1], self.sampleCount) 
            sampleRow = numpy.clip(sampleRow, self.minAngularVelocity, self.maxAngularVelocity)
            samples[i][1] = sampleRow

        return samples

    '''
    Uses a Basic motion model to generate a position matrix for each sample (pos, angle)
    '''
    def generateState(self, samples, current_state, arena):
        states = numpy.empty((self.timesteps, 3, self.sampleCount)) #3d matrix: Each row: timestep, columns: x,y, theta, each layer is a sample
        
        #Basic Motion Model here (replace)

        for i in range(self.sampleCount): #For each sample: 
            tempState = current_state.copy() #Reset to Shallow copy of robot state (x,y,theta)
            for j in range(self.timesteps): #Go down each timestep
                linearV = samples[j][0][i]
                angularV = samples[j][1][i]

                newAngle = tempState[2] + angularV * self.timeStepTime #New angle = current + movement
                if (newAngle < 0):
                    newAngle = 6.28318530718 + newAngle
                if (newAngle >= 6.28318530718):
                    newAngle -= 6.28318530718 

                tempState[2] = newAngle
                states[j][2][i] = tempState[2] #set theta of the current timestep/current sample to new angle

                #DETAIL on/off drawing

                #drawDir([tempState[1],tempState[0]], arena, newAngle, linearV, self.itera)

                xMovement = linearV * self.timeStepTime * math.cos((newAngle)) #X movement = linear mvmt * cos(angle)
                yMovement = linearV * self.timeStepTime * math.sin((newAngle))

                tempState[0] += xMovement
                tempState[1] += yMovement #increase state

                states[j][0][i] = tempState[0] #Set x of the current timestep/current sample to new state
                states[j][1][i] = tempState[1]


        return states

    '''
    Uses positions (states) to calculate the cost of each generated sample based on cost function
    '''
    def calculateCosts(self, states, goal, arena): #Cost Function

        reduction = 0.9 #Used to reduce the cost of each sequential timestep

        costs = [] #List of costs per sample

        for i in range(self.sampleCount): #for each sample
            cost = 0
            for j in range(self.timesteps): #for each timestep
                distance = math.sqrt((states[j][1][i] - goal[0])**2 + (states[j][0][i] - goal[1])**2) #calculate the distance to goal
                distance *= reduction**j #apply reduction

                cost += distance #add it to the total cost for this sample

                yval = int(states[j][1][i])
                xval = int(states[j][0][i])

                if (yval < 0 or yval > len(arena) - 1 or xval < 0 or xval > len(arena[0]) - 1): #Check for out of bounds
                    cost = sys.maxsize
                    break

                if (arena[yval][xval] == 1 or arena[yval][xval] == 2): #Check for obstacle
                    cost = sys.maxsize
                    break

            costs.append((cost, i)) #add this cost to the list along with sample number

        return costs


    '''
    Uses the samples and their associated costs to calculate the best samples and take their means /stdDevs, and set the values to them
    '''
    def findNewMeans(self, costs, samples):
        costs.sort()

        bestCosts = costs[0:self.bestSamples]

        bestSamplesList = []

        for tuple in bestCosts: #For each sample in the best samples
            sampleNum = tuple[1] 
            bestSamplesList.append(samples[:,:,sampleNum]) #add the 2d sample array to the list of best samples

        for i in range(self.timesteps): #For each timestep row
            velocityList = []
            angularVList = []
            
            for j in range(self.bestSamples): #Get a list of the values from all the best samples for both columns
                velocityList.append(bestSamplesList[j][i][0]) 
                angularVList.append(bestSamplesList[j][i][1])

            #Fill in the means and stdDevs for this timestep row
            self.means[i][0] = numpy.mean(velocityList, dtype=float) 
            self.stdDevs[i][0] = numpy.std(velocityList)

            self.means[i][1] = numpy.mean(angularVList, dtype=float)
            self.stdDevs[i][1] = numpy.std(angularVList)

        # print(self.means)
        

    '''
    Main control loop- initializes, then generates new samples, finds the means/stdDevs to use, "moves" the robot, and repeats until the goal is reached
    '''
    def control(self):


        drawstart(self.arena, [self.robotState[1],self.robotState[0]], self.goalValue)
        drawDir([self.robotState[1],self.robotState[0]], self.arena, self.robotState[2], 10, -1)

        while (True):
            for i in range(self.iterations):

                samples = self.generateSamples()

                states = self.generateState(samples)
                costs = self.calculateCosts(states)

                self.findNewMeans(costs,samples)

                if (self.itera < 6):
                    self.itera+= 1
                else:
                    self.itera = 1


            self.itera = 1

            velocity = self.means[0][0]
            angularVelocity = self.means[0][1]


            #"Move robot"
            newAngle = self.robotState[2] + angularVelocity * self.timeStepTime #New angle = current + movement
            if (newAngle < 0):
                newAngle = 6.28318530718 + newAngle
            if (newAngle >= 6.28318530718):
                newAngle -= 6.28318530718 

            drawDir([(self.robotState[1]),(self.robotState[0])], self.arena, newAngle, velocity * 5, -1)

            
            print("Moving at", velocity, "velocity and", newAngle, "angle." )
            s = input("Press enter or update")

            if (s == "ug" or s == "update" or s == "update goal"):
                self.goalValue = update_goal(self.arena,self.goalValue)
                print ("done")

            circle("white",self.arena,[(self.robotState[1]),(self.robotState[0])], 10)

            self.robotState[2] = newAngle

            xMovement = velocity * self.timeStepTime * math.cos((newAngle)) #X movement = linear mvmt * cos(angle)
            yMovement = velocity * self.timeStepTime * math.sin((newAngle))

            self.robotState[0] += xMovement
            self.robotState[1] += yMovement #increase state
            # End move robot

            circle("blue",self.arena,[(self.robotState[1]),(self.robotState[0])],10)

            #Reset after iterations
            self.means = numpy.full((self.timesteps,2), 0.0) 
            self.stdDevs = numpy.full((self.timesteps, 2), self.initStandardDeviation) 

            

    


mpc = MPC(timesteps=4,sampleCount=500,bestSamples=20,iterations=5)
