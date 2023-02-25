import sys
import numpy
import math
import random
import time
from Map_maker import map_generate
from Collison_avoid import expand_grid

class MPC:

    initStandardDeviation = 100.0

    minVelocity = -5 #Units per second (0.05 m = 1 unit)
    maxVelocity = 5

    minAngularVelocity = -1 #Theta per second (Rad)
    maxAngularVelocity = 1


    def __init__(self, timesteps, sampleCount, bestSamples, iterations, x_offset, y_offset, res=0.05, timePerTimestep=0.2):
        self.timesteps = timesteps #Number of timesteps per sample
        self.sampleCount = sampleCount #Number of samples to create
        self.bestSamples = bestSamples #Number of best samples to take the mean/SD of
        self.iterations = iterations #Number of iterations that the MPC runs for
        self.timeStepTime = timePerTimestep #time taken per timestep

        self.means = numpy.full((self.timesteps,2), 0.0) #Initial Means (All 0); Column 0: Velocity; Column 1: AngularV
        self.stdDevs = numpy.full((self.timesteps, 2), self.initStandardDeviation) #Init StdDevs (All set to value) Column 0: Vel; Column 1: AngV

        self.path = [] #List of coordinates (path given by dstar)
        self.pathIndex = 0 #Which point on the path is the goal

        self.currentState = [] #x,y,theta (robot coords, rad)

        self.arena = []

        self.res = res
        self.x_offset = x_offset
        self.y_offset = y_offset



    def updatePath(self, path):
        self.path = path
        self.pathIndex = 0 #reset path index when new grid

    def updateState(self, state):
        self.currentState = state
    
    def updateGrid(self, grid, r=5):
        self.arena = expand_grid(grid, r)
    

    def robotCoordsToGrid(self, pos):
        shifted_pos = [pos[0] - self.x_offset, pos[1] - self.y_offset]
        coord = [int(shifted_pos[1] / self.res + 0.5), int(shifted_pos[0] / self.res + 0.5)]
        return coord

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
    def generateState(self, samples, current_state):
        states = numpy.empty((self.timesteps, 3, self.sampleCount)) #3d matrix: Each row: timestep, columns: x,y, theta, each layer is a sample
        
        #Basic Motion Model here (replace)

        for i in range(self.sampleCount): #For each sample: 
            tempState = current_state.copy() #Reset to Shallow copy of robot state (x,y,theta)
            for j in range(self.timesteps): #Go down each timestep
                linearV = samples[j][0][i]
                angularV = samples[j][1][i]

                if (angularV == 0):
                    states[j][2][i] = tempState[2] #set theta of the current timestep/current sample to same angle

                    xMovement = linearV * self.timeStepTime * numpy.cos(tempState[2]) #X movement = linear mvmt * cos(angle)
                    yMovement = linearV * self.timeStepTime * numpy.sin(tempState[2])

                    tempState[0] += xMovement
                    tempState[1] += yMovement #increase state

                    states[j][0][i] = tempState[0] #Set x of the current timestep/current sample to new state
                    states[j][1][i] = tempState[1]
                else:
                    #Differential drive model (screenshot from discord)
                    changeX = -1*((linearV/angularV) * numpy.sin(tempState[2])) + ((linearV/angularV) * (numpy.sin(tempState[2] + angularV * self.timeStepTime)))
                    changeY = ((linearV/angularV) * numpy.cos(tempState[2])) - ((linearV/angularV) * (numpy.cos(tempState[2] + angularV * self.timeStepTime)))
                    changeTheta = angularV * self.timeStepTime

                    tempState[0] += changeX
                    tempState[1] += changeY
                    tempState[2] += changeTheta

                    states[j][0][i] = tempState[0] #Set x of the current timestep/current sample to new state
                    states[j][1][i] = tempState[1]
                    states[j][2][i] = tempState[2]


        return states

    '''
    Uses positions (states) to calculate the cost of each generated sample based on cost function
    '''
    def calculateCosts(self, states, arena): #Cost Function

        reduction = 0.9 #Used to reduce the cost of each sequential timestep

        costs = [] #List of costs per sample

        for i in range(self.sampleCount): #for each sample
            cost = 0
            for j in range(self.timesteps): #for each timestep

                goal = self.path[self.pathIndex]

                distance = math.sqrt((states[j][1][i] - goal[0])**2 + (states[j][0][i] - goal[1])**2) #calculate the distance to goal
                distance *= reduction**j #apply reduction

                cost += distance #add it to the total cost for this sample

                coords = self.robotCoordsToGrid([states[j][0][i], states[j][1][i]])

                if (coords[0] < 0 or coords[0] > len(arena) - 1 or coords[1] < 0 or coords[1] > len(arena[0]) - 1): #Check for out of bounds
                    cost = sys.maxsize
                    break

                if (arena[coords[0]][coords[1]] == 1 or arena[coords[0]][coords[1]] == 2): #Check for obstacle
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
        
        return bestCosts[0][1]

        

    '''
    Main control- generates new samples, finds the means/stdDevs to use
    '''
    def control(self):

        distanceToPoint = 0.0 #how close the robot must be to a path

        currentState = self.currentState

        arenaGrid = self.arena

        velocity = 0
        angularVelocity = 0

        if ((distance(currentState, self.path[self.pathIndex]) <= distanceToPoint) and len(self.path) > self.pathIndex + 1):
            self.pathIndex += 1

        for i in range(self.iterations):

            samples = self.generateSamples()

            states = self.generateState(samples, currentState)
            costs = self.calculateCosts(states, arenaGrid)

            best_idx = self.findNewMeans(costs,samples)

            velocity = samples[0][0][best_idx]
            angularVelocity = samples[0][1][best_idx]

        #TODO- change from mean to best sample

        #Reset after iterations
        self.means = numpy.full((self.timesteps,2), 0.0) 
        self.stdDevs = numpy.full((self.timesteps, 2), self.initStandardDeviation) 

        return velocity, angularVelocity

            

def distance(point1, point2):
    return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)


#mpc = MPC(timesteps=4,sampleCount=500,bestSamples=20,iterations=5)
