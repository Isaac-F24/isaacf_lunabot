import draw_dstar
import draw_mpc
from Map_maker import *
from Collison_avoid import expand_grid

import random
import math
import numpy


"""
Takes in an accurate map, and the current robot knowledge, and updates what the robot should be able to see 
based on the vision paramater. It then expands the grid to include a certain radius outside obstacles based on the given
radius of the robot (given in % of the map).
"""
def updateVisibleGrid(real_map, current_map, position, goal): #What the robot can see
    vision = 4 #Range of vision for rocks (grid units)

    robotGridRad = draw_mpc.robotRad * max(len(real_map),len(real_map[0])) #Rad of robot, make sure it is smaller than vision

    leftbound = position[1] - min(vision, position[1])
    rightbound = position[1] + min(vision, (len(real_map[0]) - position[1] - 1))
    topbound = position[0] - min(vision, position[0])
    bottombound = position[0] + min(vision, (len(real_map) - position[0] - 1))

    for i in range(topbound, bottombound+1):
        for j in range(leftbound, rightbound+1):
            if (real_map[i][j] == 1 or real_map[i][j] == 2):
                current_map[i][j] = 1


    current_map = expand_grid(current_map, robotGridRad)

    current_map[position[0]][position[1]] = 0
    current_map[goal[0]][goal[1]] = 0

    return current_map

'''
Main loop for mpc and dstar. Creates a random map/start/end, initializes both dstar/mpc
Then loops until the destination is reached: generate a path list based on the Dstar values, give the MPC the path list, set the goal to lookAhead steps in the path
Generate samples and resulting velocity/angular velocity mean based on the goal, then 'move the robot' with a timestep
Update the movement for the dstar information and check to see if the map has changed. If so, update the affected nodes and all their surroundings, 
then mark that the path needs to be replanned.
'''
def combo(lookAhead):
    #Set up map
    arenaHeight = 40
    arenaWidth = 40

    rockList = rocks(8)
    arena = rocksToMap(rockList, arenaWidth, arenaHeight)

    dStarArena = map_generate(arenaWidth,arenaHeight,0) #DStar knowledge starts blank

    #Start and end
    start_node = [random.randint(0,arenaHeight-1), random.randint(0,arenaWidth-1)]
    goal_node = [random.randint(0,arenaHeight-1), random.randint(0,arenaWidth-1)]

    percentStart = [(start_node[0] / arenaHeight), (start_node[1]) / arenaWidth] #For MPC
    percentGoal = [(goal_node[0] / arenaHeight), (goal_node[1]) / arenaWidth]

    robotState = [start_node[1],start_node[0],0.0] #x,y,theta (y goes up to down (rows))

    #MPC paramaters
    mpc = draw_mpc.MPC(timesteps=5,sampleCount=1000,bestSamples=100,iterations=10)

    dStarArena = updateVisibleGrid(arena, dStarArena, start_node, goal_node) #Initial range around robot

    #Base Case Check
    if ((start_node[0] == goal_node[0] and start_node[1] == goal_node[1]) or 
        arena[start_node[0]][start_node[1]] == 1 or arena[goal_node[0]][goal_node[1]] == 1):
        print("Error: no path")
        exit()

    #Init Drawings
    draw_mpc.drawstart(rockList, percentStart, percentGoal)
    draw_mpc.drawDir([robotState[1],robotState[0]], arena, robotState[2], 10, -1)
    draw_dstar.drawstart(dStarArena, start_node, goal_node)


    #Init DSTAR
    prev_node = start_node
    current_node = start_node

    init_values = draw_dstar.initialize(dStarArena,goal_node,start_node)
    node_queue = init_values[0]
    km = init_values[1]
    node_value_list = init_values[2]

    draw_dstar.find_path(start_node, node_queue, node_value_list, km, dStarArena, goal_node, True)

    node_list = [] #Drawing block
    prev_node_list = [] #Drawing block

    needs_replan = True #If the path needs to be found again
    #END Init DSTAR

    while (current_node[0] != goal_node[0] or current_node[1] != goal_node[1]): #Until reached destination

        #DSTAR 
        #This block finds and draws path
        if (needs_replan):
            prev_node_list.clear()
            for node in node_list:
                prev_node_list.append(node.copy())
            node_list.clear()

            path_node = current_node.copy()
            while (path_node[0] != goal_node[0] or path_node[1] != goal_node[1]):
                gvals = [] #find smallest g value (closest to goal)
                if (path_node[0] > 0): #above
                    if (dStarArena[path_node[0]-1][path_node[1]] == 0):
                        gvals.append((node_value_list[path_node[0]-1][path_node[1]][0] + 1, [path_node[0]-1, path_node[1]])) 
                if (path_node[0] < len(node_value_list)-1): #below
                    if (dStarArena[path_node[0]+1][path_node[1]] == 0):
                        gvals.append((node_value_list[path_node[0]+1][path_node[1]][0] + 1, [path_node[0]+1, path_node[1]]))
                if (path_node[1] > 0): #left
                    if (dStarArena[path_node[0]][path_node[1]-1] == 0):
                        gvals.append((node_value_list[path_node[0]][path_node[1]-1][0] + 1, [path_node[0], path_node[1]-1]))
                if (path_node[1] < len(node_value_list[0])-1): #right
                    if (dStarArena[path_node[0]][path_node[1]+1] == 0):
                        gvals.append((node_value_list[path_node[0]][path_node[1]+1][0] + 1, [path_node[0], path_node[1]+1]))
                if (path_node[0] > 0 and path_node[1] > 0): #Topleft
                    if (dStarArena[path_node[0]-1][path_node[1]-1] == 0):
                        gvals.append((node_value_list[path_node[0]-1][path_node[1]-1][0] + 1.4, [path_node[0]-1, path_node[1]-1]))
                if (path_node[0] < (len(node_value_list)-1) and path_node[1] > 0): #Bottomleft
                    if (dStarArena[path_node[0]+1][path_node[1]-1] == 0):
                        gvals.append((node_value_list[path_node[0]+1][path_node[1]-1][0] + 1.4, [path_node[0]+1, path_node[1]-1]))
                if (path_node[0] < (len(node_value_list)-1) and path_node[1] < len(node_value_list[0])-1): #Bottomright
                    if (dStarArena[path_node[0]+1][path_node[1]+1] == 0):
                        gvals.append((node_value_list[path_node[0]+1][path_node[1]+1][0] + 1.4, [path_node[0]+1, path_node[1]+1]))
                if (path_node[0] > 0 and path_node[1] < len(node_value_list[0])-1): #Topright
                    if (dStarArena[path_node[0]-1][path_node[1]+1] == 0):
                        gvals.append((node_value_list[path_node[0]-1][path_node[1]+1][0] + 1.4, [path_node[0]-1, path_node[1]+1]))
            
                if (len(gvals) == 0): #Nowhere to go
                    print("Error: No more path")
                    input()
                    return

                min_val = min(gvals)
                path_node = min_val[1]

                if (path_node in node_list): #Doubling back- no more path
                    print("Error: Double back")
                    input()
                    return

                node_list.append(path_node)
                gvals.clear()

            path_list = node_list.copy()

            # #Drawing new path/removing old one
            node_list.remove(goal_node)
            for node in prev_node_list:
                if (node not in node_list and (node[0] != current_node[0] or node[1] != current_node[1] )): #Eliminates old path, but not robot
                    if (dStarArena[node[0]][node[1]] == 0):
                        draw_dstar.square("white", dStarArena, node)
                    elif (dStarArena[node[0]][node[1]] == 1):
                        draw_dstar.square("gray", dStarArena, node)
                    else:
                        draw_dstar.square("light gray", dStarArena, node)
            for node in node_list:
                draw_dstar.square("yellow", dStarArena, node)

            #The path has been updated and doesn't need to be reupdated
            needs_replan = False
        #END DSTAR

        mpc_goal = path_list[min(lookAhead - 1, len(path_list) - 1)] # The MPC controller looks ahead on the path for its goal

        #MPC
        #This block calculates velocity/angular velocity and changes the angle of the robot
        for i in range(mpc.iterations): # Run MPC for one timestep

            samples = mpc.generateSamples()

            states = mpc.generateState(samples, robotState, arena)
            costs = mpc.calculateCosts(states, mpc_goal, arena)

            mpc.findNewMeans(costs,samples)

            if (mpc.itera < 6):
                mpc.itera+= 1
            else:
                mpc.itera = 1

        mpc.itera = 1

        velocity = mpc.means[0][0] #What the mpc calculated to move
        angularVelocity = mpc.means[0][1]

        #"Move robot"
        newAngle = robotState[2] + angularVelocity * mpc.timeStepTime #New angle = current + movement
        if (newAngle < 0):
            newAngle = 6.28318530718 + newAngle
        if (newAngle >= 6.28318530718):
            newAngle -= 6.28318530718 

        draw_mpc.drawDir([(robotState[1]),(robotState[0])], arena, newAngle, velocity * 5, -1) #Draw new angle
        #END MPC

        print(f"Moving at {velocity:.2f}", f"velocity at {newAngle:.2f} angle." ) #Input- each one per timestep
        input("Press Enter to continue a timestep") 

        #MPC
        #This block moves the robot based on calculated values
        draw_mpc.circle("white",(robotState[0] / arenaWidth),(robotState[1] / arenaHeight), draw_mpc.robotRad)

        robotState[2] = newAngle

        xMovement = velocity * mpc.timeStepTime * math.cos((newAngle)) #X movement = linear mvmt * cos(angle)
        yMovement = velocity * mpc.timeStepTime * math.sin((newAngle))

        robotState[0] += xMovement
        robotState[1] += yMovement #increase state

        draw_mpc.circle("blue",(robotState[0] / arenaWidth),(robotState[1] / arenaHeight), draw_mpc.robotRad)

        #Reset after iterations
        mpc.means = numpy.full((mpc.timesteps,2), 0.0) 
        mpc.stdDevs = numpy.full((mpc.timesteps, 2), mpc.initStandardDeviation)
        #END MPC

        #DSTAR
        #This block moves the dstar model and updates the visible map, and refinds path
        draw_dstar.square("white" , dStarArena, current_node)

        current_node = [int(robotState[1]), int(robotState[0])] #Move dstar model (based on mpc- nongrid based)

        if (current_node in path_list): #Robot has moved to this point on the path- updates so lookahead knows how far ahead to look
            path_list.remove(current_node)
        elif (len(path_list) >= 2):
            path_list.pop(0) #Otherwise keep progress moving by removing the first one

        draw_dstar.square("blue" , dStarArena, current_node)

        prev_map = []
        for row in range(len(dStarArena)):
            prev_map.append(dStarArena[row].copy()) #set previous map

        dStarArena = updateVisibleGrid(arena, dStarArena, current_node, goal_node) #update the map

        #Updates nodes when map changes
        if (dStarArena != prev_map):
            needs_replan = True

            km += (((prev_node[0]-current_node[0])**2 + (prev_node[1]-current_node[1])**2)**0.5) #Accumulation 
            prev_node = current_node.copy()


            for i in range(len(prev_map)):
                for j in range(len(prev_map[i])):
                    if (dStarArena[i][j] != prev_map[i][j]):

                        if (dStarArena[i][j] == 1): #Redraw map
                            draw_dstar.square("gray", dStarArena, [i,j])
                        if (dStarArena[i][j] == 2):
                            draw_dstar.square("light gray", dStarArena, [i,j])

                        #Update itself and all surrounding nodes
                        draw_dstar.update_node([i,j],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)
                        
                        if (i > 0): #above
                            draw_dstar.update_node([i-1,j],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)
                        if (i < len(node_value_list)-1): #below
                            draw_dstar.update_node([i+1,j],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)
                        if (j > 0): #left
                            draw_dstar.update_node([i,j-1],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)
                        if (j < len(node_value_list[0])-1): #right
                            draw_dstar.update_node([i,j+1],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)
                        if (i > 0 and j > 0): #Topleft
                            draw_dstar.update_node([i-1,j-1],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)
                        if (i < (len(node_value_list)-1) and j > 0): #Bottomleft
                            draw_dstar.update_node([i+1,j-1],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)
                        if (i < (len(node_value_list)-1) and j < len(node_value_list[0])-1): #Bottomright
                            draw_dstar.update_node([i+1,j+1],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)
                        if (i > 0 and j < len(node_value_list[0])-1): #Topright
                            draw_dstar.update_node([i-1,j+1],goal_node,dStarArena,current_node,node_value_list,node_queue,km, False)

            draw_dstar.find_path(current_node,node_queue,node_value_list,km,dStarArena,goal_node, False)
        
    input("You did it")


combo(2)