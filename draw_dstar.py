from queue import PriorityQueue
from sys import maxsize
import turtle
import math
import numpy as np

from Map_maker import map_generate

screen = turtle.Screen()
pen = turtle.RawTurtle(screen)
xlength = screen.window_width()-20
ylength = screen.window_height()-20
screen.screensize(xlength,ylength)
pen.shapesize(0.1,0.1,0.1)
pen.speed(100)
screen.tracer(20,0.01)

pen.up()
pen.goto(-1*xlength/2,ylength/2)
pen.pencolor("black")

def translate(node,arena):
    x = node[1]*(xlength/(len(arena[0]))) - (xlength/2)
    y = node[0]*(ylength/(len(arena)))*-1 + (ylength/2)
    return (x,y)

def square(color,arena,node):
    pen.goto(translate(node,arena))
    pen.down()
    pen.fillcolor(color)
    pen.begin_fill()
    for i in range(2):
        pen.forward(xlength/(len(arena[0])))
        pen.right(90)
        pen.forward(ylength/(len(arena)))
        pen.right(90)
    pen.end_fill()
    pen.up()

def startSquare(color,arena):
    pen.down()
    pen.fillcolor(color)
    pen.begin_fill()
    for i in range(2):
        pen.forward(xlength/(len(arena[0])))
        pen.right(90)
        pen.forward(ylength/(len(arena)))
        pen.right(90)
    pen.end_fill()
    pen.up()


def drawstart(arena,start,goal):

    for i in range(len(arena)):
        for j in range(len(arena[0])):
            if (arena[i][j] == 0):
                startSquare("white",arena)
            elif (arena[i][j] == 1):
                startSquare("gray",arena)
            else:
                startSquare("light gray",arena)

            pen.goto(pen.position()[0] + xlength/(len(arena[0])), pen.position()[1])

        pen.goto(-1*xlength/2, pen.position()[1] - ylength/(len(arena)))

    pen.goto(translate(start,arena))
    startSquare("blue",arena)

    pen.goto(translate(goal,arena))
    startSquare("green",arena)


# returns the lowest priority in the queue
def topKey(nodequeue: PriorityQueue):
    if (nodequeue.empty()):
        return((maxsize,maxsize))
    else:
        return nodequeue.queue[0][0]

# inserts a node into the queue
def insert(nodequeue: PriorityQueue, node: list, key: tuple):
    nodequeue.put((key,node))

# removes a given node from the queue
def remove(nodequeue: PriorityQueue, node: list):
    nodelist = nodequeue.queue
    newlist = nodelist.copy()

    for node_tuple in nodelist:
        if node_tuple[1] == node:
            newlist.remove(node_tuple)
    
    while (not nodequeue.empty()):
        nodequeue.get()

    for node_tuple in newlist:
        nodequeue.put(node_tuple)

# Defines the hueristic used for calculating the priority of a node (the first run uses AStar, subsequent uses a focused value)
def hueristic(node: list, current_node: list, init:bool):
    #TODO use np.sqrt()
    if (init):
        h = ((((node[0]-current_node[0])**2 + (node[1]-current_node[1])**2 )**.50))
    else:
        h = 0.9 * ((((node[0]-current_node[0])**2 + (node[1]-current_node[1])**2 )**.5))
    return h

# Calculates the priority of a node based on its g and rhs values
def calculate_key(node: list, node_value_list: list, current_node: list, km: float, init: bool):
    g_value = node_value_list[node[0]][node[1]][0]
    rhs_value = node_value_list[node[0]][node[1]][1]
    min_val = min(g_value,rhs_value)

    h = hueristic(node,current_node, init)

    key1 = min_val + h + km
    key2 = min_val

    return ((key1, key2))

'''
Initializes the Dstar algorithm by creating a priority queue, initializing accumulation (km),
creating a list of node values (g and rhs), all initialized to infinity, setting the rhs of the goal node to 0, and inserting
the goal node into the priority queue
'''
def initialize_np(init_map: np.array, goal: np.array, start: np.array):
    node_queue = PriorityQueue()
    km = 0

    node_values_list = maxsize * np.ones((init_map.shape[0], init_map.shape[1], 2)) #2d Array of nodes: each value is [Distance (g), Estimate (rhs)]

    node_values_list[goal[0], goal[1], 1] = 0

    insert(node_queue,goal,calculate_key(goal,node_values_list,start,km, False))

    return node_queue, km, node_values_list


def initialize(init_map: list, goal: list, start: list):
    node_queue = PriorityQueue()
    km = 0

    node_values_list = [] #2d Array of nodes: each value is [Distance (g), Estimate (rhs)]
    for i in range(len(init_map)):
        node_values_list.append([])
        for j in range(len(init_map[i])):
            node_values_list[i].append([maxsize,maxsize])
    
    node_values_list[goal[0]][goal[1]][1] = 0

    insert(node_queue,goal,calculate_key(goal,node_values_list,start,km, False))

    return node_queue, km, node_values_list


'''
Calculates the RHS (estimate value) of a given node by: first checking if it's an obstacle, then
checking each surrounding node, calculating what the distance value should be based on those nodes,
and taking the lowest value.
'''
def calculate_RHS(node:list, current_map:list, nodevalues:list):
    if (current_map[node[0]][node[1]] == 1 or current_map[node[0]][node[1]] == 2): #Chheck for obstacle
        return maxsize

    surrounding_values = [] #a list of distance values (floats)

    #For each node (all 8 directions)

    if (node[0] > 0): #above
        if (current_map[node[0]-1][node[1]] == 1 or current_map[node[0]-1][node[1]] == 2): #Check if off grid or it is an obstacle
            surrounding_values.append(maxsize) #if so, add inf to the list
        else:
            g_val = nodevalues[node[0]-1][node[1]][0] #Otherwise get the gvalue of the node we're checking
            surrounding_values.append(g_val + 1)      #and add 1 or 1.4 based on the euclidean distance to get to the node we're calculating for
    if (node[0] < len(current_map)-1): #below
        if (current_map[node[0]+1][node[1]] == 1 or current_map[node[0]+1][node[1]] == 2):
            surrounding_values.append(maxsize)
        else:
            g_val = nodevalues[node[0]+1][node[1]][0]
            surrounding_values.append(g_val + 1)
    if (node[1] > 0): #left
        if (current_map[node[0]][node[1]-1] == 1 or current_map[node[0]][node[1]-1] == 2):
            surrounding_values.append(maxsize)
        else:
            g_val = nodevalues[node[0]][node[1]-1][0]
            surrounding_values.append(g_val + 1)
    if (node[1] < len(current_map[0])-1): #right
        if (current_map[node[0]][node[1]+1] == 1 or current_map[node[0]][node[1]+1] == 2):
            surrounding_values.append(maxsize)
        else:
            g_val = nodevalues[node[0]][node[1]+1][0]
            surrounding_values.append(g_val + 1)    
    if (node[0] > 0 and node[1] > 0): #Topleft
        if (current_map[node[0]-1][node[1]-1] == 1 or current_map[node[0]-1][node[1]-1] == 2):
            surrounding_values.append(maxsize)
        else:
            g_val = nodevalues[node[0]-1][node[1]-1][0]
            surrounding_values.append(g_val + 1.4)       
    if (node[0] < (len(current_map)-1) and node[1] > 0): #Bottomleft
        if (current_map[node[0]+1][node[1]-1] == 1 or current_map[node[0]+1][node[1]-1] == 2):
            surrounding_values.append(maxsize)
        else:
            g_val = nodevalues[node[0]+1][node[1]-1][0]
            surrounding_values.append(g_val + 1.4)            
    if (node[0] < (len(current_map)-1) and node[1] < len(current_map[0])-1): #Bottomright
        if (current_map[node[0]+1][node[1]+1] == 1 or current_map[node[0]+1][node[1]+1] == 2):
            surrounding_values.append(maxsize)
        else:
            g_val = nodevalues[node[0]+1][node[1]+1][0]
            surrounding_values.append(g_val + 1.4)            
    if (node[0] > 0 and node[1] < len(current_map[0])-1): #Topright
        if (current_map[node[0]-1][node[1]+1] == 1 or current_map[node[0]-1][node[1]+1] == 2):
            surrounding_values.append(maxsize)
        else:
            g_val = nodevalues[node[0]-1][node[1]+1][0]
            surrounding_values.append(g_val + 1.4)     

    return min(surrounding_values)       

'''
Updates a node's values by calculating its RHS (estimate value). It removes the node from the queue (based on old value) and
replaces it on the queue if its values are locally inconsistent (if g != RHS)
'''
def update_node(node: list, goal:list, current_map:list, current_node:list, nodevalues:list, nodequeue:PriorityQueue, km:int, init: bool):
    if (node != goal):
        nodevalues[node[0]][node[1]][1] = calculate_RHS(node, current_map, nodevalues) #Calculate RHS

    #Calculate if it's in the queue and removes
    queue_contains = False 
    for node_tuple in nodequeue.queue:
        if (node_tuple[1] == node):
            queue_contains = True

    if (queue_contains):
        remove(nodequeue, node)

    g_val = nodevalues[node[0]][node[1]][0]
    rhs_val = nodevalues[node[0]][node[1]][1]

    # Place it on the queue if not consistent
    if (g_val != rhs_val):
        insert(nodequeue,node,calculate_key(node,nodevalues,current_node,km, init))
        # if (current_map[node[0]][node[1]] == 0):
        #     #square("lime", current_map, node)
        # else:
        #     #square("forest green", current_map, node)

#Updates the map with new grid
def update_map(current_map): #however map is be updated
    
    doneEditing = True
    new_map = current_map.copy()

    def click_to_update(xpos, ypos):
        if (not doneEditing):
            x_node = math.trunc(((xpos+(xlength/2))/xlength)*len(current_map[0]))
            y_node = math.trunc((((-ypos)+ylength/2)/ylength)*len(current_map))
            if (new_map[y_node][x_node] == 0):
                new_map[y_node][x_node] = 1
                square("gray",new_map, [y_node, x_node])
            else:
                new_map[y_node][x_node] = 0
                square("white",new_map, [y_node,x_node])

    screen.onclick(click_to_update,1,True)

    s = screen.textinput("Update Map?", "Y/N\n")
    if (s == "y" or s == "Y"):
        
        doneEditing = False
        input("Press Enter to continue")
        doneEditing = True

        return new_map
    else:
        return current_map

'''
Find_Path calculates the path for DStar by looping until the current node is locally consistent (g value = rhs) and the priority of the current node is the lowest in the queue.
It picks the lowest priority node, checks whether its priority is correct, then updates its g value (distance). If the g value is higher then the estimate, it lowers the g value to the estimate.
If the g value is lower then the estimate, it sets it for correction by setting the g value to infinity. It then marks all surrounding nodes to be checked.

Through this process, all nodes on the grid have their g value calculated correctly so the path can be found.
'''
def find_path(current_node:list, nodequeue:PriorityQueue, nodevalues:list, km:int, current_map:list, goal:list, init: bool):
    while (topKey(nodequeue) < calculate_key(current_node,nodevalues,current_node,km, init) or nodevalues[current_node[0]][current_node[1]][0] != nodevalues[current_node[0]][current_node[1]][1]):
        #Loop until current node is locally consistent and priority is lowest in the queue
        
        old_key = topKey(nodequeue)
        chosen_node = nodequeue.get()[1] #Chosen node to check
        #square("orange", current_map, chosen_node)

        #If the priority of the node was incorrect, add back to the queue with the correct priority.
        if (old_key < calculate_key(chosen_node,nodevalues,current_node,km, init)):
            insert(nodequeue,chosen_node,calculate_key(chosen_node,nodevalues,current_node,km, init)) 

        # If g value is greater then rhs
        elif (nodevalues[chosen_node[0]][chosen_node[1]][0] > nodevalues[chosen_node[0]][chosen_node[1]][1]):
            nodevalues[chosen_node[0]][chosen_node[1]][0] = nodevalues[chosen_node[0]][chosen_node[1]][1] #Lower the g value

            #update all surrounding nodes
            if (chosen_node[0] > 0): #above
                update_node([chosen_node[0]-1, chosen_node[1]], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] < len(nodevalues)-1): #below
                update_node([chosen_node[0]+1, chosen_node[1]], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[1] > 0): #left
                update_node([chosen_node[0], chosen_node[1]-1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[1] < len(nodevalues[0])-1): #right
                update_node([chosen_node[0], chosen_node[1]+1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] > 0 and chosen_node[1] > 0): #Topleft
                update_node([chosen_node[0]-1, chosen_node[1]-1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] < (len(nodevalues)-1) and chosen_node[1] > 0): #Bottomleft
                update_node([chosen_node[0]+1, chosen_node[1]-1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] < (len(nodevalues)-1) and chosen_node[1] < len(nodevalues[0])-1): #Bottomright    
                update_node([chosen_node[0]+1, chosen_node[1]+1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] > 0 and chosen_node[1] < len(nodevalues[0])-1): #Topright    
                update_node([chosen_node[0]-1, chosen_node[1]+1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
        
        #G is lower then rhs
        else: 
            #Set g to infinity
            nodevalues[chosen_node[0]][chosen_node[1]][0] = maxsize

            #Update this node
            update_node(chosen_node,goal,current_map,current_node,nodevalues,nodequeue,km, init)

            #update all the surrounding nodes
            if (chosen_node[0] > 0): #above
                update_node([chosen_node[0]-1, chosen_node[1]], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] < len(nodevalues)-1): #below
                update_node([chosen_node[0]+1, chosen_node[1]], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[1] > 0): #left
                update_node([chosen_node[0], chosen_node[1]-1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[1] < len(nodevalues[0])-1): #right
                update_node([chosen_node[0], chosen_node[1]+1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] > 0 and chosen_node[1] > 0): #Topleft
                update_node([chosen_node[0]-1, chosen_node[1]-1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] < (len(nodevalues)-1) and chosen_node[1] > 0): #Bottomleft
                update_node([chosen_node[0]+1, chosen_node[1]-1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] < (len(nodevalues)-1) and chosen_node[1] < len(nodevalues[0])-1): #Bottomright    
                update_node([chosen_node[0]+1, chosen_node[1]+1], goal, current_map, current_node, nodevalues, nodequeue, km, init)
            if (chosen_node[0] > 0 and chosen_node[1] < len(nodevalues[0])-1): #Topright    
                update_node([chosen_node[0]-1, chosen_node[1]+1], goal, current_map, current_node, nodevalues, nodequeue, km, init)

        # if (current_map[chosen_node[0]][chosen_node[1]] == 0): #CHANGED
        #     square("white",current_map, chosen_node)                 #CHANGED
        # if (current_map[chosen_node[0]][chosen_node[1]] == 1): #CHANGED
        #     square("gray",current_map, chosen_node)                 #CHANGED
        # if (chosen_node == current_node): #CHANGED
        #     square("blue",current_map, chosen_node)                  #CHANGED
        # if (chosen_node == goal): #CHANGED
        #     square("green",current_map, chosen_node)                  #CHANGED

        if nodequeue.qsize() == 0:
            print ("Error: No Path")
            break
'''
Main control loop: start by initializing map, queue, and node values, then run find path once. 
This block creates a temporary node at the start, and picks the lowest g value (lowest distance to goal) as the next step on the path, adds it to the list, and
repeats until the goal is reached. All of these points in order are the path.

This loop then also checks for the same lowest g-value, and 'moves' the robot to that position (one step along the path)

After, the current map is updated with new information. If any nodes changed, then it and its surroundings are updated with the estimates.
'''
def dstarlite(start, goal, init_map):

    # if start = goal or map/goal are obstacle
    if (start[0]==goal[0] and start[1] == goal[1] or init_map[start[0]][start[1]]==1 or init_map[goal[0]][goal[1]]==1):
        print ("Error: No path")
        return

    drawstart(init_map,start,goal)


    prev_node = start
    current_node = start

    current_map = [] 
    for row in range(len(init_map)):
        current_map.append(init_map[row].copy()) #Sets current map 

    init_values = initialize(init_map,goal,start) #initialize queue, accumulation, and node values
    node_queue = init_values[0]
    km = init_values[1]
    node_value_list = init_values[2]

    find_path(start, node_queue, node_value_list, km, init_map, goal, True)  #Find initial path

    temp_list = [] #Drawing block
    prev_temp_list = [] #Drawing block
    while (current_node[0] != goal[0] or current_node[1] != goal[1]): #Until robot reaches goal

        if (node_value_list[current_node[0]][current_node[1]][0] >= (maxsize)): #If g value of current node is inf
            print ("No known path")
            break

        #Drawing Block- calculates the whole path and draws it
        prev_temp_list.clear()
        for node in temp_list:
            prev_temp_list.append(node.copy()) #Prev_list- last timestep's list
        temp_list.clear()                      #temp_list- the current timestep's list of path

        temp_node = current_node.copy() #temp node used to draw whole path
        while (temp_node[0] != goal[0] or temp_node[1] != goal[1]): #until the temp node reaches the goal

            #Check all surrounding nodes for the lowest g value

            temp_gvals = [] #find smallest g value (closest to goal)
            if (temp_node[0] > 0): #above
                if (current_map[temp_node[0]-1][temp_node[1]] == 0):
                    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + 1, [temp_node[0]-1, temp_node[1]])) 
                #else:
                #    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + maxsize, [temp_node[0]-1, temp_node[1]]))
            if (temp_node[0] < len(node_value_list)-1): #below
                if (current_map[temp_node[0]+1][temp_node[1]] == 0):
                    temp_gvals.append((node_value_list[temp_node[0]+1][temp_node[1]][0] + 1, [temp_node[0]+1, temp_node[1]]))
               # else:
                #    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + maxsize, [temp_node[0]+1, temp_node[1]]))
            if (temp_node[1] > 0): #left
                if (current_map[temp_node[0]][temp_node[1]-1] == 0):
                    temp_gvals.append((node_value_list[temp_node[0]][temp_node[1]-1][0] + 1, [temp_node[0], temp_node[1]-1]))
                #else:
                #    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + maxsize, [temp_node[0], temp_node[1]-1]))
            if (temp_node[1] < len(node_value_list[0])-1): #right
                if (current_map[temp_node[0]][temp_node[1]+1] == 0):
                    temp_gvals.append((node_value_list[temp_node[0]][temp_node[1]+1][0] + 1, [temp_node[0], temp_node[1]+1]))
               # else:
               #     temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + maxsize, [temp_node[0], temp_node[1]+1]))
            if (temp_node[0] > 0 and temp_node[1] > 0): #Topleft
                if (current_map[temp_node[0]-1][temp_node[1]-1] == 0):
                    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]-1][0] + 1.4, [temp_node[0]-1, temp_node[1]-1]))
                #else:
                #    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + maxsize, [temp_node[0]-1, temp_node[1]-1]))
            if (temp_node[0] < (len(node_value_list)-1) and temp_node[1] > 0): #Bottomleft
                if (current_map[temp_node[0]+1][temp_node[1]-1] == 0):
                    temp_gvals.append((node_value_list[temp_node[0]+1][temp_node[1]-1][0] + 1.4, [temp_node[0]+1, temp_node[1]-1]))
                #else:
                #    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + maxsize, [temp_node[0]+1, temp_node[1]-1]))
            if (temp_node[0] < (len(node_value_list)-1) and temp_node[1] < len(node_value_list[0])-1): #Bottomright
                if (current_map[temp_node[0]+1][temp_node[1]+1] == 0):
                    temp_gvals.append((node_value_list[temp_node[0]+1][temp_node[1]+1][0] + 1.4, [temp_node[0]+1, temp_node[1]+1]))
                #else:
                #    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + maxsize, [temp_node[0]+1, temp_node[1]+1]))
            if (temp_node[0] > 0 and temp_node[1] < len(node_value_list[0])-1): #Topright
                if (current_map[temp_node[0]-1][temp_node[1]+1] == 0):
                    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]+1][0] + 1.4, [temp_node[0]-1, temp_node[1]+1]))
                #else:
                #    temp_gvals.append((node_value_list[temp_node[0]-1][temp_node[1]][0] + maxsize, [temp_node[0]-1, temp_node[1]+1]))
        
            temp_min_val = min(temp_gvals) #pick lowest g value
            temp_node = temp_min_val[1]
            temp_list.append(temp_node)
            temp_gvals.clear()

        temp_list.remove(goal) #Draws the current map and replaces the old one
        for node in prev_temp_list:
            if (node not in temp_list):
                if (current_map[node[0]][node[1]] == 0):
                    square("white", current_map, node)
                else:
                    square("gray", current_map, node)
        for node in temp_list:
            square("yellow", current_map, node)


        #End drawing Block

        #After drawing is done, find the lowest g value from surrounding nodes

        surrounding_gvalues = [] #find smallest g value (closest to goal)
        if (current_node[0] > 0): #above
            if (current_map[current_node[0]-1][current_node[1]] == 0):
                surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + 1, [current_node[0]-1, current_node[1]])) 
            #else:
            #    surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + maxsize, [current_node[0]-1, current_node[1]]))
        if (current_node[0] < len(node_value_list)-1): #below
            if (current_map[current_node[0]+1][current_node[1]] == 0):
                surrounding_gvalues.append((node_value_list[current_node[0]+1][current_node[1]][0] + 1, [current_node[0]+1, current_node[1]]))
           # else:
            #    surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + maxsize, [current_node[0]+1, current_node[1]]))
        if (current_node[1] > 0): #left
            if (current_map[current_node[0]][current_node[1]-1] == 0):
                surrounding_gvalues.append((node_value_list[current_node[0]][current_node[1]-1][0] + 1, [current_node[0], current_node[1]-1]))
            #else:
            #    surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + maxsize, [current_node[0], current_node[1]-1]))
        if (current_node[1] < len(node_value_list[0])-1): #right
            if (current_map[current_node[0]][current_node[1]+1] == 0):
                surrounding_gvalues.append((node_value_list[current_node[0]][current_node[1]+1][0] + 1, [current_node[0], current_node[1]+1]))
           # else:
           #     surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + maxsize, [current_node[0], current_node[1]+1]))
        if (current_node[0] > 0 and current_node[1] > 0): #Topleft
            if (current_map[current_node[0]-1][current_node[1]-1] == 0):
                surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]-1][0] + 1.4, [current_node[0]-1, current_node[1]-1]))
            #else:
            #    surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + maxsize, [current_node[0]-1, current_node[1]-1]))
        if (current_node[0] < (len(node_value_list)-1) and current_node[1] > 0): #Bottomleft
            if (current_map[current_node[0]+1][current_node[1]-1] == 0):
                surrounding_gvalues.append((node_value_list[current_node[0]+1][current_node[1]-1][0] + 1.4, [current_node[0]+1, current_node[1]-1]))
            #else:
            #    surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + maxsize, [current_node[0]+1, current_node[1]-1]))
        if (current_node[0] < (len(node_value_list)-1) and current_node[1] < len(node_value_list[0])-1): #Bottomright
            if (current_map[current_node[0]+1][current_node[1]+1] == 0):
                surrounding_gvalues.append((node_value_list[current_node[0]+1][current_node[1]+1][0] + 1.4, [current_node[0]+1, current_node[1]+1]))
            #else:
            #    surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + maxsize, [current_node[0]+1, current_node[1]+1]))
        if (current_node[0] > 0 and current_node[1] < len(node_value_list[0])-1): #Topright
            if (current_map[current_node[0]-1][current_node[1]+1] == 0):
                surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]+1][0] + 1.4, [current_node[0]-1, current_node[1]+1]))
            #else:
            #    surrounding_gvalues.append((node_value_list[current_node[0]-1][current_node[1]][0] + maxsize, [current_node[0]-1, current_node[1]+1]))



        min_value = min(surrounding_gvalues) # 'move robot'
        square("white" , current_map, current_node)
        current_node = min_value[1]
        square ("blue" , current_map, current_node)
        surrounding_gvalues.clear()

        prev_map = [] 
        for row in range(len(current_map)):
            prev_map.append(current_map[row].copy()) #set previous map

        current_map = update_map(current_map) #update the current map

        if (current_map != prev_map):

            km += (((prev_node[0]-current_node[0])**2 + (prev_node[1]-current_node[1])**2)**0.5) 
            #Add to the accumulation value the distance from the last point (of changed map) to the current point
            prev_node = current_node.copy() #update the prev_node

            for i in range(len(prev_map)):
                for j in range(len(prev_map[i])):
                    if (current_map[i][j] != prev_map[i][j]): #for all differing values, update it and its surrounding nodes

                        update_node([i,j],goal,current_map,current_node,node_value_list,node_queue,km, False)
                        
                        if (i > 0): #above
                            update_node([i-1,j],goal,current_map,current_node,node_value_list,node_queue,km, False)
                        if (i < len(node_value_list)-1): #below
                            update_node([i+1,j],goal,current_map,current_node,node_value_list,node_queue,km, False)
                        if (j > 0): #left
                            update_node([i,j-1],goal,current_map,current_node,node_value_list,node_queue,km, False)
                        if (j < len(node_value_list[0])-1): #right
                            update_node([i,j+1],goal,current_map,current_node,node_value_list,node_queue,km, False)
                        if (i > 0 and j > 0): #Topleft
                            update_node([i-1,j-1],goal,current_map,current_node,node_value_list,node_queue,km, False)
                        if (i < (len(node_value_list)-1) and j > 0): #Bottomleft
                            update_node([i+1,j-1],goal,current_map,current_node,node_value_list,node_queue,km, False)
                        if (i < (len(node_value_list)-1) and j < len(node_value_list[0])-1): #Bottomright
                            update_node([i+1,j+1],goal,current_map,current_node,node_value_list,node_queue,km, False)
                        if (i > 0 and j < len(node_value_list[0])-1): #Topright
                            update_node([i-1,j+1],goal,current_map,current_node,node_value_list,node_queue,km, False)

            find_path(current_node,node_queue,node_value_list,km,current_map,goal, False) #Replan