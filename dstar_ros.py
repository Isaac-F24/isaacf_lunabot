from queue import PriorityQueue
from sys import maxsize
import math
import numpy as np
from Collison_avoid import expand_grid

from Map_maker import map_generate

class Dstar():

    '''
    Initializes the Dstar algorithm by creating a priority queue, initializing accumulation (km),
    creating a list of node values (g and rhs), all initialized to infinity, setting the rhs of the goal node to 0, and inserting
    the goal node into the priority queue
    '''
    def __init__(self, goal, start, init_map, radius, resolution, x_offset, y_offset):
        self.node_queue = PriorityQueue()
        self.km = 0.0

        self.node_values_list = maxsize * np.ones((init_map.shape[0], init_map.shape[1], 2)) #2d Array of nodes: each value is [Distance (g), Estimate (rhs)]

        self.x_offset = x_offset
        self.y_offset = y_offset

        self.current_map = init_map

        self.res = resolution

        goal = self.convertToGrid(goal)

        self.node_values_list[goal[0], goal[1], 1] = 0

        start = self.convertToGrid(start)

        self.current_node = start
        self.prev_node = start

        self.goal = goal

        self.radius = radius

        self.needs_new_path = True #updates when map changes to know when to create new path 
        
        self.insert(goal,self.calculate_key(goal, False))



    #Update Pose (current position)
    def update_position(self, coords):
        position = self.convertToGrid(coords)
        self.current_node = position

    def convertToGrid(self, pos):
        shifted_pos = [pos[0] - self.x_offset, pos[1] - self.y_offset]
        coord = [int(self.current_map.shape[0] - shifted_pos[1] / self.res + 0.5), int(shifted_pos[0] / self.res + 0.5)]
        return coord
    
    def convertToReal(self, coord):
        pos = [(coord[1] + 0.5) * self.res, (self.current_map.shape[0] - coord[0] + 0.5) * self.res]
        pos = [pos[0] + self.x_offset, pos[1] + self.y_offset]
        return pos

    # returns the lowest priority in the queue
    def topKey(self):
        if (self.node_queue.empty()):
            return((maxsize,maxsize))
        else:
            return self.node_queue.queue[0][0]

    # inserts a node into the queue
    def insert(self, node: list, key: tuple):
        self.node_queue.put((key,node))

    # removes a given node from the queue
    def remove(self, node: list):
        nodelist = self.node_queue.queue
        newlist = nodelist.copy()

        for node_tuple in nodelist:
            if node_tuple[1] == node:
                newlist.remove(node_tuple)
        
        while (not self.node_queue.empty()):
            self.node_queue.get()

        for node_tuple in newlist:
            self.node_queue.put(node_tuple)

    # Defines the hueristic used for calculating the priority of a node (the first run uses AStar, subsequent uses a focused value)
    def hueristic(self, node: list, init:bool):
        #TODO use np.sqrt()
        if (init):
            h = ((((node[0]-self.current_node[0])**2 + (node[1]-self.current_node[1])**2 )**.50))
        else:
            h = 0.9 * ((((node[0]-self.current_node[0])**2 + (node[1]-self.current_node[1])**2 )**.5))
        return h

    # Calculates the priority of a node based on its g and rhs values
    def calculate_key(self, node: list, init: bool):
        g_value = self.node_values_list[node[0]][node[1]][0]
        rhs_value = self.node_values_list[node[0]][node[1]][1]
        min_val = min(g_value,rhs_value)

        h = self.hueristic(node, init)

        key1 = min_val + h + self.km
        key2 = min_val

        return ((key1, key2))

    '''
    Calculates the RHS (estimate value) of a given node by: first checking if it's an obstacle, then
    checking each surrounding node, calculating what the distance value should be based on those nodes,
    and taking the lowest value.
    '''
    def calculate_RHS(self, node:list):
        if (self.current_map[node[0]][node[1]] == 1 or self.current_map[node[0]][node[1]] == 2): #Chheck for obstacle
            return maxsize

        surrounding_values = [] #a list of distance values (floats)

        #For each node (all 8 directions)

        if (node[0] > 0): #above
            if (self.current_map[node[0]-1][node[1]] == 1 or self.current_map[node[0]-1][node[1]] == 2): #Check if off grid or it is an obstacle
                surrounding_values.append(maxsize) #if so, add inf to the list
            else:
                g_val = self.node_values_list[node[0]-1][node[1]][0] #Otherwise get the gvalue of the node we're checking
                surrounding_values.append(g_val + 1)      #and add 1 or 1.4 based on the euclidean distance to get to the node we're calculating for
        if (node[0] < len(self.current_map)-1): #below
            if (self.current_map[node[0]+1][node[1]] == 1 or self.current_map[node[0]+1][node[1]] == 2):
                surrounding_values.append(maxsize)
            else:
                g_val = self.node_values_list[node[0]+1][node[1]][0]
                surrounding_values.append(g_val + 1)
        if (node[1] > 0): #left
            if (self.current_map[node[0]][node[1]-1] == 1 or self.current_map[node[0]][node[1]-1] == 2):
                surrounding_values.append(maxsize)
            else:
                g_val = self.node_values_list[node[0]][node[1]-1][0]
                surrounding_values.append(g_val + 1)
        if (node[1] < len(self.current_map[0])-1): #right
            if (self.current_map[node[0]][node[1]+1] == 1 or self.current_map[node[0]][node[1]+1] == 2):
                surrounding_values.append(maxsize)
            else:
                g_val = self.node_values_list[node[0]][node[1]+1][0]
                surrounding_values.append(g_val + 1)    
        if (node[0] > 0 and node[1] > 0): #Topleft
            if (self.current_map[node[0]-1][node[1]-1] == 1 or self.current_map[node[0]-1][node[1]-1] == 2):
                surrounding_values.append(maxsize)
            else:
                g_val = self.node_values_list[node[0]-1][node[1]-1][0]
                surrounding_values.append(g_val + 1.4)       
        if (node[0] < (len(self.current_map)-1) and node[1] > 0): #Bottomleft
            if (self.current_map[node[0]+1][node[1]-1] == 1 or self.current_map[node[0]+1][node[1]-1] == 2):
                surrounding_values.append(maxsize)
            else:
                g_val = self.node_values_list[node[0]+1][node[1]-1][0]
                surrounding_values.append(g_val + 1.4)            
        if (node[0] < (len(self.current_map)-1) and node[1] < len(self.current_map[0])-1): #Bottomright
            if (self.current_map[node[0]+1][node[1]+1] == 1 or self.current_map[node[0]+1][node[1]+1] == 2):
                surrounding_values.append(maxsize)
            else:
                g_val = self.node_values_list[node[0]+1][node[1]+1][0]
                surrounding_values.append(g_val + 1.4)            
        if (node[0] > 0 and node[1] < len(self.current_map[0])-1): #Topright
            if (self.current_map[node[0]-1][node[1]+1] == 1 or self.current_map[node[0]-1][node[1]+1] == 2):
                surrounding_values.append(maxsize)
            else:
                g_val = self.node_values_list[node[0]-1][node[1]+1][0]
                surrounding_values.append(g_val + 1.4)     

        return min(surrounding_values)       

    '''
    Updates a node's values by calculating its RHS (estimate value). It removes the node from the queue (based on old value) and
    replaces it on the queue if its values are locally inconsistent (if g != RHS)
    '''
    def update_node(self, node: list, init: bool):
        if (node != self.goal):
            self.node_values_list[node[0]][node[1]][1] = self.calculate_RHS(node) #Calculate RHS

        #Calculate if it's in the queue and removes
        queue_contains = False 
        for node_tuple in self.node_queue.queue:
            if (node_tuple[1] == node):
                queue_contains = True

        if (queue_contains):
            self.remove(node)

        g_val = self.node_values_list[node[0]][node[1]][0]
        rhs_val = self.node_values_list[node[0]][node[1]][1]

        # Place it on the queue if not consistent
        if (g_val != rhs_val):
            self.insert(node,self.calculate_key(node,init))


    '''
    Find_Path calculates the path for DStar by looping until the current node is locally consistent (g value = rhs) and the priority of the current node is the lowest in the queue.
    It picks the lowest priority node, checks whether its priority is correct, then updates its g value (distance). If the g value is higher then the estimate, it lowers the g value to the estimate.
    If the g value is lower then the estimate, it sets it for correction by setting the g value to infinity. It then marks all surrounding nodes to be checked.

    Through this process, all nodes on the grid have their g value calculated correctly so the path can be found.
    '''
    def find_path(self, init: bool):
        while (self.topKey() < self.calculate_key(self.current_node,init) or self.node_values_list[self.current_node[0]][self.current_node[1]][0] != self.node_values_list[self.current_node[0]][self.current_node[1]][1]):
            #Loop until current node is locally consistent and priority is lowest in the queue
            old_key = self.topKey()
            chosen_node = self.node_queue.get()[1] #Chosen node to check

            #If the priority of the node was incorrect, add back to the queue with the correct priority.
            if (old_key < self.calculate_key(chosen_node,init)):
                self.insert(chosen_node,self.calculate_key(chosen_node, init)) 

            # If g value is greater then rhs
            elif (self.node_values_list[chosen_node[0]][chosen_node[1]][0] > self.node_values_list[chosen_node[0]][chosen_node[1]][1]):
                self.node_values_list[chosen_node[0]][chosen_node[1]][0] = self.node_values_list[chosen_node[0]][chosen_node[1]][1] #Lower the g value

                #update all surrounding nodes
                if (chosen_node[0] > 0): #above
                    self.update_node([chosen_node[0]-1, chosen_node[1]], init)
                if (chosen_node[0] < len(self.node_values_list)-1): #below
                    self.update_node([chosen_node[0]+1, chosen_node[1]], init)
                if (chosen_node[1] > 0): #left
                    self.update_node([chosen_node[0], chosen_node[1]-1], init)
                if (chosen_node[1] < len(self.node_values_list[0])-1): #right
                    self.update_node([chosen_node[0], chosen_node[1]+1], init)
                if (chosen_node[0] > 0 and chosen_node[1] > 0): #Topleft
                    self.update_node([chosen_node[0]-1, chosen_node[1]-1], init)
                if (chosen_node[0] < (len(self.node_values_list)-1) and chosen_node[1] > 0): #Bottomleft
                    self.update_node([chosen_node[0]+1, chosen_node[1]-1],  init)
                if (chosen_node[0] < (len(self.node_values_list)-1) and chosen_node[1] < len(self.node_values_list[0])-1): #Bottomright    
                    self.update_node([chosen_node[0]+1, chosen_node[1]+1], init)
                if (chosen_node[0] > 0 and chosen_node[1] < len(self.node_values_list[0])-1): #Topright    
                    self.update_node([chosen_node[0]-1, chosen_node[1]+1], init)
            
            #G is lower then rhs
            else: 
                #Set g to infinity
                self.node_values_list[chosen_node[0]][chosen_node[1]][0] = maxsize

                #Update this node
                self.update_node(chosen_node,init)

                #update all the surrounding nodes
                if (chosen_node[0] > 0): #above
                    self.update_node([chosen_node[0]-1, chosen_node[1]], init)
                if (chosen_node[0] < len(self.node_values_list)-1): #below
                    self.update_node([chosen_node[0]+1, chosen_node[1]], init)
                if (chosen_node[1] > 0): #left
                    self.update_node([chosen_node[0], chosen_node[1]-1], init)
                if (chosen_node[1] < len(self.node_values_list[0])-1): #right
                    self.update_node([chosen_node[0], chosen_node[1]+1], init)
                if (chosen_node[0] > 0 and chosen_node[1] > 0): #Topleft
                    self.update_node([chosen_node[0]-1, chosen_node[1]-1], init)
                if (chosen_node[0] < (len(self.node_values_list)-1) and chosen_node[1] > 0): #Bottomleft
                    self.update_node([chosen_node[0]+1, chosen_node[1]-1], init)
                if (chosen_node[0] < (len(self.node_values_list)-1) and chosen_node[1] < len(self.node_values_list[0])-1): #Bottomright    
                    self.update_node([chosen_node[0]+1, chosen_node[1]+1], init)
                if (chosen_node[0] > 0 and chosen_node[1] < len(self.node_values_list[0])-1): #Topright    
                    self.update_node([chosen_node[0]-1, chosen_node[1]+1], init)

            if self.node_queue.qsize() == 0:
                print ("Error: No Path")
                break

    '''
    Create path: This creates a temporary node at the start, and picks the lowest g value (lowest distance to goal) as the next step on the path, adds it to the list, and
    repeats until the goal is reached. All of these points in order are the path.
    '''
    def createPathList(self):

        # if start = goal or map/goal are obstacle
        if (self.current_node[0]==self.goal[0] and self.current_node[1] == self.goal[1] 
            or self.current_map[self.current_node[0]][self.current_node[1]]==1 
            or self.current_map[self.goal[0]][self.goal[1]]==1):
            print ("Error: No path")
            return

        path_node = self.current_node.copy()

        path_list = []

        while (path_node[0] != self.goal[0] or path_node[1] != self.goal[1]): #Until robot reaches self.goal

            if (self.node_values_list[path_node[0]][path_node[1]][0] >= (maxsize)): #If g value of current node is inf
                print ("No known path")
                break

            #Check all surrounding nodes for the lowest g value

            gvals = [] #find smallest g value (closest to self.goal)
            if (path_node[0] > 0): #above
                if (self.current_map[path_node[0]-1][path_node[1]] == 0):
                    gvals.append((self.node_values_list[path_node[0]-1][path_node[1]][0] + 1, [path_node[0]-1, path_node[1]])) 

            if (path_node[0] < len(self.node_values_list)-1): #below
                if (self.current_map[path_node[0]+1][path_node[1]] == 0):
                    gvals.append((self.node_values_list[path_node[0]+1][path_node[1]][0] + 1, [path_node[0]+1, path_node[1]]))

            if (path_node[1] > 0): #left
                if (self.current_map[path_node[0]][path_node[1]-1] == 0):
                    gvals.append((self.node_values_list[path_node[0]][path_node[1]-1][0] + 1, [path_node[0], path_node[1]-1]))

            if (path_node[1] < len(self.node_values_list[0])-1): #right
                if (self.current_map[path_node[0]][path_node[1]+1] == 0):
                    gvals.append((self.node_values_list[path_node[0]][path_node[1]+1][0] + 1, [path_node[0], path_node[1]+1]))

            if (path_node[0] > 0 and path_node[1] > 0): #Topleft
                if (self.current_map[path_node[0]-1][path_node[1]-1] == 0):
                    gvals.append((self.node_values_list[path_node[0]-1][path_node[1]-1][0] + 1.4, [path_node[0]-1, path_node[1]-1]))

            if (path_node[0] < (len(self.node_values_list)-1) and path_node[1] > 0): #Bottomleft
                if (self.current_map[path_node[0]+1][path_node[1]-1] == 0):
                    gvals.append((self.node_values_list[path_node[0]+1][path_node[1]-1][0] + 1.4, [path_node[0]+1, path_node[1]-1]))

            if (path_node[0] < (len(self.node_values_list)-1) and path_node[1] < len(self.node_values_list[0])-1): #Bottomright
                if (self.current_map[path_node[0]+1][path_node[1]+1] == 0):
                    gvals.append((self.node_values_list[path_node[0]+1][path_node[1]+1][0] + 1.4, [path_node[0]+1, path_node[1]+1]))

            if (path_node[0] > 0 and path_node[1] < len(self.node_values_list[0])-1): #Topright
                if (self.current_map[path_node[0]-1][path_node[1]+1] == 0):
                    gvals.append((self.node_values_list[path_node[0]-1][path_node[1]+1][0] + 1.4, [path_node[0]-1, path_node[1]+1]))

        
            if (len(gvals) == 0): #Nowhere to go
                print("Error: No more path")
                input()
                return

            min_val = min(gvals) #pick lowest g value
            path_node = min_val[1]

            if (path_node in path_list): #Doubling back- no more path
                print("Error: Double back")
                input()
                return
            
            path_list.append(path_node)
            gvals.clear()
        
        self.needs_new_path = False

        for i in range(len(path_list)):
            path_list[i] = self.convertToReal(path_list[i])

        return path_list

    '''
    Update and replanning: Should trigger whenever there is a new map.
    Sets affected nodes to update, and calculates new g values (finds new path)
    '''
    def update_replan(self, prev_map):

        #This function runs when map changes

        self.km += (((self.prev_node[0]-self.current_node[0])**2 + (self.prev_node[1]-self.current_node[1])**2)**0.5) 
        #Add to the accumulation value the distance from the last point (of changed map) to the current point
        self.prev_node = self.current_node.copy() #update the prev_node

        for i in range(len(prev_map)):
            for j in range(len(prev_map[i])):
                if (self.current_map[i][j] != prev_map[i][j]): #for all differing values, update it and its surrounding nodes

                    self.update_node([i,j], False)
                    
                    if (i > 0): #above
                        self.update_node([i-1,j], False)
                    if (i < len(self.node_values_list)-1): #below
                        self.update_node([i+1,j], False)
                    if (j > 0): #left
                        self.update_node([i,j-1], False)
                    if (j < len(self.node_values_list[0])-1): #right
                        self.update_node([i,j+1], False)
                    if (i > 0 and j > 0): #Topleft
                        self.update_node([i-1,j-1], False)
                    if (i < (len(self.node_values_list)-1) and j > 0): #Bottomleft
                        self.update_node([i+1,j-1], False)
                    if (i < (len(self.node_values_list)-1) and j < len(self.node_values_list[0])-1): #Bottomright
                        self.update_node([i+1,j+1], False)
                    if (i > 0 and j < len(self.node_values_list[0])-1): #Topright
                        self.update_node([i-1,j+1], False)

            self.find_path(False) #recalculate g values

            self.needs_new_path = True


    #Updates the map with new grid whenever map is changed
    def update_map(self,new_map, x_offset=0, y_offset=0):
        #Agument map to good mode

        #set prev_map
        prev_map = self.current_map.copy()

        new_map = expand_grid(new_map, self.radius)

        self.current_map = new_map
        self.update_replan(prev_map)

        #New path will be published
        # path = self.createPathList()