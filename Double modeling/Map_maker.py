import random
import math

def make_map(rows,columns):
    mapString  = ""
    for i in range(rows):
        mapString += "["
        for i in range (columns - 1):
            if (random.randint(1,10) <= 3):
                mapString += "1, "
            else:
                mapString += "0, "
        if (random.randint(1,10) <= 3):
            mapString += "1],\n"
        else:
            mapString += "0],\n"

    return mapString

def map_generate(w,h,d):
    grid = []
    density = d

    width = w
    height = h

    for i in range(height):
        row = []
        for j in range(width):
            if random.random() < density:
                row.append(1)
            else:
                row.append(0)
        grid.append(row)
    return grid

#Generates random rocks to be used as an environment based on radius, and x/y position based on % of map
def rocks(num): 
    rocks = []
    
    maxradius = 0.3
    buffer = 0.1

    #generate tuple (x,y,rad) # % of arena
    for i in range(num):

        rockIsValid = False

        counter = 0
        while (not rockIsValid):
            xpos = random.random()
            ypos = random.random()

            radius = random.uniform(0.01, maxradius)

            rockIsValid = True

            for rockTuple in rocks:
                rockX = rockTuple[0]
                rockY = rockTuple[1]
                rockRadius = rockTuple[2]

                dist = math.sqrt( (abs(xpos-rockX))**2 + (abs(ypos-rockY))**2 )

                if (dist <= rockRadius + radius + buffer):
                    rockIsValid = False
            
            counter+=1

        rocks.append((xpos,ypos,radius))

    return rocks

#Given a list of rocks, creates an associated grid for dstar
def rocksToMap(rocks, width, height):
    arena = []

    for i in range(height):
        row = []
        for j in range(width):
            row.append(0)
        arena.append(row)

    for i in range(len(arena)):
        for j in range(len(arena[0])):
            for rockTuple in rocks:
                rockX = rockTuple[0]
                rockY = rockTuple[1]
                rockRadius = rockTuple[2]

                gridx = width * rockX
                gridy = height * rockY
                gridRad = max(width,height) * rockRadius

                dist = math.sqrt( (abs(gridx-j))**2 + (abs(gridy-i))**2 )
                
                if (dist <= gridRad):
                    arena[i][j] = 1

    return arena



