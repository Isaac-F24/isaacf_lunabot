from Map_maker import map_generate
import math


# Given the grid and radius (in grid units), expands the grid with 2 values
def expand_grid(grid, radius):
    grid2 = grid.copy()

    radius = round(radius)

    for i in range(len(grid)):
        for j in range(len(grid[i])):
            
            if (grid[i][j] == 1):
                leftbound = j - min(radius, j)
                rightbound = j + min(radius, (len(grid[j]) - j - 1))
                topbound = i - min(radius, i)
                bottombound = i + min(radius, (len(grid) - i - 1))

                for k in range(topbound, bottombound+1):
                    for l in range(leftbound, rightbound+1):
                        if (distance([k,l],[i,j]) <= radius) and (grid2[k][l] == 0):
                            grid2[k][l] = 2

    return grid2


def distance(point1, point2):
    return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)


#expand_grid(map_generate(100,100,0.05),2)