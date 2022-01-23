'''
an algorithm to make an array like this:
8 7 6 5 4 5 6 7 8
7 6 5 4 3 4 5 6 7
6 5 4 3 2 3 4 5 6
5 4 3 2 1 2 3 4 5
4 3 2 1 0 1 2 3 4
5 4 3 2 1 2 3 4 5
6 5 4 3 2 3 4 5 6
7 6 5 4 3 4 5 6 7
8 7 6 5 4 5 6 7 8
'''

#array of -1's of size 5x5
# size = 10
# maze =  [[-1 for i in range(10)] for j in range(10)]
''' -left wall 1
    -down wall 10
    -right wall 100
    -up wall 1000
'''
# walls = [[0 for i in range(10)] for j in range(10)]
# # wall between 2,2 and 3,2
# walls[2][2] += 10
# walls[3][2] += 1000
# # wall between 3,2 and 3,3
# walls[3][3] += 1
# walls[3][2] += 100

# walls[2][2] += 1
# walls[2][1] += 100

def make_wall(wall_array, cell1, cell2):
    ''' 
    -left wall 1
    -down wall 10
    -right wall 100
    -up wall 1000
    '''
    x1 = cell1[0]
    y1 = cell1[1]
    x2 = cell2[0]
    y2 = cell2[1]
    
    #assume cell1 is the reference point
    #right wall y2-y1 = 1
    if y2-y1 == 1:
        if((wall_array[x2][y2]//1)%10) != 1:
            wall_array[x2][y2] += 1
        if((wall_array[x1][y1]//100)%10) != 1:
            wall_array[x1][y1] += 100

    #left wall y2-y1 = -1
    if y2-y1 == -1:
        if((wall_array[x2][y2]//100)%10) != 1:
            wall_array[x2][y2] += 100
        if((wall_array[x1][y1]//1)%10) != 1:
            wall_array[x1][y1] += 1
    
    #up wall x2-x1 = -1
    if x2-x1 == -1:
        if((wall_array[x2][y2]//10)%10) != 1:
            wall_array[x2][y2] += 10
        if((wall_array[x1][y1]//1000)%10) != 1:
            wall_array[x1][y1] += 1000
    
    #down wall x2-x1 = 1
    if x2-x1 == 1:
        if((wall_array[x2][y2]//1000)%10) != 1:
            wall_array[x2][y2] += 1000
        if((wall_array[x1][y1]//10)%10) != 1:
            wall_array[x1][y1] += 10
    else:
        print("Please enter adjacent cells")
    return wall_array

# make_wall(walls, (6,2), (7,2))
# make_wall(walls, (3,2), (3,3))

# print(maze)
# print()
# print(walls)
# print()

def mod_flood_fill(maze, walls, start_row, start_col):
    '''
    mod_flood_fill function
    '''
    size = len(maze)
    # print(size)
    visited = [[0 for i in range(size)] for j in range(size)]
    
    value = 0
    maze[start_row][start_col] = value
    visited[start_row][start_col] = True
    # maze[start_row+1][start_col] = value
    # visited[start_row+1][start_col] = True
    
    queue = [(start_row, start_col,value)]
    # queue.append((start_row+1, start_col,value))
    while len(queue) > 0:
        row, col, value = queue.pop(0)
        value += 1

        #up case
        i = row - 1
        j = col
        if 0 <= i < size and 0 <= j < size and ((walls[i][j]//10)%10) != 1 and not visited[i][j]:
            if visited[i][j] == False:
                maze[i][j] = value
                visited[i][j] = True
                queue.append((i, j, value))
        #down case
        i = row + 1
        j = col
        
        if 0 <= i < size and 0 <= j < size and ((walls[i][j]//1000)) != 1 and not visited[i][j]:
            if visited[i][j] == False:
                maze[i][j] = value
                visited[i][j] = True
                queue.append((i, j, value))

        #left case
        j = col - 1 
        i = row
        if 0 <= i < size and 0 <= j < size and ((walls[i][j]//100)%10) != 1 and not visited[i][j]:
            if visited[i][j] == False:
                maze[i][j] = value
                visited[i][j] = True
                queue.append((i, j, value))
        #right case
        j = col + 1 
        i = row
        if 0 <= i < size and 0 <= j < size and ((walls[i][j]//1)%10) != 1 and not visited[i][j]:
            if visited[i][j] == False:
                maze[i][j] = value
                visited[i][j] = True
                queue.append((i, j, value))
    return maze
    
# print(mod_flood_fill(maze, walls, 3, 2))