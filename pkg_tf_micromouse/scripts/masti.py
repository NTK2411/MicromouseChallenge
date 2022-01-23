import math
# def normalize_angle(angle):
#     if(math.fabs(angle) > math.pi):
#         angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
#     return angle

# print(math.fabs(normalize_angle(6.267681444461686)))



def initial_position():
    global  maze_size
    '''
        find quadrant of a point
    '''
    # print("Thnda", position_)
    x = -1.351300
    y = 1.233343
    size = 16
    quad = -1
    posi_x = -1
    posi_y = -1
    if x > 0 and y > 0:
        quad = 1
    elif x < 0 and y > 0:
        quad = 2
    elif x < 0 and y < 0:
        quad = 3
    elif x > 0 and y < 0:
        quad = 4

    if quad == 1:
        posi_x = size - 1
        posi_y = 0
    if quad == 2:
        posi_x = 0
        posi_y = 0
    if quad == 3:
        posi_x = 0
        posi_y = size - 1
    if quad == 4:
        posi_x = size - 1
        posi_y = size - 1
    print('Hola', quad, x, y)
    print('Hola2', posi_x, posi_y)
    return posi_x, posi_y

#function to convert global coordinates to maze coordinates
def convert_to_maze_coordinates(x,y):
    maze_box_size = 0.18
    maze_start = -1.35
    maze_x = int((x - (maze_start)) / maze_box_size)
    maze_y = int((y - (maze_start)) / maze_box_size)
    
    return maze_x, maze_y

#function to convert maze coordinates to global coordinates
def convert_to_global_coordinates(x,y):
    maze_box_size = 0.18
    maze_start = -1.35
    global_x = maze_start + maze_box_size * x
    global_y = maze_start + maze_box_size * y
    
    return global_x, global_y

if __name__ == '__main__':
    # initial_position()
    print(convert_to_maze_coordinates(-1.374552,1.23))
    print(convert_to_global_coordinates(0,0))