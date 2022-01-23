#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
#import cool stuff
import algo
import math


# robot state variables
position_ = Point()
yaw_ = 0
pos_maze_x = -1
pos_maze_y = -1
# laser variables
region_front = 10000
region_fright = 10000
region_fleft = 10000
region_right = 10000
region_left = 10000
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = -1.35#rospy.get_param(0)
desired_position_.y = 1.23#rospy.get_param(0)
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.02
# mapping
maze_size = 16
maze =  [[-1 for i in range(maze_size)] for j in range(maze_size)]
walls = [[0 for i in range(maze_size)] for j in range(maze_size)]

# publishers
pub = None


# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    # print('position_: ', position_)
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

# Printed Lasers
def clbk_laser(msg):
    global region_front, region_fright, region_fleft, region_right, region_left
    regions = {
        'right':  min(min(msg.ranges[0:90]), 10),
        'fright': min(min(msg.ranges[72:143]), 10),
        'front':  min(min(msg.ranges[144:215]), 10),
        'fleft':  min(min(msg.ranges[216:287]), 10),
        'left':   min(min(msg.ranges[270:359]), 10),
    }
    # print()
    # print("Lasers: ", regions['left'],regions['fleft'],regions['front'],regions['fright'],regions['right'])
    region_left = regions['left'] 
    region_fleft = regions['fleft'] 
    region_front = regions['front'] 
    region_fright = regions['fright'] 
    region_right = regions['right'] 
        # print()
    # take_action(regions)
# Printed phi
def find_orientation():
    '''
    down (-pi/4, pi/4)
    '''
    '''
        3 Up (+ y axis)
        2 Down
        0 Right(+ x axis)
        1 Left
    '''
    global yaw_
    yaw = yaw_
    if -math.pi/4 <= yaw <= math.pi/4:
        return 2#'down'
    elif math.pi/4 <= yaw <= 3*math.pi/4:
        return 0#'right'
    elif -3*math.pi/4 <= yaw <= -math.pi/4:
        return 1#'left'
    elif -2*math.pi <= yaw <= -3*math.pi/4 or 3*math.pi/4 <= yaw <= 2*math.pi:
        return 3#'up'
# Printed phi
def update_wall_mapping(x,y):
    global walls, yaw_, region_left, region_fleft, region_front,region_fright, region_right
    mouse_orn = find_orientation()
    '''
        3 Up (+ y axis)
        2 Down
        0 Right(+ x axis)
        1 Left

        w/ reference to +y axis
        -left wall 1
        -down wall 10
        -right wall 100
        -up wall 1000
    '''
    wallf = 0
    walll = 0
    wallr = 0
    state_description = ''
    if region_front < 0.11:
        state_description += ' F'
        if mouse_orn == 0:
            wallf += 1000
        if mouse_orn == 1:
            wallf += 10
        if mouse_orn == 2:
            wallf += 100
        if mouse_orn == 3:
            wallf += 1
    if region_right < 0.11:
        state_description += ' R'
        if mouse_orn == 0:
            wallr += 100
        if mouse_orn == 1:
            wallr += 1
        if mouse_orn == 2:
            wallr += 10
        if mouse_orn == 3:
            wallr += 1000
    if region_left < 0.11:
        state_description += ' L'
        if mouse_orn == 0:
            walll += 1
        if mouse_orn == 1:
            walll += 100
        if mouse_orn == 2:
            walll += 1000
        if mouse_orn == 3:
            walll += 10
    if wallf != 0:
        if((walls[x][y]//wallf)%10) != 1:
                walls[x][y] += wallf
    if walll != 0:
        if((walls[x][y]//walll)%10) != 1:
                walls[x][y] += walll
    if wallr != 0:
        if((walls[x][y]//wallr)%10) != 1:
                walls[x][y] += wallr
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

def change_state(state):
    global state_
    state_ = state
    # print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
# Printed Walla Walla
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - (yaw_ - math.pi/2))
    
    # rospy.loginfo(err_yaw)
    # rospy.loginfo(desired_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        # twist_msg.angular.z = 0
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3
    
    pub.publish(twist_msg)
    # print('Walla Walla')
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        # print('Yaw error: [%s]' % err_yaw)
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - (yaw_ - math.pi/2)
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        # twist_msg.linear.x = 0
        # twist_msg.angular.z = 0
        twist_msg.linear.x = 0.6
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        # print('Yaw error: [%s]' % err_yaw)
        if state_ != 2:
            change_state(0)
# Printed Soup Song
def done():
    print('Soup Song')
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def moveLikeJagger(des_pos):
    print('Desired position MLJ: [%s, %s]' % (des_pos.x, des_pos.y))
    while(state_ != 2): #Here was the logical error - replaced if with while
        # State_Msg = 'State: [%s]' % state_
        # rospy.loginfo(State_Msg)
        if state_ == 0:
            fix_yaw(des_pos)
        elif state_ == 1:
            go_straight_ahead(des_pos)
        else:
            rospy.logerr('Unknown state!')
    else:
        done()
# Printed Thnda, hola

def move_to_maze_position(x,y):
    global position_, state_
    desired_position_maze = x,y
    desired_position_.x, desired_position_.y  = convert_to_global_coordinates(desired_position_maze[0], desired_position_maze[1])
    state_ = 0
    moveLikeJagger(desired_position_)

###REMOVE THIS LATER ON###
import numpy as np
###REMOVE THIS LATER ON###

#explores the maze
def explore():
    global maze, walls, pos_maze_x, pos_maze_y, state_, desired_position_, maze_size
    
    destination_array_maze_coordinates = [[8,8],[8,7],[7,8],[7,7]]
    
    #spawn take spawn position and convert to maze coordinates
    pos_maze_x, pos_maze_y = convert_to_maze_coordinates(position_.x, position_.y)
    # move_to_maze_position(0,0)
    # print("next")
    # move_to_maze_position(6,0)
    
    #spawn done above
    #wall updatw
    update_wall_mapping(pos_maze_x, pos_maze_y)
    #flood fill maze
    algo.mod_flood_fill(maze, walls, destination_array_maze_coordinates)
    print(np.array(maze))
    #for loop
    pos_maze_x, pos_maze_y = convert_to_maze_coordinates(position_.x, position_.y)
    
    x = 0
    y = 15
    while(1):
        #decision from flood fill maze
        #move to decision position
        
        #pseduo decision
        y = y - 1
        print(x,y)
        move_to_maze_position(x,y)
        #update walls
        pos_maze_x, pos_maze_y = convert_to_maze_coordinates(position_.x, position_.y)
        update_wall_mapping(pos_maze_x, pos_maze_y)
        #update flood fill maze
        algo.mod_flood_fill(maze, walls, destination_array_maze_coordinates)
        #delete later one / debug print
        print(np.array(walls))
        print(np.array(maze))
        
        pass
    pass


def main():
    global pub, desired_position_, state_, pos_maze_x, pos_maze_y, maze_size

    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rate = rospy.Rate(20)
    #function acting as dummy
    done()
    while not rospy.is_shutdown():
        #function as dummy
        done()
        #function exploring maze
        explore()    
        print("uwu")
        
        
        pass

if __name__ == '__main__':
    main()