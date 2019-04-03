#! /usr/bin/env python

'''
Author: Marco Antonio Arruda
Source: https://bitbucket.org/theconstructcore/two-wheeled-robot-motion-planning
Modified: Roberto Zegers
'''

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
pub_ = None # defining a global publisher
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
# state variable and state dictionary
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    global regions_
    # read the minimum values of each region
    # a second min is required to filter out 'inf' values, in that case 10 is used 
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
    
    #print 'Laser Callback Call!' # for debugging
    #rospy.logdebug('Laser Callback Call!')
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    ''' obstacle avoidance logic '''
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 0.5 # fine tune distance used to consider a region blocked as by an obstacle
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 1 - no obstacle sensed'
        print state_description # for debugging
        change_state(0) # find wall: turn CW and move ahead
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 2 - obstacle sensed only in front'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 3 - obstacle sensed only in front-right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 4 - obstacle sensed only in front-left'
        print state_description # for debugging
        change_state(0) # find wall: turn CW and move ahead
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 5 - obstacle sensed in front and front-right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 6 - obstacle sensed in front and front-left'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 7 - obstacle sensed in front and front-left and front-right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 8 - obstacle sensed in front-left and front-right'
        print state_description # for debugging
        change_state(3) # move slow straight ahead

# logic block 2:          
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 9 - obstacle sensed only in right'
        print state_description # for debugging
        change_state(2) #follow the wall: keep moving straight ahead
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 10 - obstacle sensed in front and right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 11 - obstacle sensed in front-right and right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 12 - obstacle sensed in front-left and right'
        print state_description # for debugging
        change_state(3) # move slow straight ahead
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 13 - obstacle sensed in front, front-right and right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 14 - obstacle sensed in front, front-left and right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 15 - obstacle sensed in front, front-left, front-right and right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 16 - obstacle sensed in front-left, front-right and right'
        print state_description # for debugging
        change_state(3) # move slow straight ahead

# logic block 3:  
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 17 - obstacle sensed only in left'
        print state_description # for debugging
        change_state(0) # find wall: turn CW and move ahead
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 18 - obstacle sensed in front and left'
        print state_description # for debugging
        change_state(0) # find wall: turn CW and move ahead
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 19 - obstacle sensed in front-right and left'
        print state_description # for debugging
        change_state(3) # move slow straight ahead
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 20 - obstacle sensed in front-left and left'
        print state_description # for debugging
        change_state(0) # find wall: turn CW and move ahead
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 21 - obstacle sensed in front, front-right and left'
        print state_description # for debugging
        change_state(0) # find wall: turn CW and move ahead
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 22 - obstacle sensed in front, front-left and left'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 23 - obstacle sensed in front, front-left, front-right and left'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 24 - obstacle sensed in front-left, front-right and left'
        print state_description # for debugging
        change_state(3) # move slow straight ahead

# logic block 4:  
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 25 - obstacle sensed in left and right'
        print state_description # for debugging
        change_state(3) # move slow straight ahead
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 26 - obstacle sensed in front, left and right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 27 - obstacle sensed in front-right, left and right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 28 - obstacle sensed in front-left, left and right'
        print state_description # for debugging
        change_state(0) # find wall: turn CW and move ahead
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 29 - obstacle sensed in front, front-right, left and right'
        print state_description # for debugging
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 30 - obstacle sensed in front, front-left, left and right'
        print state_description # for debugging
        change_state(0) # find wall: turn CW and move ahead
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 31 - obstacle sensed in front, front-left, front-right, left and right'
        print state_description # for debugging
        change_state(4) # reverse turning left
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 32 - obstacle sensed in front-left, front-right, left and right'
        print state_description # for debugging
        change_state(3) # move slow straight ahead
        
    else:
        state_description = 'unknown case'
        print 'unknown case' # for debugging
        rospy.loginfo(regions)

def find_wall():
    twistmsg = Twist()
    twistmsg.linear.x = 0.2
    twistmsg.angular.z = -0.2 # negative value equals turning CW
    return twistmsg

def drive_straight_ahead():
    twistmsg = Twist()
    twistmsg.linear.x = 0.15 
    return twistmsg

def turn_left():
    twistmsg = Twist()
    twistmsg.angular.z = 0.2 # positive value equals turning CCW
    return twistmsg

def follow_the_wall():
    global regions_
    
    twistmsg = Twist()
    twistmsg.linear.x = 0.25
    return twistmsg

def reverse_left():
    twistmsg = Twist()
    twistmsg.linear.x = -0.2
    twistmsg.angular.z = 0.2 # positive value equals turning CCW
    return twistmsg

def main():
    global pub_, active_ # to use this global variables inside main()
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        #if not active_:
        #    rate.sleep()
        #    continue
        
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
            print 'find wall: turn CW and move ahead'
        elif state_ == 1:
            msg = turn_left()
            print 'turn left'
        elif state_ == 2:
            msg = follow_the_wall()
            print 'follow the wall: keep moving straight ahead'
        elif state_ == 3:
            msg = drive_straight_ahead()
            print 'move slow straight ahead'
        elif state_ == 4:
            msg = reverse_left()
            print 'reverse turning left'
            pass
        
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
