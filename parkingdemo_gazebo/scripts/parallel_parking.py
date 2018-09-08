#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import roslib
import rospy
import numpy as np
import math

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# overall length of the vehicle (meters)
# length = 4.20
length = 4.88

# overall wheel base of the vehicle (meters)
#wheel_base = 2.75
wheel_base = 2.85
# overhang
overh = (length - wheel_base)/2

# overall width
#width = 1.920
width = 1.910

# maximum steering angle (radians)
beta_max = 0.6

# minimum turning radius (meters)
# R_min = overh/math.sin(beta_max)
R_min = 5

# inner turning radius - minimum
Ri_min = np.sqrt(R_min**2 - wheel_base**2) - (width/2)

# outer turning radius - minimum
Re_min = np.sqrt((Ri_min+width)**2 + (wheel_base+overh)**2)

# minimum length of the parking space required - 1 shot
L_min = overh + np.sqrt(Re_min**2 - Ri_min**2)

flag = 0

def main():
    
    # perception data
    L_available = 7
    
    if L_available >= L_min:
        rospy.init_node('parallel_parking', anonymous=True) #make node 
        rospy.Subscriber('ground_truth/state',Odometry,veh_mission)
        rospy.spin()
        
    else:
        print("There isn't enough space for parking!")


def park_points(ri, current, goal, w):
    r_prime = ri + w/2
    c1 = np.array([goal[0], goal[1] + r_prime])

    x_c1 = c1[0]
    y_c1 = c1[1]

    x_i = current[0]
    y_i = current[1]

    y_s = y_i
    y_c2 = y_s - r_prime

    y_t = (y_c1 + y_c2)/2
    x_t = x_c1 + np.sqrt(r_prime**2 - (y_t - y_c1)**2)

    x_s = 2 * x_t - x_c1

    x_c2 = x_s

    c2 = np.array([x_c2, y_c2])
    i = np.array([x_i, y_i])
    s = np.array([x_s, y_s])
    pt = np.array([x_t, y_t])
    #print r_prime, c1, c2, i, s, pt
    return r_prime, c1, c2, i, s, pt


"""
def trajectory(center1, center2, initial, start, transition, goal, r):

    x1 = np.linspace(initial[0], start[0])
    y1 = np.linspace(initial[1], initial[1])

    x2 = np.linspace(start[0], transition[0])
    y2 = np.sqrt(r**2 - (x2 - center2[0])**2) + center2[1]

    x3 = np.linspace(transition[0], goal[0])
    y3 = - np.sqrt(r**2 - (x3 - center1[0])**2) + center1[1]

    x = np.append(x1, [x2, x3])
    y = np.append(y1, [y2, y3])
    plt.plot(x1, y1)
    plt.plot(x2, y2)
    plt.plot(x3, y3)
    #plt.plot(x, y)

    plt.show()
"""        
 

def calc_distance(a, b):
    dist = np.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
    return dist
    
    
def veh_control(initial_pos, start_pos, transition_pos, goal_pos, radius, update_pos):
    
    print initial_pos, start_pos, transition_pos, goal_pos, radius, update_pos

    if update_pos[0]>start_pos[0]:
        vel = -12
        steer = 0
        
    elif update_pos[0]<=(start_pos[0]) and update_pos[0]>(transition_pos[0]):
        vel = -104
        steer = -0.49

    elif update_pos[0]<=(transition_pos[0]) and update_pos[0]>(goal_pos[0]-0.5) and flag == 0:
        steer = 0.49
        vel = -104
    elif update_pos[0]<=goal_pos[0]-0.5:
        steer = 0
        vel = 20
        global flag
        flag = 1
        
    elif flag == 1:
        steer = 0
        vel = -5

    return vel, steer    
    
    
def veh_mission(msg):

    rr_pub = rospy.Publisher('parkingdemo/rr_Wheel_effort_controller/command', Float64, queue_size=10)
    rl_pub = rospy.Publisher('parkingdemo/rl_Wheel_effort_controller/command', Float64, queue_size=10)
    fr_pub = rospy.Publisher('parkingdemo/fr_Steer_position_controller/command', Float64, queue_size=10)
    fl_pub = rospy.Publisher('parkingdemo/fl_Steer_position_controller/command', Float64, queue_size=10)
    
    #while not rospy.is_shutdown():
    rate = rospy.Rate(100) # 100hz
    
    # perception data
    first_obstacle = np.array([20-(length/2+7), 20-(width/2+0.5)])
    goal = np.array([first_obstacle[0], first_obstacle[1] - width/2])
    
    print float(msg.pose.pose.position.x)
    launch_coord = np.array([20, 20])
    current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    
    r_star, center1, center2, initial, start, transition = park_points(Ri_min, launch_coord, goal, width)
    #trajectory(center1, center2, initial, start, transition, goal_pos, r_star)
        
    vel_cmd, steer_cmd = veh_control(initial, start, transition, goal, r_star, current_pos)

    
    rospy.loginfo(vel_cmd)
    rospy.loginfo(steer_cmd)
    #rospy.loginfo(current_pos)
    rr_pub.publish(vel_cmd)
    rl_pub.publish(vel_cmd)
    fr_pub.publish(steer_cmd)
    fl_pub.publish(steer_cmd)
    rate.sleep()

if __name__ == "__main__":
    
    main()    
    
    
