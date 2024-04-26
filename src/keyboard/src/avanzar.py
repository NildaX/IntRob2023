#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from std_msgs.msg import Int8
from ultralytics import YOLO
import pybboxes as pbx
from std_msgs.msg import Float64
import math
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

VALUE_SECTION=0
VALUE_DISTANCE=0

VALUE_CENTRO=99

VALUE_OBJETOS=[]
VALUE_OBJETOS_DISTANCES=[]
VALUE_OBJETOS_SECTIONS=[]

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

def flag(msg):
    #print("msg",msg.data)
    global VALUE_SECTION
    VALUE_SECTION=msg.data
def flag2(msg):
    #print("msg2",msg.data)
    global VALUE_DISTANCE
    VALUE_DISTANCE=msg.data
def flag3(msg):
    #print("msg2",msg.data)
    global VALUE_CENTRO
    VALUE_CENTRO=msg.data

def flag4(msg):
    #print("msg2",msg.data)
    global VALUE_OBJETOS
    VALUE_OBJETOS=msg.data

def flag5(msg):
    #print("msg2",msg.data)
    global VALUE_OBJETOS_DISTANCES
    VALUE_OBJETOS_DISTANCES=msg.data
def flag6(msg):
    #print("msg2",msg.data)
    global VALUE_OBJETOS_SECTIONS
    VALUE_OBJETOS_SECTIONS=msg.data

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    Ovr = rospy.Subscriber('/camera/section', Int8, flag)
    Dista = rospy.Subscriber('/camera/distance', Float64, flag2)
    #--
    centro = rospy.Subscriber('/camera/distance_centro', Float64, flag3)
    #--
    objetcs_detected = rospy.Subscriber('/camera/objects', Int32MultiArray, flag4)
    objetcs_detected_distances = rospy.Subscriber('/camera/objects_distances', Float32MultiArray, flag5)
    objetcs_detected_sections = rospy.Subscriber('/camera/objects_sections', Int32MultiArray, flag6)

    class_name_dict = {0: 'table', 1: 'chair', 2: 'sofa', 3: 'fridge', 4: 'barbell', 5: 'bed', 6: 'tv', 7: 'ball', 8: 'cabinet', 9: 'glass', 10: 'portrait', 11: 'door', 12: 'shelf', 13: 'wall'}
    class_name_sections = {0: 'CENTER',1:'TOP LEFT',2:'TOP RIGHT',3:'BOTTOM LEFT',4:'BOTTOM RIGHT'}
    class_name_actions = {'a': 'LEFT','w':'FORWARD','s':'STOP','d':'RIGHT','x':'REVERSE'}
    turtlebot3_model = "waffle"

    status = 0
    target_linear_vel   = 0.7 ##cuando va hacia adelante
    target_angular_vel  = -0.33  ##cuando va hacia derecha

    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    key_anterior='w'
    key='x'
    realizar_vuelta=0
    try:
        while not rospy.is_shutdown():
            #rospy.sleep(1)
            #print(VALUE_SECTION)
            #print(VALUE_DISTANCE)
            #print("value centro",VALUE_CENTRO)
            key_anterior=key
            if VALUE_CENTRO>0.7:
                key='w'
            else:
                key='d'
                realizar_vuelta=1
            '''
            if VALUE_SECTION==0:
                if VALUE_DISTANCE>0.5:
                    key='w'
                else:
                    key='x'
            elif VALUE_DISTANCE>0.5:
                    key='w'
            else:
                key='d'
                realizar_vuelta=1
            '''
            '''
            if VALUE_CENTRO>0.5:
                key='w'
            else:
                key='d'
                realizar_vuelta=1
            '''
            '''
                if VALUE_SECTION==1:
                    if math.isnan(VALUE_DISTANCE):
                        key='w'
                    else:
                        if VALUE_DISTANCE>0.7:
                            key='w'
                        else:
                            key='d'
                elif VALUE_SECTION==2 or VALUE_SECTION==4:
                    if math.isnan(VALUE_DISTANCE):
                        key='w'
                    else:
                        if VALUE_DISTANCE>0.7:
                            key='w'
                        else:
                            key='d'
                elif VALUE_SECTION==3 or VALUE_SECTION==5:
                    if math.isnan(VALUE_DISTANCE):
                        key='w'
                    else:
                        if VALUE_DISTANCE>0.7:
                            key='w'
                        else:
                            key='a'
                elif VALUE_SECTION==0:
                    if VALUE_DISTANCE==0:
                        key='x'
                    else:
                        key='w'
                if key_anterior=='w' and key=='x':
                    key='d'
                    realizar_vuelta=1
                    print("realizar vuelta")
                print(key)
            #key='w'
            '''
            if realizar_vuelta==1:
                contador=0
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                twist = Twist()

                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                pub.publish(twist)
                
            else:
                contador=6
                #rospy.sleep(1)
            while contador<7:
                if realizar_vuelta<6:
                    rospy.sleep(1)
                #if key_anterior=='d' or key_anterior=='a' or key_anterior=='x':
                #target_linear_vel   = 0.0
                #control_linear_vel  = 0.0
                #target_angular_vel  = 0.0
                #control_angular_vel = 0.0
                if key == 'w' :
                    #target_linear_vel = 
                    #checkLinearLimitVelocity(target_linear_vel)# + LIN_VEL_STEP_SIZE)
                    target_linear_vel=0.7
                    target_angular_vel=0
                    status = status + 1
                    print('Go ahead')
                elif key == 'x' :
                    #target_linear_vel = 
                    #checkLinearLimitVelocity(target_linear_vel)# - LIN_VEL_STEP_SIZE)
                    target_linear_vel=-0.7
                    target_angular_vel=0
                    status = status + 1
                    print('return')
                elif key == 'a' :
                    #target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    target_linear_vel=0
                    target_angular_vel=0.33
                    status = status + 1
                    print('Left')
                elif key == 'd' :
                    #target_angular_vel = checkAngularLimitVelocity(target_angular_vel- ANG_VEL_STEP_SIZE)
                    #status = status + 1
                    target_linear_vel=0
                    target_angular_vel=-0.33
                    print('Right')
                elif key == ' ' or key == 's' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    print('stop')
                    #print(vels(target_linear_vel, target_angular_vel))
                else:
                    if (key == '\x03'):
                        break

                if status == 20 :
                    #print(msg)
                    status = 0

                twist = Twist()

                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                pub.publish(twist)
                
                contador+=1
            if contador==7:
                contador=0
                if realizar_vuelta==1:
                    realizar_vuelta=0
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    twist = Twist()

                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                    pub.publish(twist)
            for i in range(len(VALUE_OBJETOS)):
                print("The object "+class_name_dict[int(VALUE_OBJETOS[i])]+" is "+class_name_sections[int(VALUE_OBJETOS_SECTIONS[i]-1)]+" with a distance of  "+str(VALUE_OBJETOS_DISTANCES[i])+" m with the action "+class_name_actions[key])
    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
