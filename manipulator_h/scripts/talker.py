#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import String
from manipulator_h_base_module_msgs.msg import *
from math import *

def angles_from_links(l1, l2, l3):  # law of cosines
    l1s = l1**2
    l2s = l2**2
    l3s = l3**2
    #print('Links: [{},{},{}], squared:[{},{},{}]'.format(l1,l2,l3,l1s,l2s,l3s))
    try:
        num = l1s+l2s-l3s
        denum = (2*l1*l2)
        #print('\tacos argument = [ {} / {} ]'.format(num,denum))
        alpha = acos( num / denum)
    except:
        print('Error occured while processing alpha.')
        raise

    try:
        beta = acos( -(l2s-l1s-l3s)/(2*l1*l3))
    except:
        print('Error occured while processing beta.')
        raise
    
    try:
        gamma = acos( -(l1s-l3s-l2s)/(2*l2*l2))
    except:
        print('Error occured while processing gamma')
        raise
    
    return [alpha, beta, gamma]


def talker():
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    joint_pub = rospy.Publisher('robotis/base/joint_pose_msg', JointPose, queue_size = 10)
    pose_pub = rospy.Publisher('robotis/base/kinematics_pose_msg', KinematicsPose, queue_size = 10)
    rospy.init_node('talker', anonymous=True, log_level=rospy.DEBUG)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    d = rospy.Duration(1, 0)
    d2 = rospy.Duration(10, 0)
    i = 0
    
    #pose_msg = KinematicsPose()
    joint_msg = JointPose()
    joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    z = 0.2 / sqrt(2)
    r = 0.2 / sqrt(2)
    l1 = 0.264
    l2 = 0.258
    
    if 1:
        for i in range(0,100):
            t = float(i)/50*pi
            x = 0.2+cos(t)
            y = 0.2+sin(t)
            j1 = atan2(y,x)
            
            r = sqrt(x*x + y*y)
            l = sqrt(r*r + z*z)
            
            sub_alpha = atan2(r,z)
            
            [a,b,c] = angles_from_links(0.2,l1,l2)
            j2 = pi/2-a-sub_alpha
            j3 = pi/2-b
            joint_msg.value = [j1,j2,j3,0,0,0]
            print(joint_msg.value)
            print('i={}\tt={}\tDest coord: {} {} '.format(i,t,x,y))
            rospy.logdebug('i={}\tt={}\tDest coord: {} {} '.format(i,t,x,y))
            joint_pub.publish(joint_msg)
            #rate.sleep()
            rospy.sleep(d)
            
            
        #rate.sleep()
        #rospy.sleep(d2)
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    
    
    
    
        
        #hello_str = "hello world {}".format(rospy.get_time())
        #msg = JointPose()
        #msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        #random.uniform(-pi,pi)
        #msg.value = [random.uniform(-pi,pi),random.uniform(-pi,pi),random.uniform(-pi,pi),random.uniform(-pi,pi),random.uniform(-pi,pi),random.uniform(-pi,pi)]
            
        #print(msg.value)
        ##inp = input("Enter desired orientation: [a,b,c,d,e,f]")
        #list(inp)
        #msg.value = [i,0,0,0,0,0]
        
        #rospy.loginfo(hello_str)
        #rospy.logdebug(hello_str)
        #pub.publish(hello_str)
        #joint_pub.publish(msg)
        
        #print("Moving to {} degrees".format(i))