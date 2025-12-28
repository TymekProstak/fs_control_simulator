#!/usr/bin/env python
import rospy
from dv_interfaces.msg import Control

ready_to_launch_inspection_test = False
AS_DRIVING = 3

def inspection_test():
    iterator=0
    steeringAngle_rad=0
    speed=0
    STEER=0
    steeringAngle_step = 0.0175/4

    max_speed = 3

    
    while not rospy.is_shutdown() and iterator<120*4:
        msg = Control()
        if speed<max_speed and iterator%10==0:
            speed=speed+1
        else:
            speed=max_speed
            
        if STEER==0 and steeringAngle_rad<=0.3:
            steeringAngle_rad=steeringAngle_rad+steeringAngle_step
        else:
             STEER=1
             
        if STEER==1 and steeringAngle_rad >=-0.3:
            steeringAngle_rad=steeringAngle_rad-steeringAngle_step
        else:
            STEER=0
        
        msg.speed = speed
        msg.steeringAngle_rad=steeringAngle_rad
        msg.serviceBrake=False
        pub.publish(msg)
        iterator=iterator+1
        rospy.sleep(0.05)

    msg.speed = 0
    msg.steeringAngle_rad=0
    msg.serviceBrake=True
    msg.finished=True
    pub.publish(msg)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('dv_inspection_control_node', anonymous=True)
    pub = rospy.Publisher('/dv_board/control', Control, queue_size=10)

    print("Started inspection!")
    inspection_test()
   

