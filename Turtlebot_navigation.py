#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x=0
y=0
z=0
yaw=0

def poseCallback(pose_message):
    global x
    global y, z, yaw
    
    #obtaining the positions with global access
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    global yaw
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    theta0 = yaw
    #converting from degrees to radians
    angular_speed = math.radians(abs(angular_speed_degree))
    
    
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(200) # we publish the velocity at 100 Hz (100 times a second)    
    
    #initial time
    t0 = rospy.Time.now().to_sec() 

    while True :
        if clockwise:
            rospy.loginfo("Turtle rotates clockwise")
        else:
            rospy.loginfo("Turtle rotates anticlockwise")

        #publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)        
        velocity_publisher.publish(vel_msg)
        
        #calculating the current angle
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()
                       
        #check if the angle has reached         
        if  current_angle_degree>relative_angle_degree:
            rospy.loginfo("reached")
            break

    #publish zero velocity to stop 
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def move(speed, distance, direction):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()

    #get current location 
    x0=x
    y0=y
    #z0=z;
    #yaw0=yaw;
    if direction:
        velocity_message.linear.x = speed
    else:
        velocity_message.linear.x = -speed
    distance_moved = 0.0
    loop_rate = rospy.Rate(200) # we publish the velocity at 100 Hz (100 times a second)    
            
    while True :
        if direction:
            rospy.loginfo("Turtlesim moves forwards")
        else:
            rospy.loginfo("Turtlesim moves back")

        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
                    
        #rospy.Duration(1.0)
        #distance formula
        distance_moved = distance_moved+abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print  distance_moved      

        #check if the distance has reached         
        if  not (distance_moved<distance):
            rospy.loginfo("reached distance")
            break
            
    #finally, stop the robot when the goal has been reached
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def linear_velocity(xdot,ydot,xi,yi):
    kv = 0.5
    vdot = kv*abs(math.sqrt(((xdot-xi)**2 + (ydot-yi)**2)))
    return vdot

def steering_angle(xdot,ydot,xi,yi):
    thetadot = math.atan((ydot - yi)/(xdot - xi))
    return thetadot

def proportional_c(thetadot, theta):
    kh = 4.0
    a_vel = kh*(thetadot-theta)
    return a_vel


def goal(destx,desty,tolerance):
    global x, y, z
    global yaw

    #values of destination
    xf = destx + tolerance
    yf = desty + tolerance

    #formulae applied
    #final_lv = linear_velocity(xdot, ydot, x, y)
    #av = steering_angle(xdot, ydot, x, y)
    #final_av = proportional_c(av, theta0)

    #apply these values into the message
    loop_rate = rospy.Rate(100)

    while True :

        goal_message = Twist()
        goal_message.linear.x = linear_velocity(xf,yf,x,y)
        goal_message.linear.y = 0
        goal_message.linear.z = 0
        goal_message.angular.x = 0
        goal_message.angular.y = 0
        goal_message.angular.z = proportional_c(steering_angle(xf,yf,x,y), yaw)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size =10)
        velocity_publisher.publish(goal_message)
        loop_rate.sleep()

        if (linear_velocity(xf,yf,x,y)<0.01):
           rospy.loginfo('reached')
           break
    
    goal_message.linear.x = 0
    goal_message.angular.z = 0
    velocity_publisher.publish(goal_message)


if __name__ == '__main__':


    try:
        #initialise node
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        #declare position subscriber
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)
        
        #final execution : grid mode

        goal(10,10,0)
        #rotate(7,124,1)
        #rotate(9,126,1)
        #rotate(8,128,1) #great one
        rotate(8,136,1)
        #rotate(10,130,1)

        move(5,500,1)
        rotate(8,88,1)

        move(5,30,1)
        rotate(8,88,1)

        move(5,500,1)
        rotate(8,88,0)   

        move(5,30,1)
        rotate(8,88,0)    

        move(5,500,1)
        rotate(8,88,1)  

        move(5,30,1)
        rotate(8,88,1)

        move(5,500,1)
        rotate(8,88,0)

        move(5,30,1)
        rotate(8,88,0)

        move(5,500,1)
        goal(10,10,0)

        #for resetting the turtle
        print 'start reset: '
        rospy.wait_for_service('reset')
        reset_turtle = rospy.ServiceProxy('reset', Empty)
        reset_turtle()
        print 'end reset: '

        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
