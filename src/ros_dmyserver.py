#!/usr/bin/env python

from __future__ import print_function

import rospy
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import actionlib
import tf
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import socket
import select
import math

port = 1890
host = "0.0.0.0"

rospy.init_node('talker')
pub=rospy.Publisher('chatter',String, queue_size=10)
rate = rospy.Rate(10)

listener = tf.TransformListener()

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()
listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(2.0))

speed = rospy.get_param("~speed", 0.5)
turn = rospy.get_param("~turn", 0.5)

backlog = 10
bufsize = 4096

server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
readfds = set([server_sock])

moveBindings = {
        'cmd;Forward':(1,0,0,0),
        # 'o':(1,0,0,-1),
        'cmd;TurnLeft':(0,0,0,1),
        'cmd;TurnRight':(0,0,0,-1),
        # 'u':(1,0,0,1),
        'cmd;Backward':(-1,0,0,0),
        # '.':(-1,0,0,1),
        # 'm':(-1,0,0,-1),
	'cmd;room_m':(0,0,0,0),
        'cmd;room_h':(0,0,0,0),
        'cmd;room_i':(0,0,0,0),
	'cmd;desk_s':(0,0,0,0),
        'cmd;end':(0,0,0,0),
}

msg_init = """
Reading from the teleop-key and Publishing to Twist! -> rover_twist
---------------------------
Moving around:
   forward:    cmd;Forward
   backward:   cmd;Backward
   turn right: cmd;TurnRight
   turn left:  cmd;TurnLeft
   stop:       cmd;end

CTRL-C to quit
"""

room_hagita = [(3.08, 3.88, 0.0),(0.0, 0.0, 0.0, 1.0)]
room_ishiguro = [(3.02, 7.13, 0.0),(0.0, 0.0, 0.0, 1.0)]
room_miyasita = [(3.02, 13.37, 0.0),(0.0, 0.0, 0.0, 1.0)]
desk_sakai = [(0.15, 24.0, 0.0),(0.0, 0.0, 0.0, 1.0)]

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


print(msg_init)
print("[*]Server waiting on port %d" % (port))

try:
    server_sock.bind((host, port))
    server_sock.listen(backlog)

    while True:
        rready, wready, xready = select.select(readfds, [], [])
        for sock in rready:
            if sock is server_sock:
                conn, address = server_sock.accept()
                readfds.add(conn)
            else:
                msg = sock.recv(bufsize)
                if len(msg) == 0:
                    sock.close()
                    readfds.remove(sock)
                else:
                    print(msg)
                    #sock.send(msg)

		    # manual move
                    key = msg
                    if key in moveBindings.keys():
                        x = moveBindings[key][0]
                        y = moveBindings[key][1]
                        z = moveBindings[key][2]
                        th = moveBindings[key][3]

                    if not rospy.is_shutdown():
                        pub.publish(msg)
                        p=rospy.Publisher('rover_twist', Twist, queue_size=10)

 	       	        twist=Twist()

                        twist.linear.x = x * speed
                        twist.linear.y = y * speed
                        twist.linear.z = z * speed
                        twist.angular.x = 0
                        twist.angular.y = 0
                        twist.angular.z = th * turn

                        p.publish(twist)

		    # move to navigation target position 1
		    if(msg == 'cmd;room_h'):
			try:
    			    pub.publish(msg)
		            print("let's go to Prof.Hagita's Room")
	  	            goal = goal_pose(room_hagita)
		            result = client.send_goal(goal)
		            if result:
        	    	        print(result)
		                rospy.loginfo("Goal execution done!")
		    	except rospy.ROSInterruptException:
		            rospy.loginfo("Navigation test finished.")

                    # move to navigation target position 2
                    if(msg == 'cmd;room_i'):
                        try:
                            pub.publish(msg)
                            print("let's go to Prof.Ishiguro's Room")
                            goal = goal_pose(room_ishiguro)
                            result = client.send_goal(goal)
                            if result:
                                print(result)
                                rospy.loginfo("Goal execution done!")
                        except rospy.ROSInterruptException:
                            rospy.loginfo("Navigation test finished.")

                    # move to navigation target position 3
                    if(msg == 'cmd;room_m'):
                        try:
                            pub.publish(msg)
                            print("let's go to Prof.Miyasita's Room")
                            goal = goal_pose(room_miyasita)
                            result = client.send_goal(goal)
                            if result:
                                print(result)
                                rospy.loginfo("Goal execution done!")
                        except rospy.ROSInterruptException:
                            rospy.loginfo("Navigation test finished.")

                    # move to navigation target position 4
                    if(msg == 'cmd;desk_s'):
                        try:
                            pub.publish(msg)
                            print("let's go to Dr.Sakai's Desk")
                            goal = goal_pose(desk_sakai)
                            result = client.send_goal(goal)
                            if result:
                                print(result)
                                rospy.loginfo("Goal execution done!")
                        except rospy.ROSInterruptException:
                            rospy.loginfo("Navigation test finished.")

finally:
    for sock in readfds:
            sock.close()
