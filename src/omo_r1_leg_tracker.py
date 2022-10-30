#!/usr/bin/env python3
import rospy

from leg_tracker.msg import Person, PersonArray, Leg, LegArray
from geometry_msgs.msg import Twist


# class leg(object):

#     def __init__(self):
#         rospy.Subscriber("/people_tracked", PersonArray, self.callback)

#     def callback(data):
#         # rospy.loginfo(data)
#         n=len(data.people)
#         x=[]
#         y=[]
#         dis=[]
        
#         if cnt[0]==0:
#             mode=0

#         if mode==0:
#             for i in range (0,n):
#                 global id
#                 x.append(data.people[i].pose.position.x)
#                 y.append(data.people[i].pose.position.y)
#                 # rospy.loginfo(x)
#                 dis.append(pow(pow(x[i],2)+pow(y[i],2),0.5))
#                 if i==0:
#                     id=data.people[i].id
#                 elif i>0:
#                     if dis[i]<dis[i-1]:
#                         id=data.people[i].id
#             mode=1
        
#         elif mode==1:
#             rospy.loginfo("hi")
#             for i in range(0,n):
#                 person=0
#                 if data.people[i].id==id:
#                     person=1
#                     c_x=data.people[i].pose.position.x
#                     c_y=data.people[i].pose.position.y
#                     if c_y>=0:
#                         if c_y<=0.1:
#                             cmd(0,0)
#                         else:
#                             cmd(0, 0.1)
#                     else:
#                         if c_y>=-0.1:
#                             cmd(0,0)
#                         else:
#                             cmd(0,-0.1)
#                 if person==0:
#                     mode=0

#         cnt[0]+=1
            

#         rospy.loginfo(id)

#     def cmd(linear, angular):
#         pub=rospy.Publisher("cmd_vel", Twist, queue_size=10)

#         cmd_msg=Twist()
#         cmd_msg.linear.x=linear
#         cmd_msg.linear.y=0
#         cmd_msg.linear.z=0
#         cmd_msg.angular.x=0
#         cmd_msg.angular.y=0
#         cmd_msg.angular.z=angular
#         pub.publish(cmd_msg)

def listener():
    rospy.Subscriber("/people_tracked", PersonArray, callback)
    rospy.spin()
    
def callback(data):
    # rospy.loginfo(data)
    n=len(data.people)
    x=[]
    y=[]
    dis=[]
    
    for i in range (0,n):
        global id
        x.append(data.people[i].pose.position.x)
        y.append(data.people[i].pose.position.y)
        # rospy.loginfo(x)
        dis.append(pow(pow(x[i],2)+pow(y[i],2),0.5))
        if i==0:
            id=data.people[i].id
        elif i>0:
            if dis[i]<dis[i-1]:
                id=data.people[i].id
    
    for i in range(0,n):
        if data.people[i].id==id:
            c_x=data.people[i].pose.position.x
            c_y=data.people[i].pose.position.y
            if c_y>=0:
                if c_y<=0.1:
                    if c_x<=0.5:
                        cmd(0,0)
                    else:
                        cmd(0.1,0)
                else:
                    cmd(0, 0.1)
            else:
                if c_y>=-0.1:
                    if c_x<=0.5:
                        cmd(0,0)
                    else:
                        cmd(0.1,0)
                else:
                    cmd(0,-0.1)                         
    rospy.loginfo(id)

def cmd(linear, angular):
        pub=rospy.Publisher("cmd_vel", Twist, queue_size=10)

        cmd_msg=Twist()
        cmd_msg.linear.x=linear
        cmd_msg.linear.y=0
        cmd_msg.linear.z=0
        cmd_msg.angular.x=0
        cmd_msg.angular.y=0
        cmd_msg.angular.z=angular
        pub.publish(cmd_msg)

if __name__ == '__main__':
    rospy.init_node('leg_controller', anonymous=True)
    listener()
