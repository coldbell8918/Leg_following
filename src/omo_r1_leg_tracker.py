#!/usr/bin/env python3
import rospy

from leg_tracker.msg import Person, PersonArray, Leg, LegArray
from geometry_msgs.msg import Twist
from leg_tracker.srv import Track, TrackResponse

class leg_tracker():
    def __init__(self):
        self.is_active = False
        self.subs = []
        self.pubs = {}
        self.srvs = []
        self.subs.append(rospy.Subscriber("/people_tracked",PersonArray,self.callback))
        self.pubs['cmd_vel'] = rospy.Publisher("cmd_vel", Twist,queue_size=1)
        self.srvs.append(rospy.Service('track', Track, self.handle_track))
        
    def callback(self, data):
        # rospy.loginfo(data)
        if not self.is_active: return
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
                            self.cmd(0,0)
                        else:
                            self.cmd(0.1,0)
                    else:
                        self.cmd(0, 0.1)
                else:
                    if c_y>=-0.1:
                        if c_x<=0.5:
                            self.cmd(0,0)
                        else:
                            self.cmd(0.1,0)
                    else:
                        self.cmd(0,-0.1)                         
        rospy.loginfo(id)

    def cmd(self, linear, angular):
        cmd_msg=Twist()
        cmd_msg.linear.x=linear
        cmd_msg.linear.y=0
        cmd_msg.linear.z=0
        cmd_msg.angular.x=0
        cmd_msg.angular.y=0
        cmd_msg.angular.z=angular
        self.pubs['cmd_vel'].publish(cmd_msg)

    def handle_track(self, req):
        if req.str=='stop':
            rospy.loginfo("stop track")
            self.is_active = False
            return TrackResponse('stop')
        elif req.str=='go':
            self.is_active = True
            return TrackResponse('go')


if __name__ == '__main__':
    rospy.init_node('leg_controller', anonymous=True)
    cls_ = leg_tracker()
    rospy.loginfo("Ready to track.")
    rospy.spin()
