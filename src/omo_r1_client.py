#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
from leg_tracker.srv import Track, TrackResponse

def leg_client():
    rospy.wait_for_service('track')
    try:
        track = rospy.ServiceProxy('track', Track)
        s=input("go or stop: ")
        req2=track(s)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    leg_client()
