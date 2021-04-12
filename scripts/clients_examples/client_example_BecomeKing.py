#!/usr/bin/env python2
"""
Client example BecomeKing
test if a checker can be transformed in a king (service test)
Service provided in move_checkers.py
"""

import rospy
from gazebo_checkers.srv import AffectChecker #import custom services

def become_king_client(from_row,from_col):
    rospy.wait_for_service('checkers/become_king/')
    try:
        remove = rospy.ServiceProxy('checkers/become_king/', AffectChecker)
        resp = remove(from_row,from_col)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    #create move (using matrix coordinates)
    from_col=3
    from_row=0

    #use client
    become_king_client(from_row,from_col)