#!/usr/bin/env python2
"""
Client example ComputerMovechecker
test if ComputerMovechecker service works.
Service provided in move_checkers.py
"""

import rospy
from gazebo_checkers.srv import RemoveChecker #import custom services

def remove_checker_client(from_row,from_col):
    rospy.wait_for_service('checkers/remove_piece/')
    try:
        remove = rospy.ServiceProxy('checkers/remove_piece/', RemoveChecker)
        resp = remove(from_row,from_col)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    #create move (using matrix coordinates)
    from_col=1
    from_row=0

    #use client
    remove_checker_client(from_row,from_col)