#!/usr/bin/env python
"""
Client example ComputerMovechecker
test if ComputerMovechecker service works.
Service provided in move_checkers.py
"""

import rospy
from gazebo_checkers.srv import ComputerMoveChecker #import custom services

def move_checkers_client(from_row,from_col,to_row,to_col):
    rospy.wait_for_service('checkers/computer_move/')
    try:
        computer_move = rospy.ServiceProxy('checkers/computer_move/', ComputerMoveChecker)
        resp = computer_move(from_row,from_col,to_row,to_col)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    #create move (using matrix coordinates)
    from_col=0
    from_row=7
    to_col=1
    to_row=4

    #use client
    move_checkers_client(from_row,from_col,to_row,to_col)
