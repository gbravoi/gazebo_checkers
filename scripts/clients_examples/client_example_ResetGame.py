#!/usr/bin/env python2
"""
Client example ComputerMovechecker
test if ComputerMovechecker service works.
Service provided in move_checkers.py
"""

import rospy
from gazebo_checkers.srv import ResetGame

def reset_game_client():
    rospy.wait_for_service('checkers/reset_game/')
    try:
        reset_game = rospy.ServiceProxy('checkers/reset_game/', ResetGame)
        resp = reset_game()
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    #use client
    reset_game_client()
