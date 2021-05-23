#!/usr/bin/env python2
"""
All clients functions
"""
import rospy
from gazebo_checkers.srv import AffectChecker,CellInfo, ComputerMoveChecker, ResetGame

def become_king_client(from_row,from_col):
    rospy.wait_for_service('checkers/become_king/')
    try:
        remove = rospy.ServiceProxy('checkers/become_king/', AffectChecker)
        resp = remove(from_row,from_col)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def cell_info_client(from_row,from_col):
    rospy.wait_for_service('checkers/cell_info/')
    try:
        cell_info = rospy.ServiceProxy('checkers/cell_info/', CellInfo)
        resp = cell_info(from_row,from_col)
        print("Cell "+ resp.cell_name+ " Pose:")
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def move_checkers_client(from_row,from_col,to_row,to_col):
    rospy.wait_for_service('checkers/computer_move/')
    try:
        computer_move = rospy.ServiceProxy('checkers/computer_move/', ComputerMoveChecker)
        resp = computer_move(from_row,from_col,to_row,to_col)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def human_moved_checker_client(from_row,from_col,to_row,to_col):
    rospy.wait_for_service('checkers/human_move/')
    try:
        human_move = rospy.ServiceProxy('checkers/human_move/', ComputerMoveChecker)
        resp = human_move(from_row,from_col,to_row,to_col)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def remove_checker_client(from_row,from_col):
    rospy.wait_for_service('checkers/remove_piece/')
    try:
        remove = rospy.ServiceProxy('checkers/remove_piece/', AffectChecker)
        resp = remove(from_row,from_col)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def reset_game_client():
    rospy.wait_for_service('checkers/reset_game/')
    try:
        reset_game = rospy.ServiceProxy('checkers/reset_game/', ResetGame)
        resp = reset_game()
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

