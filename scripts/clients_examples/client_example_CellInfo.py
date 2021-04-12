#!/usr/bin/env python2
"""
Client example Cell info
query information about a cell and print it
"""

import rospy
from gazebo_checkers.srv import CellInfo #import custom services
 #import custom services

def cell_info_client(from_row,from_col):
    rospy.wait_for_service('checkers/cell_info/')
    try:
        cell_info = rospy.ServiceProxy('checkers/cell_info/', CellInfo)
        resp = cell_info(from_row,from_col)
        print("Cell "+ resp.cell_name+ " Pose:")
        print(resp.pose)
        print("Available? ")
        print(resp.available)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    #create move (using matrix coordinates)
    from_col=1
    from_row=0

    #use client
    cell_info_client(from_row,from_col)