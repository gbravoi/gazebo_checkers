#!/usr/bin/env python2

"""
This code creates markers in the position of the elements in gazebo.
This can be replaced in the duture for information from sensors

note: seems to be slow the refresh rate. how can this be improved?
"""
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker


class Checkers_Marker(object):
    def __init__(self):
        self.board_name='board' #name given in sdf file of checkers_game.
        self.board_size=[0.2921, 0.2921, 0.005] #x,y,z
        self.red_name='red_checker' #name given in sdf file of checkers_game. with numbers from 1 to 12
        self.blue_name='blue_checker' #name given in sdf file of checkers_game.
        self.checker_size=[0.0116, 0.0116, 0.025] #x,y,z; where x=y=radious

        #dictionary with publishers
        self.publishers_dic={}#{name=publisher}
        #disctionary with markers
        self.markers_dic={} #{name=marker}

        #init publishers
        self.marker_initialization()

        #init subscriber to gazebo
        self.gazebo_subscriber=rospy.Subscriber("/gazebo/link_states", LinkStates, self.states_callback)


    def marker_initialization(self):
        """
        Creates publisher for the markers and the markers
        """
        #board publisher
        name=self.board_name
        publisher = rospy.Publisher("checkers/marker/"+name,Marker,queue_size=100)
        self.publishers_dic[name]=publisher
        #create marker and add to dictionary
        self.markers_dic[name]=create_marker("CUBE",self.board_size,[0.5,0.5,0.5])

        #red and blue checkers
        for i in range(1,13):
            #red
            name=self.red_name+"_"+str(i)
            publisher = rospy.Publisher("checkers/marker/"+name,Marker,queue_size=1)
            self.publishers_dic[name]=publisher #add publisher to the dictionary
            self.markers_dic[name]=create_marker("CYLINDER",self.checker_size,[1,0,0]) #add marker to dictionary
            
            #blue
            name=self.blue_name+"_"+str(i)
            publisher = rospy.Publisher("checkers/marker/"+name,Marker,queue_size=1)
            self.publishers_dic[name]=publisher
            self.markers_dic[name]=create_marker("CYLINDER",self.checker_size,[0,0,1]) #add marker to dictionary



    def states_callback(self,data):
        """
        callback function of the gaebo subscriber
        """
        #for loop in all data
        for i in range(len(data.name)):
            #extract name
            s=data.name[i]
            object_name=(s.split('::'))[1].split('::')[0]
            #check if is a piece of the checkers game
            if object_name in self.markers_dic:
                #retrieve marker
                marker=self.markers_dic[object_name]
                #update pose
                object_pose=data.pose[i]
                marker.pose=object_pose
                #publish
                publisher=self.publishers_dic[object_name]
                publisher.publish(marker)
                # print(object_name + "\n\r")

##OTHER FUNCTIONS                

def create_marker(m_type,size,color):
    """
    Function that creates a marker
    m_type: string CYLINDER or CUBE
    size = [x,y,z]
    color=[r,g,b] #number between 0 and 1
    """
    marker=Marker()

    #determine type
    if m_type=="CUBE":
        marker.type=marker.CUBE
    else:
        marker.type=marker.CYLINDER

    marker.header.frame_id = "world" #world frame
    marker.id = 0
    marker.action = marker.ADD #add and modify count as the same
    marker.scale.x = size[0]
    marker.scale.y = size[1]
    marker.scale.z = size[2]
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    #initialize position and orientation on 0, will be corrected when they are loaded
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1

    return marker

#main
if __name__ == '__main__':
    #create node
    rospy.init_node('rviz_board_publisher', anonymous=True)

    #create markers and its publishers
    markers=Checkers_Marker()







    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
