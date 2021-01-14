#!/usr/bin/env python
"""
Code contain functions to
-Set in gazebo board in initial state
-Keep track of position of checker in gazebo
---Move automatically checkers (computer movements)
---Reset board for a new game
---Keep track of position of the checker moved by user? (This may not be needed)

Notes:
--When user human, keep track here of board, or send it as parameter (requires modify functions)
"""
import rospy
import numpy as np
from gazebo_msgs.msg import LinkStates, LinkState
from gazebo_msgs.srv import SetLinkState
from geometry_msgs.msg import Pose, Twist
from gazebo_checkers.msg import CheckersMove #import custom messages
from gazebo_checkers.srv import ComputerMoveChecker, ComputerMoveCheckerResponse, ResetGame, ResetGameResponse #import custom services
from scipy.spatial.transform import Rotation as R #to peform rotations

class Board(object):
    def __init__(self):
        ##BOARD PROPERTIES
        #origin in center of the board
        self.square_size=0.0325
        self.board_squares_side=8 #board 8x8. number must be even
        self.board_name='board' #name given in sdf file of checkers_game.
        self.board_pose=None #this will be filled by checkers_state_subscriber on the first callback
        #checker info
        self.red_name='red_checker' #name given in sdf file of checkers_game. with numbers from 1 to 12
        self.blue_name='blue_checker'
        self.red_n=10 #encoding number that represent a red checker in board when board described as a matrix
        self.blue_n=20 #encoding number that represent a blue checker in board when board described as a matrix


        ##INFO TO FILL DURING INITIALIZATION
        #matrix with board coordinates name, and a dictionary with info about the position of that coordinate. (to fill) 
        self.coordinates_matrix_name=np.zeros((self.board_squares_side,self.board_squares_side), dtype=object) 
        self.coordinates_dict={}
        #values that will obtain from topic "/gazebo/link_states"
        self.blue_checkers_link_names=[]#array to keep all the names of the chekers, completed on the first callback
        self.red_checkers_link_names=[]
        self.current_board_matrix=[]

        
        ##INITIALIZATION FUNCTIONS
        #first wait to get some info of what there is on gazebo
        link_state_message=rospy.wait_for_message("/gazebo/link_states", LinkStates)
        #process gazebo info (get board position, name of the checkers objects)
        self.process_gazebo_info(link_state_message)
        #compute coordinates position with respect center of the board
        self.compute_coordinates()
        #init board placing checkers on initial position (world frame)
        self.init_board()
        ##start services
        #start service that move checker positions (when computer is playing)
        self.computer_moves_service=rospy.Service('checkers/computer_move/', ComputerMoveChecker, self.handle_computer_moves)
        #start service to reset the game
        self.reset_game_service=rospy.Service('checkers/reset_game/', ResetGame, self.handle_reset_game)


    def set_checkers_position(self,checker_link_name,target_cell_name):
        """
        uses gazebo client to set checkers position
        checker: link_name of the checker
        target_cell_name: board position, example "a1"
        """

        target_cell=self.coordinates_dict[target_cell_name] #get cell object 
        #check cell is free
        if target_cell.checker_name is None:
            new_link_state=LinkState()
            new_link_state.link_name=checker_link_name
            new_link_state.pose=target_cell.get_cell_pose(self.board_pose)# position in real world
            new_link_state.twist=target_cell.get_cell_twist()
            new_link_state.reference_frame='world'
            
            #use client for set new position
            rospy.wait_for_service('/gazebo/set_link_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
                resp = set_state( new_link_state )
                target_cell.checker_name=checker_link_name
                return resp.success

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("there is a checker in goal position "+target_cell_name)
            return False

    def compute_coordinates(self):
        """
        board coordinates (x,y)=(letter,number)
        where a1 is the lower left corner
        """
        letters=['a','b','c','d','e','f','g','h']#add more if board bigger
        x_0=-1*self.square_size*(self.board_squares_side//2)+ self.square_size/2#position in "a" or "1"
        #start on the lowe left corner
        x=x_0
        y=x_0
        #first loop leters
        for i in range(len(letters)): #letters are x coordinate
            letter=letters[i]
            #then loop number
            for number in range(self.board_squares_side):
                name=letter+str(number+1) #+1 since start counting from 1
                #save info of the cell
                self.coordinates_dict[name]=Cell_info(name,x,y)
                self.coordinates_matrix_name[number,i]=name 
                #increase y coordinate
                y+=self.square_size

            #increment leter coordinate and reset number coordinate
            x+=self.square_size
            y=x_0
        
        #to matrix look like real life need to be flipped
        self.coordinates_matrix_name=np.flip(self.coordinates_matrix_name,0)
        print("board layout: \n\r")
        print(self.coordinates_matrix_name)

    def process_gazebo_info(self,data):
        """
        -save the name of the links of the checkers in the first callback
        -save the board position (asume board will be fixed in that position all the simulation)
        """
        print("initial info from gazebo received")
        #save the name of the links of the checkers in the first callback
        #save the pose of the board in the first callback. This assume the board is fixed and this value wont change
        if self.board_pose is None or len(self.blue_checkers_link_names)==0 or len(self.red_checkers_link_names==0):
            for i in range(len(data.name)):
                link_name=data.name[i]
                if self.board_name in link_name:#board pose
                    self.board_pose=data.pose[i]
                elif self.red_name in link_name:#link names of red checkers
                    self.red_checkers_link_names.append(link_name)
                elif self.blue_name in link_name:#link name of blue checkers
                    self.blue_checkers_link_names.append(link_name)



    def init_board(self):
        """
        place pieces in desired position.
        This will only work if pieces are in gazebo.
        """
        #matrix of initial board. where 10 is red, 20 is blue
        half_size=self.board_squares_side//2
        starting_board = [
            sum([[0, self.blue_n] for _ in range(half_size)], []),
            sum([[self.blue_n, 0] for _ in range(half_size)], []),
            sum([[0,self.blue_n] for _ in range(half_size)], []),
            sum([[0] for _ in range(self.board_squares_side)], []),
            sum([[0] for _ in range(self.board_squares_side)], []),
            sum([[self.red_n, 0] for _ in range(half_size)], []),
            sum([[0, self.red_n] for _ in range(half_size)], []),
            sum([[self.red_n, 0] for _ in range(half_size)], []),
        ]
        #print(starting_board)
        self.current_board_matrix=starting_board

        #place red and blue checkers
        red_checker_counter=0
        blue_checker_counter=0
        #go trhough all cell in the board
        for i in range(self.board_squares_side):
            for j in range(self.board_squares_side):
                target_cell_name=self.coordinates_matrix_name[i][j]
                #red
                if starting_board[i][j]==self.red_n and red_checker_counter<len(self.red_checkers_link_names): #if we have enought red checkers
                    checker_link_name=self.red_checkers_link_names[red_checker_counter]
                    #move cher using service
                    result=self.set_checkers_position(checker_link_name,target_cell_name)
                    #increase red_checker_counter
                    red_checker_counter+=1
                #blue
                elif starting_board[i][j]==self.blue_n and blue_checker_counter<len(self.blue_checkers_link_names): #if we have enought blue checkers
                    checker_link_name=self.blue_checkers_link_names[blue_checker_counter]
                    #move checker using service
                    result=self.set_checkers_position(checker_link_name,target_cell_name)
                    #increase blue_checker_counter
                    blue_checker_counter+=1
                else:
                    #no checker in this cell
                    target_cell=self.coordinates_dict[target_cell_name]
                    target_cell.checker_name=None
                #     print("no enought pieces in gazebo")


    #SERVICES CALLBACKS
    def handle_computer_moves(self,req):
        """
        process request to move a checker by the computer
        the computer process board as a matrix (8x8 in the example)
        so the inputs of the service are (from_row,from_col,to_row,to_col)
        """
        #get from_cell information
        from_cell_name=self.coordinates_matrix_name[req.from_row][req.from_col]
        from_cell=self.coordinates_dict[from_cell_name] #get cell object

        #check if there is a checker in the from cell
        checker_link_name=from_cell.checker_name
        if  checker_link_name is not None:
            #get to_cell information
            to_cell_name=self.coordinates_matrix_name[req.to_row][req.to_col]
            #move checker using gazebo service
            result=self.set_checkers_position(checker_link_name,to_cell_name)
            #if checker moved, free cell
            if result:
                from_cell.checker_name=None
            return ComputerMoveCheckerResponse(result)

        else:
            print("there isn't a checker in cell "+from_cell_name)
            return ComputerMoveCheckerResponse(False)

    def handle_reset_game(self,req):
        """
        Service that reset the game
        (checkers to initial position)
        """
        self.init_board()
        return ResetGameResponse(True)



           







class Cell_info(object):
    """
    stores information of a coordinate of the board
    cell name.
    (x,y)=distance from the center of the board to the cell
    """
    def __init__(self,name,x,y):
        self.cell_name=name
        self.x=x
        self.y=y
        self.checker_name=None #different from none is there is a checker in this cell. Save gazebo's link name
        #print(name + " x:"+str(x)+" y:"+str(y)+"\n\r")
    
    def get_cell_pose(self,board_pose):
        """
        get pose of the cell in world frame
        """
        #board orientation.
        rot=R.from_quat([board_pose.orientation.x, board_pose.orientation.y, board_pose.orientation.z, board_pose.orientation.w])

        #cell position considering rotation
        cell_pos=rot.apply(np.array([self.x,self.y,0]))

        cell_pose=Pose()
        cell_pose.position.x = cell_pos[0]+board_pose.position.x
        cell_pose.position.y = cell_pos[1]+board_pose.position.y
        cell_pose.position.z = cell_pos[2]+board_pose.position.z
        cell_pose.orientation.x = 0
        cell_pose.orientation.y = 0
        cell_pose.orientation.z = 0
        cell_pose.orientation.w = 1

        return cell_pose
    
    def get_cell_twist(self):
        """
        cell twist is 0, board is not moving
        """
        cell_twist=Twist()
        cell_twist.linear.x=0
        cell_twist.linear.y=0
        cell_twist.linear.z=0
        cell_twist.angular.x=0
        cell_twist.angular.y=0
        cell_twist.angular.z=0

        return cell_twist



        
if __name__ == '__main__':
    #create node
    rospy.init_node('move_checkers', anonymous=True)

    #create board object
    board=Board()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



    
