#!/usr/bin/env python2
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
import rospkg
import numpy as np
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Twist
from gazebo_checkers.msg import CheckersMove #import custom messages
from gazebo_checkers.srv import ComputerMoveChecker, ComputerMoveCheckerResponse, ResetGame, ResetGameResponse, AffectChecker, AffectCheckerResponse, CellInfo, CellInfoResponse #import custom services
from scipy.spatial.transform import Rotation as R #to peform rotations

class Board(object):
    def __init__(self):
        ##BOARD PROPERTIES
        #origin in center of the board
        self.square_size=0.0325
        self.board_squares_side=8 #board 8x8. number must be even
        self.board_name='Checkers_Board' #name given in sdf file of checkers_game.
        self.board_pose=None #this will be filled by checkers_state_subscriber on the first callback
        #checker info
        self.red_name='red_checker' #name given in sdf file of checkers_game. with numbers from 1 to 12
        self.blue_name='blue_checker'
        self.red_n=10 #encoding number that represent a red checker in board when board described as a matrix
        self.blue_n=20 #encoding number that represent a blue checker in board when board described as a matrix


        ##INFO TO FILL DURING INITIALIZATION
        #matrix with cell objects. cell objects store info like position in real world and checker in it
        self.coordinates_matrix=np.zeros((self.board_squares_side,self.board_squares_side), dtype=object) 
        self.blue_checkers_model_names=[]#array to keep all the names of the chekers, completed on the first callback
        self.red_checkers_model_names=[]

        #VARIABLES TO KEEP TRACK
        self.checkers_in_game=[]#to keep track of all checker in game. This way to know if there are king or deleted checkers in the moment of restart

        
        ##INITIALIZATION FUNCTIONS
        #first wait to get some info of what there is on gazebo
        model_state_message=rospy.wait_for_message("/gazebo/model_states", ModelStates)
        #process gazebo info (get board position)
        self.process_gazebo_info(model_state_message)
        #init board placing checkers on initial position (world frame)
        self.init_board()


        ##START SERVICES
        #start service that move checker positions (when computer is playing)
        self.computer_moves_service=rospy.Service('checkers/computer_move/', ComputerMoveChecker, self.handle_computer_moves)
        #start service to reset the game
        self.reset_game_service=rospy.Service('checkers/reset_game/', ResetGame, self.handle_reset_game)
        #delete a piece
        self.remove_piece_service=rospy.Service('checkers/remove_piece/', AffectChecker, self.handle_remove_checker)
        #change a piece for a king
        self.become_king_service=rospy.Service('checkers/become_king/', AffectChecker, self.handle_become_king)
        #Info about a cell by name. position in real world, and if there is a checker in there (bool) 
        self.cell_info=rospy.Service('checkers/cell_info/', CellInfo, self.handle_cell_info)
    


    def init_board(self):
        """
        -compute board's cell position in world frame
        -creates 12 checkers per side
        -reset board (place checkers in initial position)
        """
        #compute board's cell position
        self.compute_board_coordinates()

        #place cherks on initial position
        self.reset_board()

        

    def spawn_all_checkers(self):
        """
        spawn 12 checkers per side.
        re-spawn deleted checkers.
        """
        #create chekers
        for i in range(12):
            #create a blue checker
            xml_name='blue_checker'
            model_name=xml_name+'_'+str(i)
            if model_name not in self.blue_checkers_model_names:
                self.blue_checkers_model_names.append(model_name)#save name of the model
            if model_name not in self.checkers_in_game:#try to spawn only if the checker is missing
                self.spawn_model(xml_name,model_name)

            #create a red checker
            xml_name='red_checker'
            model_name=xml_name+'_'+str(i)
            if model_name not in self.red_checkers_model_names:
                self.red_checkers_model_names.append(model_name)#save name of the model
            if model_name not in self.checkers_in_game:#try to spawn only if the checker is missing
                self.spawn_model(xml_name,model_name)


    def spawn_model(self,xml_name,model_name,initial_pose=None):
        """
        uses gazebo client to spawn .sdf model
        xml_name: name of the folder (under models)
        model_name: choose the name of the model in gazebo 
        initial_pose: by default in origin
        """
        #spawn by defaul in 0 position
        if initial_pose is None:
            initial_pose=Pose()
            initial_pose.position.x = 0
            initial_pose.position.y = 0
            initial_pose.position.z = 0
            initial_pose.orientation.x = 0
            initial_pose.orientation.y = 0
            initial_pose.orientation.z = 0
            initial_pose.orientation.w = 1

        #other atributes
        robot_namespace=''
        reference_frame='world'
        model_path = rospkg.RosPack().get_path('gazebo_checkers')+'/models/'
        model_xml = ''

        # Spawn the new model #
        with open (model_path + xml_name + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')

        #use client for set new position
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            set_state = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp = set_state( model_name, model_xml,robot_namespace,initial_pose,reference_frame)
            if resp.success:
                self.checkers_in_game.append(model_name)
            return resp.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False



    def reset_board(self):
        """
        place pieces in desired position.
        This will only work if pieces are in gazebo.
        """
        #delete all kings
        self.delete_all_kings()

        #create again pieces that where deleted
        self.spawn_all_checkers()

        #matrix of initial board positions. where 10 is red, 20 is blue
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


        #place red and blue checkers
        red_checker_counter=0
        blue_checker_counter=0
        #go trhough all cell in the board
        for i in range(self.board_squares_side):
            for j in range(self.board_squares_side):
                target_cell=self.coordinates_matrix[i][j]
                #red
                if starting_board[i][j]==self.red_n and red_checker_counter<len(self.red_checkers_model_names): #if we have enought red checkers
                    checker_model_name=self.red_checkers_model_names[red_checker_counter]
                    #move cher using service
                    result=self.set_checkers_position(checker_model_name,target_cell)
                    #increase red_checker_counter
                    red_checker_counter+=1
                #blue
                elif starting_board[i][j]==self.blue_n and blue_checker_counter<len(self.blue_checkers_model_names): #if we have enought blue checkers
                    checker_model_name=self.blue_checkers_model_names[blue_checker_counter]
                    #move checker using service
                    result=self.set_checkers_position(checker_model_name,target_cell)
                    #increase blue_checker_counter
                    blue_checker_counter+=1
                else:
                    #no checker in this cell
                    target_cell.checker_name=None



    def set_checkers_position(self,checker_model_name,target_cell):
        """
        uses gazebo client to set checkers position
        checker: model_name of the checker
        target_cell: cell object, represent a cell in the board
        """

        #check cell is free
        if target_cell.checker_name is None:
            new_model_state=ModelState()
            new_model_state.model_name=checker_model_name
            new_model_state.pose=target_cell.cell_pose# position in real world
            new_model_state.twist=target_cell.get_cell_twist()
            new_model_state.reference_frame='world'
            
            #use client for set new position
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( new_model_state )
                target_cell.checker_name=checker_model_name
                return resp.success

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        else:
            print("there is a checker in goal position "+target_cell.cell_name)
            return False

    def compute_board_coordinates(self):
        """
        Computed during initalization
        Creates a matrix where the elements are of the class "Cell_info" which stores: 
        --coordinates of the cell with respect the center of the board (x,y). x horizontal axis, y vertical axis.
        --the name of the cell, for example "a1" (where a1 is the lower left corner of the board)
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
                self.coordinates_matrix[number,i]=Cell_info(name,x,y,self.board_pose) 
                #increase y coordinate
                y+=self.square_size

            #increment leter coordinate and reset number coordinate
            x+=self.square_size
            y=x_0
        
        #to matrix look like real life need to be flipped
        self.coordinates_matrix=np.flip(self.coordinates_matrix,0)



    def process_gazebo_info(self,data):
        """
        Function called during initalization of the board
        -save the board position (asume board will be fixed in that position all the simulation)
        """
        print("initial info from gazebo received")
        #save the pose of the board in the first callback. This assume the board is fixed and this value wont change
        if self.board_pose is None:
            for i in range(len(data.name)):
                model_name=data.name[i]
                if self.board_name in model_name:#board pose
                    self.board_pose=data.pose[i]
                    print("board_pose")
                    print(self.board_pose)
                


    def delete_all_kings(self):
        """
        function used when restarting the game that delete all kings
        """
        #go though the list of active checkers
        for name in self.checkers_in_game:
            if 'king' in name:
                #use client for delete piece
                rospy.wait_for_service('/gazebo/delete_model')
                try:
                    service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                    resp = service( name )
                    if resp.success:
                        self.checkers_in_game.remove(name)
                    return resp.success

                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                    return False



    #SERVICES CALLBACKS
    def handle_computer_moves(self,req):
        """
        process request to move a checker by the computer
        the computer process board as a matrix (8x8 in the example)
        so the inputs of the service are (from_row,from_col,to_row,to_col)
        """
        #get from_cell information
        from_cell=self.coordinates_matrix[req.from_row][req.from_col]

        #check if there is a checker in the from cell
        checker_model_name=from_cell.checker_name
        if  checker_model_name is not None:
            #get to_cell information
            to_cell=self.coordinates_matrix[req.to_row][req.to_col]
            #move checker using gazebo service
            result=self.set_checkers_position(checker_model_name,to_cell)
            #if checker moved, free cell
            if result:
                from_cell.checker_name=None
            return ComputerMoveCheckerResponse(result)

        else:
            print("there isn't a checker in cell "+from_cell.cell_name)
            return ComputerMoveCheckerResponse(False)

    def handle_reset_game(self,req):
        """
        Service that reset the game
        (checkers to initial position)
        """
        self.init_board()
        return ResetGameResponse(True)


    def handle_remove_checker(self,req):
        """
        Service that remove a piece of the board.
        (make it desapear)
        """
        #get from_cell information
        from_cell=self.coordinates_matrix[req.from_row][req.from_col]

        #get checker name
        checker_model_name=from_cell.checker_name

        if checker_model_name is not None: #if there is a checker
            #use client for delete piece
            rospy.wait_for_service('/gazebo/delete_model')
            try:
                service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                resp = service( checker_model_name )
                if resp.success:
                    self.checkers_in_game.remove(checker_model_name)
                    from_cell.checker_name=None
                return resp.success

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False

        else:
            print("there is not a checker on that position")
            return False


    def handle_become_king(self,req):
        """
        delete piece and spawn a king in the same spot
        """
        #get from_cell information
        from_cell=self.coordinates_matrix[req.from_row][req.from_col]
        #get checker name
        model_name=from_cell.checker_name

        #delete piece
        success=self.handle_remove_checker(req)
        if success:
            #determine color of the king
            if 'red' in model_name:
                xml_name='red_checker_king'
            else: #else blue
                xml_name='blue_checker_king'
            
            #model name, add king word
            model_name+='_king'

            #get pose of the cell
            initial_pose=from_cell.cell_pose
            #spawn piece
            resp=self.spawn_model(xml_name,model_name,initial_pose)
            #add piece to active list
            if resp:
                self.checkers_in_game.append(model_name)
            return resp

        else:
            print("there isnt a checker on cell to become king")
            return False

    def handle_cell_info(self,req):
        """
        Service that return information about a cell of the board
        """
        print("service received")
        cell=self.coordinates_matrix[req.row][req.col]

        resp = CellInfoResponse()
        resp.cell_name=cell.cell_name
        resp.pose= cell.cell_pose
        resp.available=(cell.checker_name is None)
        return resp







class Cell_info(object):
    """
    stores information of a coordinate of the board
    cell name.
    (x,y)=distance from the center of the board to the cell
    """
    def __init__(self,name,x,y,board_pose):
        self.cell_name=name
        self.x=x
        self.y=y
        self.checker_name=None #different from none is there is a checker in this cell. Save gazebo's model name
        #print(name + " x:"+str(x)+" y:"+str(y)+"\n\r")
        #compute cell pose (consider fixed since board doesn't move)
        self.cell_pose=None
        self.compute_cell_pose(board_pose)
    
    def compute_cell_pose(self,board_pose):
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
        cell_pose.orientation=board_pose.orientation
        # cell_pose.orientation.x = 0
        # cell_pose.orientation.y = 0
        # cell_pose.orientation.z = 0
        # cell_pose.orientation.w = 1

        self.cell_pose=cell_pose

    
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



    
