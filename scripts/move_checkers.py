import numpy as np

class Board(object):
    def __init__(self):
        #board properties
        #origin in center of the board
        self.square_size=0.0325
        self.board_squares_side=8 #board 8x8. number must be even

        #initialization functions
        self.compute_coordinates()
        

    def compute_coordinates(self):
        """
        board coordinates (x,y)=(leter,number)
        where a1 is the lower left corner
        """
        self.coordinates_matrix=np.zeros((self.board_squares_side,self.board_squares_side), dtype=object)
        self.coordinates_matrix_name=np.zeros((self.board_squares_side,self.board_squares_side), dtype=object) #to quick access
        letters=['a','b','c','d','e','f','g','h']#add more if board bigger
        x_0=-1*self.square_size*self.board_squares_side/2+ self.square_size/2#position in "a" or "1"
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
                self.coordinates_matrix[i,number]=Cell_info(name,x,y)
                self.coordinates_matrix_name[i,number]=name
                #increase y coordinate
                y+=self.square_size

            #increment leter coordinate
            x+=self.square_size



class Cell_info(object):
    """
    stores information of a coordinate of the board
    cell name.
    (x,y)=distance from the center of the board to the cell
    """
    def __init__(self,name,x,y):
        self.name=name
        self.x=x
        self.y=y
        #print(name + " x:"+str(x)+" y:"+str(y)+"\n\r")


        
if __name__ == '__main__':
    #create board object
    board=Board()
    #check board in matriz form
    print(board.coordinates_matrix_name)

    
