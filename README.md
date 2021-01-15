# gazebo_checkers
This repo contain the design of elements in gazebo to play checkers.
This is part of a larger project (in development) where a robot arm play checkers agains a computer.

The repo contain clients that interact with the checkers in gazebo, as a way to provide the "computer movements"

# How to use this package
1. Include in the world "checkers_board" (check as example world/world_checkers.world). It can be placed in any position of the world.
2. Launch in a node move_checkers.py (check as example launch/checkers_sim.launch)
3. (optional) lauch rviz_board_publisher.py to place markers on the board and the checkers on rviz. Note: the update rate is slow..

# move_checkers
This node is in encharge of
1. Create the checkers for both players
2. Put the checkers in initial position
3. Allow interaction with the checkers trogh services

The checker board is represented as a matrix where the rows are equibalent to the "y" direction (numbers) and the columns to the "x" direction (letters). Where the top right elements of the board match the top right elements of the matrix.

Each element of the matrix is a "Cell_info" objects that stores information about the position of the board's cell with respect the world frame and if there is a checker in that cell

![Alt text](readme_imgs/checkers_board.jpg?raw=true "Checkers board")



## move_checkers services:
1. 'checkers/computer_move/': move a piece from a position (from_row,from_col) to a position (to_row,to_column).
2. 'checkers/remove_piece/': Remove a piece from gazebo. Piece is identified by its position on the board (from_row,from_col).
3. 'checkers/become_king/': Tranform a piece to a king. Piece is identified by its position on the board (from_row,from_col)
4. 'checkers/reset_game/': Restart board for a new game (respawn deleted pieces, delete kings, and place pieces in starting position)

Examples of clients are included in the scrip folder.
