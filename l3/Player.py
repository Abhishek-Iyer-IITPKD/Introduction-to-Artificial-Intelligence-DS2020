import numpy as np

MAX_DEPTH = 10
EXPECT_MAX_DEPTH = 2

class AIPlayer:
    def __init__(self, player_number):
        self.player_number = player_number
        self.type = 'ai'
        self.player_string = 'Player {}:ai'.format(player_number)

    def max_value(self, board, depth, alpha, beta):
        if(depth == MAX_DEPTH):
            return self.evaluation_function(board), 0
        
        valid_cols = []
        for col in range(board.shape[1]):
            if 0 in board[:,col]:
                valid_cols.append(col)
        
        if not valid_cols:
            return self.evaluation_function(board), 0
        
        max_val = -10000
        for move in valid_cols:
            new = np.copy(board)
            i = 0
            for i in range(new.shape[0]-1):
                if new[i+1, move] != 0:
                    break
            new[i, move] = self.player_number
            val, _ = self.min_value(new, depth+1, alpha, beta)
            if val > max_val:
                max_val = val
                max_move = move
            alpha = max(alpha, max_val)
            if beta <= alpha:
                break
        return max_val, max_move
    
    def min_value(self, board, depth, alpha, beta):
        if(depth == MAX_DEPTH):
            return self.evaluation_function(board), 0
        
        valid_cols = []
        for col in range(board.shape[1]):
            if 0 in board[:,col]:
                valid_cols.append(col)
        
        if not valid_cols:
            return self.evaluation_function(board), 0
        
        min_val = 10000
        for move in valid_cols:
            new = np.copy(board)
            for i in range(new.shape[0]-1):
                if new[i+1, move] != 0:
                    break
            new[i, move] = 3-self.player_number
            val, _ = self.max_value(new, depth+1, alpha, beta)
            if val < min_val:
                min_val = val
                min_move = move
            beta = min(beta, min_val)
            if beta <= alpha:
                break
        return min_val, min_move
    
    def get_alpha_beta_move(self, board):
        """
        Given the current state of the board, return the next move based on
        the alpha-beta pruning algorithm

        This will play against either itself or a human player

        INPUTS:
        board - a numpy array containing the state of the board using the
                following encoding:
                - the board maintains its same two dimensions
                    - row 0 is the top of the board and so is
                      the last row filled
                - spaces that are unoccupied are marked as 0
                - spaces that are occupied by player 1 have a 1 in them
                - spaces that are occupied by player 2 have a 2 in them

        RETURNS:
        The 0 based index of the column that represents the next move
        """
        alpha = -10000
        beta = 10000
        
        _, best_move =  self.max_value(board, 0, alpha, beta)

        return best_move

    def expectimax_value(self, board, depth):
        if(depth == EXPECT_MAX_DEPTH):
            return self.evaluation_function(board), 0
        
        valid_cols = []
        for col in range(board.shape[1]):
            if 0 in board[:,col]:
                valid_cols.append(col)
        
        if not valid_cols:
            return self.evaluation_function(board), 0
        
        max_val = -10000
        for move in valid_cols:
            new = np.copy(board)
            i = 0
            for i in range(new.shape[0]-1):
                if new[i+1, move] != 0:
                    break
            new[i, move] = self.player_number
            
            new_valid_cols = []
            for col in range(board.shape[1]):
                if 0 in board[:,col]:
                    new_valid_cols.append(col)
            
            val = 0
            for i in new_valid_cols:
                rand = new.copy()
                j = 0
                for j in range(new.shape[0]-1):
                    if new[j+1, move] != 0:
                        break
                rand[j, i] = 3-self.player_number
                rand_val, _ = self.expectimax_value(rand, depth+1)
                val += rand_val / len(new_valid_cols)
            
            if val > max_val:
                max_val = val
                best_move = move
            
        return max_val, best_move
    
    def get_expectimax_move(self, board):
        """
        Given the current state of the board, return the next move based on
        the expectimax algorithm.

        This will play against the random player, who chooses any valid move
        with equal probability

        INPUTS:
        board - a numpy array containing the state of the board using the
                following encoding:
                - the board maintains its same two dimensions
                    - row 0 is the top of the board and so is
                      the last row filled
                - spaces that are unoccupied are marked as 0
                - spaces that are occupied by player 1 have a 1 in them
                - spaces that are occupied by player 2 have a 2 in them

        RETURNS:
        The 0 based index of the column that represents the next move
        """
        
        _, best_move =  self.expectimax_value(board, 0)

        return best_move




    def evaluation_function(self, board):
        """
        Given the current stat of the board, return the scalar value that 
        represents the evaluation function for the current player
       
        INPUTS:
        board - a numpy array containing the state of the board using the
                following encoding:
                - the board maintains its same two dimensions
                    - row 0 is the top of the board and so is
                      the last row filled
                - spaces that are unoccupied are marked as 0
                - spaces that are occupied by player 1 have a 1 in them
                - spaces that are occupied by player 2 have a 2 in them

        RETURNS:
        The utility value for the current board
        """
        
        return 0


class RandomPlayer:
    def __init__(self, player_number):
        self.player_number = player_number
        self.type = 'random'
        self.player_string = 'Player {}:random'.format(player_number)

    def get_move(self, board):
        """
        Given the current board state select a random column from the available
        valid moves.

        INPUTS:
        board - a numpy array containing the state of the board using the
                following encoding:
                - the board maintains its same two dimensions
                    - row 0 is the top of the board and so is
                      the last row filled
                - spaces that are unoccupied are marked as 0
                - spaces that are occupied by player 1 have a 1 in them
                - spaces that are occupied by player 2 have a 2 in them

        RETURNS:
        The 0 based index of the column that represents the next move
        """
        valid_cols = []
        for col in range(board.shape[1]):
            if 0 in board[:,col]:
                valid_cols.append(col)

        return np.random.choice(valid_cols)


class HumanPlayer:
    def __init__(self, player_number):
        self.player_number = player_number
        self.type = 'human'
        self.player_string = 'Player {}:human'.format(player_number)

    def get_move(self, board):
        """
        Given the current board state returns the human input for next move

        INPUTS:
        board - a numpy array containing the state of the board using the
                following encoding:
                - the board maintains its same two dimensions
                    - row 0 is the top of the board and so is
                      the last row filled
                - spaces that are unoccupied are marked as 0
                - spaces that are occupied by player 1 have a 1 in them
                - spaces that are occupied by player 2 have a 2 in them

        RETURNS:
        The 0 based index of the column that represents the next move
        """

        valid_cols = []
        for i, col in enumerate(board.T):
            if 0 in col:
                valid_cols.append(i)

        move = int(input('Enter your move: '))

        while move not in valid_cols:
            print('Column full, choose from:{}'.format(valid_cols))
            move = int(input('Enter your move: '))

        return move

