import numpy as np
import time

MAX_DEPTH = 7
EXPECT_MAX_DEPTH = 2

class AIPlayer:
    def __init__(self, player_number):
        self.player_number = player_number
        self.type = 'ai'
        self.player_string = 'Player {}:ai'.format(player_number)
        self.start_time = time.time()
    
    def is_terminal(self, board, player_num):
        """Helper function to check whether a game has reached terminal state"""
        rows, cols = board.shape
        
        # Horizontal
        for r in range(rows):
            for c in range(cols - 3):
                if all([board[r, x] == player_num for x in range(c, c+4)]):
                    return True
                    
        # Vertical
        for c in range(cols):
            for r in range(rows - 3):
                if all([board[x, c] == player_num for x in range(r, r+4)]):
                    return True
                    
        # Diagonal down-right
        for r in range(rows - 3):
            for c in range(cols - 3):
                if all([board[r+x, c+x] == player_num for x in range(4)]):
                    return True
                    
        # Diagonal down-left
        for r in range(rows - 3):
            for c in range(3, cols):
                if all([board[r+x, c-x] == player_num for x in range(4)]):
                    return True
                    
        return False
    
    def get_valid_cols(self, board):
        """Helper function to get the valid columns (non-filled)"""
        cols = board.shape[1]
        center = cols // 2
        
        # Order the available columns like [3, 2, 4, 1, 5, 0, 6]
        ordered_range = [center]
        for i in range(1, center + 1):
            if center - i >= 0:
                ordered_range.append(center - i)
            if center + i < cols:
                ordered_range.append(center + i)

        valid_cols = []
        for col in ordered_range:
            if 0 in board[:,col]:
                valid_cols.append(col)
        return valid_cols
    
    def fill_next_open_row(self, board, col, player_num):
        """Helper Function to get the next available row for a particular column"""
        for r in range(board.shape[0]-1, -1, -1):
            if board[r, col] == 0:
                board[r, col] = player_num
                return r

        return -1
            
    def max_value(self, board, depth, alpha, beta):
        # If a terminal state has been reached, then return values such that they are ranked on the basis of their depths
        if self.is_terminal(board, 3-self.player_number):
            return -10000 + depth, 0

        valid_cols = self.get_valid_cols(board)
        # If there are no more valid moves left or MAX_DEPTH has been reached, call the evaluation function
        if not valid_cols or depth == MAX_DEPTH:
            return self.evaluation_function(board), 0
        
        max_val = float('-inf')
        max_move = -1

        for move in valid_cols:
            row = self.fill_next_open_row(board, move, self.player_number)

            val, _ = self.min_value(board, depth+1, alpha, beta)
            
            board[row, move] = 0
            # print(f"Column {move} score: {val} at depth {depth} by MAX") 
            if val > max_val:
                max_val = val
                max_move = move
                alpha = max(alpha, max_val)

            if beta <= val:
                break

        return max_val, max_move
    
    def min_value(self, board, depth, alpha, beta):
        # If a terminal state has been reached, then return values such that they are ranked on the basis of their depths
        if self.is_terminal(board, self.player_number):
            return 10000 - depth, 0

        valid_cols = self.get_valid_cols(board)
        # If there are no more valid moves left or MAX_DEPTH has been reached, call the evaluation function
        if not valid_cols or depth == MAX_DEPTH:
            return self.evaluation_function(board), 0

        min_val = float('inf')
        min_move = -1

        for move in valid_cols:
            row = self.fill_next_open_row(board, move, 3-self.player_number)

            val, _ = self.max_value(board, depth+1, alpha, beta)

            board[row, move] = 0
            # print(f"Column {move} score: {val} at depth {depth} by MIN") 
            if val < min_val:
                min_val = val
                min_move = move
                beta = min(beta, min_val)

            if val <= alpha:
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
        self.start_time = time.time()
        _, best_move = self.max_value(board, depth = 0, alpha = float('-inf'), beta = float('inf'))
        print(f"AI Player {self.player_number} took {time.time() - self.start_time} seconds to make move")
        return best_move

    def expectimax_value(self, board, depth):
        # If a terminal state has been reached, then return values such that they are ranked on the basis of their depths
        if self.is_terminal(board, self.player_number):
            return 10000 - depth, 0
        elif self.is_terminal(board, 3-self.player_number):
            return -10000 - depth, 0

        valid_cols = self.get_valid_cols(board)
        # If there are no more valid moves left or MAX_DEPTH has been reached, call the evaluation function
        if not valid_cols or depth == EXPECT_MAX_DEPTH:
            return self.evaluation_function(board), 0

        max_val = float('-inf')
        best_move = -1

        for move in valid_cols:
            row_max = self.fill_next_open_row(board, move, self.player_number)

            new_valid_cols = self.get_valid_cols(board)
            val = 0

            for i in new_valid_cols:
                row_chance = self.fill_next_open_row(board, i, 3-self.player_number)
                rand_val, _ = self.expectimax_value(board, depth+1)
                val += rand_val / len(new_valid_cols)
                board[row_chance, i] = 0

            board[row_max, move] = 0

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
        _, best_move = self.expectimax_value(board, depth = 0)

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
        # if self.is_terminal(board, self.player_number):
        #     return 10000
        # elif self.is_terminal(board, 3-self.player_number):
        #     return -10000
        
        rows, cols = board.shape

        score = 0

        # Prioritize Center Columns -- Was leading to a tie at depth 5, works fine for depth 6
        # center_col = board[:, int(cols/2)]
        # score += (np.count_nonzero(center_col == self.player_number) * 3)
        # score -= (np.count_nonzero(center_col == 3-self.player_number) * 3)

        def score_window(window):
            """Helper function to score each 1x4 window/block"""
            score = 0

            my_scores  = {0:0, 1:0.5, 2:3, 3:10}
            opp_scores = {0:0, 1:0.5, 2:2, 3:40}

            my_count  = np.count_nonzero(window ==   self.player_number)
            opp_count = np.count_nonzero(window == 3-self.player_number)
            empty_count = 4 - my_count - opp_count
            
            # My (MAX) Scoring
            score += my_scores[my_count] if empty_count == 4 - my_count else 0

            # Opp (MIN) Scoring
            score -= opp_scores[opp_count] if empty_count == 4 - opp_count else 0

            return score

        # Horizontal
        for r in range(rows):
            row_arr = board[r, :]
            for c in range(cols - 3):
                score+=score_window(row_arr[c:c+4])

        # Vertical
        for c in range(cols):
            col_array = board[:, c]
            for r in range(rows - 3):
                score+=score_window(col_array[r:r+4])

        # Diagonal down-right
        for r in range(rows - 3):
            for c in range(cols - 3):
                score+=score_window([board[r+i, c+i] for i in range(4)])

        # Diagonal down-left
        for r in range(rows - 3):
            for c in range(3, cols):
                score+=score_window([board[r+i, c-i] for i in range(4)])

        return score


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