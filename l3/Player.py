import numpy as np
# import time

MAX_DEPTH = 5
EXPECT_MAX_DEPTH = 4

class AIPlayer:
    def __init__(self, player_number):
        self.player_number = player_number
        self.type = 'ai'
        self.player_string = 'Player {}:ai'.format(player_number)
        # self.start_time = time.time()
    
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
        # self.start_time = time.time()
        _, best_move = self.max_value(board, depth = 0, alpha = float('-inf'), beta = float('inf'))
        # print(f"AI Player {self.player_number} took {time.time() - self.start_time} seconds to make move")

        return best_move

    def max_for_chance(self, board, depth):
        # If a terminal state has been reached, then return values such that they are ranked on the basis of their depths
        if self.is_terminal(board, 3-self.player_number):
            return -10000 + depth, 0

        valid_cols = self.get_valid_cols(board)
        # If there are no more valid moves left or EXPECT_MAX_DEPTH has been reached, call the evaluation function
        if not valid_cols or depth == EXPECT_MAX_DEPTH:
            return self.evaluation_function(board), 0

        max_val = float('-inf')
        max_move = -1

        for move in valid_cols:
            row = self.fill_next_open_row(board, move, self.player_number)

            val, _ = self.chance_move(board, depth+1)
            
            board[row, move] = 0

            if val > max_val:
                max_val = val
                max_move = move

        return max_val, max_move

    def chance_move(self, board, depth):
        # If a terminal state has been reached, then return values such that they are ranked on the basis of their depths
        if self.is_terminal(board, self.player_number):
            return 10000 - depth, 0

        valid_cols = self.get_valid_cols(board)
        # If there are no more valid moves left or EXPECT_MAX_DEPTH has been reached, call the evaluation function
        if not valid_cols or depth == EXPECT_MAX_DEPTH:
            return self.evaluation_function(board), 0

        val = 0
        chance_move = -1

        for move in valid_cols:
            row = self.fill_next_open_row(board, move, 3-self.player_number)

            max_val, _ = self.max_for_chance(board, depth+1)

            board[row, move] = 0
            
            val += max_val / len(valid_cols)

        return val, chance_move
    
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
        # self.start_time = time.time()
        _, best_move = self.max_for_chance(board, depth = 0)
        # print(f"AI Player took {time.time() - self.start_time} seconds to make move")

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
        if self.is_terminal(board, self.player_number):
            return 10000
        elif self.is_terminal(board, 3-self.player_number):
            return -10000
        
        rows, cols = board.shape

        score = 0

        center_weights = [0, 1, 2, 3, 2, 1, 0]
        for c in range(cols):
            my_pieces = np.count_nonzero(board[:, c] == self.player_number)
            opp_pieces = np.count_nonzero(board[:, c] == 3 - self.player_number)
            score += (my_pieces - opp_pieces) * center_weights[c] * 3

        def score_window(window, r_indices, c_indices):
            """Helper function to score each 1x4 window, with gravity check"""
            score = 0

            my_grounded_scores  = {0:0, 1:1, 2:10, 3:100}
            my_air_scores       = {0:0, 1:0.5, 2:3, 3:10}
            
            opp_grounded_scores = {0:0, 1:1, 2:4, 3:500} 
            opp_air_scores      = {0:0, 1:0.5, 2:3, 3:12}

            my_count  = np.count_nonzero(window ==   self.player_number)
            opp_count = np.count_nonzero(window == 3-self.player_number)
            empty_count = 4 - my_count - opp_count
            
            # --- My (MAX) Scoring ---
            if empty_count == 4 - my_count:
                is_grounded = False
                if my_count > 0:
                    empty_idx = np.where(window == 0)[0][0]
                    empty_r = r_indices[empty_idx]
                    empty_c = c_indices[empty_idx]
                    is_grounded = (empty_r == 5) or (board[empty_r + 1, empty_c] != 0)
                
                if is_grounded:
                    score += my_grounded_scores[my_count]
                else:
                    score += my_air_scores[my_count]

            # --- Opp (MIN) Scoring ---
            if empty_count == 4 - opp_count:
                is_grounded = False
                if opp_count > 0:
                    empty_idx = np.where(window == 0)[0][0]
                    empty_r = r_indices[empty_idx]
                    empty_c = c_indices[empty_idx]
                    is_grounded = (empty_r == 5) or (board[empty_r + 1, empty_c] != 0)

                if is_grounded:
                    score -= opp_grounded_scores[opp_count]
                else:
                    score -= opp_air_scores[opp_count]

            return score


        # Horizontal
        for r in range(rows):
            for c in range(cols - 3):
                window = board[r, c:c+4]
                r_indices = [r, r, r, r]
                c_indices = [c, c+1, c+2, c+3]
                score += score_window(window, r_indices, c_indices)

        # Vertical
        for c in range(cols):
            for r in range(rows - 3):
                window = board[r:r+4, c]
                r_indices = [r, r+1, r+2, r+3]
                c_indices = [c, c, c, c]
                score += score_window(window, r_indices, c_indices)

        # Diagonal down-right
        for r in range(rows - 3):
            for c in range(cols - 3):
                window = np.array([board[r+i, c+i] for i in range(4)])
                r_indices = [r+i for i in range(4)]
                c_indices = [c+i for i in range(4)]
                score += score_window(window, r_indices, c_indices)

        # Diagonal down-left
        for r in range(rows - 3):
            for c in range(3, cols):
                window = np.array([board[r+i, c-i] for i in range(4)])
                r_indices = [r+i for i in range(4)]
                c_indices = [c-i for i in range(4)]
                score += score_window(window, r_indices, c_indices)

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
