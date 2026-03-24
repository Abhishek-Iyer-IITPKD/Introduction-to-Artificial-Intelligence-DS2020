import sys, os
import pycosat as pc
from collections import defaultdict
import numpy as np

if __name__=='__main__':
    input_file  = sys.argv[1]
    output_file = os.path.join(os.path.dirname(os.path.abspath(input_file)), "output.txt")
    try:
        with open(input_file, 'r') as f:
            grids = f.readlines()
            
            for grid in grids:
                grid = grid.strip()
                grid = grid.replace(" ", "")
                grid = grid.replace("\n", "")
                grid = grid.replace("\t", "")
                
                if len(grid) != 81:
                    raise Exception("Invalid grid size")
                
                board = np.zeros([9,9]).astype(np.uint8)
                for i in range(9):
                    for j in range(9):
                        try:
                            board[i,j] = int(grid[i*9+j])
                        except ValueError:
                            continue
                
                clauses = []
                
                # Each row
                for i in range(9):
                    for j in range(9):
                        if board[i,j] == 0:
                            clauses.append([board[i,j]])
                            
                # Each column
                for i in range(9):
                    for j in range(9):
                        if board[j,i] == 0:
                            clauses.append([board[j,i]])
                            
                # Each box
                for i in range(3):
                    for j in range(3):
                        for k in range(3):
                            if board[3*i+j,3*k+i] == 0:
                                clauses.append([board[3*i+j,3*k+i]])
                                
                # Each number
                for i in range(1,10):
                    for j in range(1,10):
                        for k in range(1,10):
                            for l in range(1,10):
                                if board[i-1,j-1] == i and board[j-1,k-1] == j and board[k-1,l-1] == k and board[l-1,i-1] == l:
                                    clauses.append([board[i-1,j-1], board[j-1,k-1], board[k-1,l-1], board[l-1,i-1]])
                                    
                clauses.append([-board[i,j] for i in range(9) for j in range(9)])
                
                model = pc.solve()
                solution = np.zeros([9,9]).astype(np.uint8)
                for i in range(9):
                    for j in range(9):
                        solution[i,j] = model.value(board[i,j])
                        
                with open(output_file, 'a') as f:
                    for i in range(9):
                        for j in range(9):
                            f.write(str(solution[i,j]))
                        f.write("\n")
    except FileNotFoundError:
        print("Error: The file was not found.")