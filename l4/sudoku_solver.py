import sys, os
import pycosat
from collections import defaultdict

def var(row, col, digit):
    return 81*row+9*col+digit

def make_diff(cells):
    for cell in cells:
        for cell1 in cells:
            if cell != cell1:
                if cell[0] < cell1[0] or cell[1] < cell1[1]:
                    for d in range(1,10):
                        clauses.append([-var(cell[0],cell[1],d), -var(cell1[0],cell1[1],d)])

if __name__=='__main__':
    input_file  = sys.argv[1]
    output_file = os.path.join(os.path.dirname(os.path.abspath(input_file)), "output.txt")
    try:
        with open(input_file, 'r') as f:
            with open(output_file, 'w') as f1:
                pass
            grids = f.readlines()
            
            for grid in grids:
                grid = grid.strip()
                grid = grid.replace(" ", "")
                grid = grid.replace("\n", "")
                grid = grid.replace("\t", "")
                
                if len(grid) != 81:
                    print("Error: Invalid grid size")
                    with open(output_file, 'a') as f:
                        f.write("No solution found\n")
                    continue
                
                clauses = []
                
                # Each cell has atleast one value
                for i in range(9):
                    for j in range(9):
                        clauses.append([var(i,j,d) for d in range(1,10)])
                
                # Each cell has atmost one value
                for i in range(9):
                    for j in range(9):
                        for d in range(1,10):
                            for d1 in range(d+1,10):
                                clauses.append([-var(i,j,d), -var(i,j,d1)])

                # Each row and column has all values 1-9
                for i in range(9):
                    for j in range(9):
                        make_diff([[i, j] for j in range(9)])
                        make_diff([[j, i] for j in range(9)])
                
                # Each 3x3 box has all values 1-9
                for i in range(3):
                    for j in range(3):
                        make_diff([[3*i+k%3, 3*j+k//3] for k in range(9)])
                
                # Existing cells
                for i in range(9):
                    for j in range(9):
                        if grid[9*i+j] != '.':
                            clauses.append([var(i,j,int(grid[9*i+j]))])
                
                solution = pycosat.solve(clauses)
                # for i in range(9):
                #     for j in range(9):
                #         for d in range(1,10):
                #             if var(i,j,d) in solution:
                #                 print(d,end="")
                # print()
                        
                if solution == "UNSAT":
                    print("This sudoku is unsolvable")
                    with open(output_file, 'a') as f:
                        f.write("No solution found\n")
                    continue
                        
                if solution == "UNKNOWN":
                    print("This sudoku took too long to solve")
                    with open(output_file, 'a') as f:
                        f.write("No solution found\n")
                    continue
                        
                with open(output_file, 'a') as f:
                    for i in range(9):
                        for j in range(9):
                            for d in range(1,10):
                                if var(i,j,d) in solution:
                                    f.write(str(d))
                    f.write("\n")
    except FileNotFoundError:
        print("Error: The file was not found.")