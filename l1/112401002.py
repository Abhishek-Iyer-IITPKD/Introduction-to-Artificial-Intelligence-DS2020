# Filename: l1.py

from operator import le


class YantraCollector:
    """
    YantraCollector class to solve the yantra collection puzzle.
    The player must collect all yantras sequentially and reach the exit.
    """
    
    def __init__(self, grid):
        """
        Initializes the game with the provided grid.

        Args:
            grid (list of list of str): The grid representing the puzzle.
        """
        self.grid = grid
        self.n = len(grid)
        self.start = self.find_position('P')
        self.exit = None
        self.yantras = self.find_all_yantras()
        self.revealed_yantra = self.find_position('Y1')
        self.collected_yantras = 0
        self.total_frontier_nodes = 0
        self.total_explored_nodes = 0
        
    def find_position(self, symbol):
        """
        Finds the position of a given symbol in the grid.

        Args:
            symbol (str): The symbol to locate.

        Returns:
            tuple or None: The position of the symbol, or None if not found.
        """
        for i in range(self.n):
            for j in range(self.n):
                if self.grid[i][j] == symbol:
                    return (i, j)
        return None

    def find_all_yantras(self):
        """
        Finds and stores the positions of all yantras in the grid.

        Returns:
            dict: A dictionary mapping yantra numbers to their positions.
        """
        positions = {}
        for i in range(self.n):
            for j in range(self.n):
                if self.grid[i][j].startswith('Y'):
                    positions[int(self.grid[i][j][1:])] = (i, j)
                elif self.grid[i][j] == 'E':
                    self.exit = (i, j)
        return positions

    def reveal_next_yantra_or_exit(self):
        """
        Reveals the next yantra in sequence or the exit when all yantras are collected.
        """
        self.collected_yantras += 1
        if self.collected_yantras + 1 in self.yantras:
            self.revealed_yantra = self.yantras[self.collected_yantras + 1]
        elif self.collected_yantras == len(self.yantras):
            self.revealed_yantra = self.exit
        else:
            self.revealed_yantra = None

    def goal_test(self, position):
        """
        Checks if the given position matches the currently revealed yantra or exit.

        Args:
            position (tuple): The current position to check.
        """
        return position == self.revealed_yantra

    def get_neighbors(self, position):
        """
        Generates valid neighboring positions for the given position.

        Args:
            position (tuple): The current position of the player.
        """
        neighbors = []
        # North
        if position[0]>0 and self.grid[position[0]-1][position[1]] != '#' and self.grid[position[0]-1][position[1]] != 'T' and self.grid[position[0]-1][position[1]] != '?':
            neighbors.append((position[0]-1, position[1]))
        # East
        if position[1]<self.n-1 and self.grid[position[0]][position[1]+1] != '#' and self.grid[position[0]][position[1]+1] != 'T' and self.grid[position[0]][position[1]+1] != '?':
            neighbors.append((position[0], position[1]+1))
        # South
        if position[0]<self.n-1 and self.grid[position[0]+1][position[1]] != '#' and self.grid[position[0]+1][position[1]] != 'T' and self.grid[position[0]+1][position[1]] != '?':
            neighbors.append((position[0]+1, position[1]))
        # West
        if position[1]>0 and self.grid[position[0]][position[1]-1] != '#' and self.grid[position[0]][position[1]-1] != 'T' and self.grid[position[0]][position[1]-1] != '?':
            neighbors.append((position[0], position[1]-1))
        return neighbors

    def bfs(self, start, goal):
        """
        Performs Breadth-First Search (BFS) to find the path to the goal.

        Args:
            start (tuple): The starting position.
            goal (tuple): The goal position.
        """
        frontier = [start]
        explored = []
        while True:
            pos = frontier.pop(0)
            explored.append(pos)
            if pos == goal:
                break
            for neighbor in self.get_neighbors(pos):
                if neighbor not in explored and neighbor not in frontier:
                    frontier.append(neighbor)
            if len(frontier) == 0:
                return None, 0, len(explored)
        path = [goal]
        while path[0] != start:
            neighbors = self.get_neighbors(path[0])
            for e in explored:
                if e in neighbors:
                    path.insert(0, e)
                    break
        return path, len(frontier), len(explored)

    def dfs(self, start, goal):
        """
        Performs Depth-First Search (DFS) to find the path to the goal.

        Args:
            start (tuple): The starting position.
            goal (tuple): The goal position.
        """
        frontier = [start]
        explored = []
        while True:
            pos = frontier.pop(0)
            explored.append(pos)
            if pos == goal:
                break
            for neighbor in self.get_neighbors(pos):
                if neighbor not in explored and neighbor not in frontier:
                    frontier.insert(0, neighbor)
            if len(frontier) == 0:
                return None, 0, len(explored)
        path = [goal]
        while path[0] != start:
            neighbors = self.get_neighbors(path[0])
            for e in explored:
                if e in neighbors:
                    path.insert(0, e)
                    break
        return path, len(frontier), len(explored)
            

    def solve(self, strategy):
        """
        Solves the yantra collection puzzle using the specified strategy.

        Args:
            strategy (str): The search strategy (BFS or DFS).
        """
        if strategy == "BFS":
            tPath = [self.start]
            tExplored = 0
            tFrontier = 0
            while True:
                if self.collected_yantras == 0:
                    path, fLen, eLen = self.bfs(self.start, self.revealed_yantra)
                else: 
                    path, fLen, eLen = self.bfs(self.yantras[self.collected_yantras], self.revealed_yantra)
                tExplored += eLen
                tFrontier += fLen
                if path:
                    for i in range(1,len(path)):
                        tPath.append(path[i])
                else:
                    return None, tFrontier, tExplored
                if self.revealed_yantra == self.exit:
                    break
                self.reveal_next_yantra_or_exit()
            return tPath, tFrontier, tExplored
        if strategy == "DFS":
            tPath = [self.start]
            tExplored = 0
            tFrontier = 0
            while True:
                if self.collected_yantras == 0:
                    path, fLen, eLen = self.dfs(self.start, self.revealed_yantra)
                else: 
                    path, fLen, eLen = self.dfs(self.yantras[self.collected_yantras], self.revealed_yantra)
                tExplored += eLen
                tFrontier += fLen
                if path:
                    for i in range(1,len(path)):
                        tPath.append(path[i])
                else:
                    return None, tFrontier, tExplored
                if self.revealed_yantra == self.exit:
                    break
                self.reveal_next_yantra_or_exit()
            return tPath, tFrontier, tExplored

if __name__ == "__main__":
    grid = [
        ['P', '.', '.', '#', 'Y2'],
        ['#', 'T', '.', '#', '.'],
        ['.', '.', 'Y1', '.', '.'],
        ['#', '.', '.', 'T', '.'],
        ['.', '.', '.', '.', 'E']
    ]
    
    # grid = [
    #     ['P', 'Y4', '#', '.', 'Y2'],
    #     ['.', '.', '.', '.', '.'],
    #     ['Y5', '#', 'Y1', '.', '.'],
    #     ['.', '.', '#', '.', '.'],
    #     ['.', 'Y3', '#', '.', 'E']
    # ]

    game = YantraCollector(grid)
    strategy = "BFS"
    # strategy = "DFS"
    solution, total_frontier, total_explored = game.solve(strategy)
    if solution:
        print("Solution Path:", solution)
        print("Total Frontier Nodes:", total_frontier)
        print("Total Explored Nodes:", total_explored)
    else:
        print("No solution found.")
