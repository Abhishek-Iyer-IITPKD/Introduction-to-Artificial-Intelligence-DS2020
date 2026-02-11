# Filename: l2.py

# Constant costs for yantra and exit cells
YANTRA_COST = 0
EXIT_COST = 0
TRAP_COST = 99999

class YantraCollector:
    """
    YantraCollector class to solve the yantra collection puzzle with cost-based movement.
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
        ## use these variables if needed
        self.collected_yantras = 0
        # self.total_frontier_nodes = 0
        # self.total_explored_nodes = 0
        # self.total_cost = 0
        self.cost_map = self.initialize_cost_map()
        
    def initialize_cost_map(self):
        """
        Initializes a dictionary mapping each position to its movement cost.
        """
        cost_map = {}
        for i in range(self.n):
            for j in range(self.n):
                cell_value = self.grid[i][j]
                if isinstance(cell_value, int):  
                    cost_map[(i, j)] = cell_value
                elif cell_value == 'P':
                    cost_map[(i, j)] = 0
                elif cell_value.startswith('Y'):  
                    cost_map[(i, j)] = YANTRA_COST
                elif cell_value == 'E':  
                    cost_map[(i, j)] = EXIT_COST
                elif cell_value == 'T':
                    cost_map[(i, j)] = TRAP_COST 
                # Walls ('#') are ignored, no need to assign them a cost.
        return cost_map
    
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
                if isinstance(self.grid[i][j], str) and self.grid[i][j].startswith('Y'):
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

        return self.revealed_yantra

    def goal_test(self, position):
        """
        Checks if the given position matches the currently revealed yantra or exit.

        Args:
            position (tuple): The current position to check.

        Returns:
            bool: True if position matches revealed_yantra, False otherwise.
        """
        return position == self.revealed_yantra

    def get_neighbors(self, position):
        """
        Generates valid neighboring positions for the given position.
        Each move has an associated movement cost, with yantras and exit having a fixed cost.

        Args:
            position (tuple): The current position (row, col).

        Returns:
            list: A list of neighboring positions [(row, col), ...]. 
                  should include all valid neighbors in the expected order.
        """
        neighbors = []
        if position[0]>0 and self.grid[position[0]-1][position[1]] != '#':
            neighbors.append((position[0]-1, position[1]))
        if position[1]<self.n-1 and self.grid[position[0]][position[1]+1] != '#':
            neighbors.append((position[0], position[1]+1))
        if position[0]<self.n-1 and self.grid[position[0]+1][position[1]] != '#':
            neighbors.append((position[0]+1, position[1]))
        if position[1]>0 and self.grid[position[0]][position[1]-1] != '#':
            neighbors.append((position[0], position[1]-1))
        return neighbors

    def ucs(self, start, goal):
        """
        Performs Uniform Cost Search (UCS) to find the path to the goal.
        Args:
            start (tuple): The starting position.
            goal (tuple): The goal position.
        Returns:
            tuple: (path, frontier_count, explored_count, path_cost)
                   - path: List of tuples representing the path from start to goal.
                   - frontier_count: Number of nodes in the frontier list at the time of exiting the function.
                   - explored_count: Number of nodes in the explored list at the time of exiting the function.
                   - path_cost: Total cost of the path.
        """
        frontier = [start]
        explored = []
        parents = {}
        cost = {start:0}
        while True:
            pos = frontier.pop(0)
            explored.append(pos)
            if pos == goal:
                break
            for neighbor in self.get_neighbors(pos):
                if neighbor not in explored and neighbor not in frontier:
                    parents[neighbor] = pos
                    cost[neighbor] = cost[parents[neighbor]] + self.cost_map[neighbor]
                    if frontier:
                        for i in range(len(frontier)):
                            if cost[frontier[i]] > cost[neighbor]:
                                frontier.insert(i, neighbor)
                                break
                            else:
                                frontier.append(neighbor)
                    else:
                        frontier.append(neighbor)
            if len(frontier) == 0:
                return None, 0, len(explored), 0
        path = [goal]
        while path[0] != start:
            # neighbors = self.get_neighbors(path[0])
            # for e in explored:
            #     if e in neighbors:
            #         path.insert(0, e)
            #         break
            path.insert(0, parents[path[0]])
        return path, len(frontier), len(explored), cost[goal]

    def heuristic(self, position, goal):
        """
        Defines a heuristic function to estimate the cost from the current position to the goal-not necessarily admissible.

        Args:
            position (tuple): The current position.
            goal (tuple): The goal position.
        Returns:
            int: The estimated cost to the goal.
        """
        return abs(goal[0]-position[0]) + abs(goal[1]-position[1]) # For Testing (Manhattan Distance)
        

    def gbfs(self, start, goal):
        """
        Performs Greedy Best-First Search (GBFS) to find the path to the goal.

        Returns:
            tuple: (path, frontier_count, explored_count, path_cost)
                   - path: List of tuples representing the path from start to goal.
                   - frontier_count: Number of nodes in the frontier list at the time of exiting the function.
                   - explored_count: Number of nodes in the explored list at the time of exiting the function.
                   - path_cost: Total cost of the path.
        """
        frontier = [start]
        explored = []
        parents = {}
        cost = {start:0}
        while True:
            pos = frontier.pop(0)
            explored.append(pos)
            if pos == goal:
                break
            for neighbor in self.get_neighbors(pos):
                if neighbor not in explored and neighbor not in frontier:
                    parents[neighbor] = pos
                    cost[neighbor] = cost[parents[neighbor]] + self.cost_map[neighbor]
                    
                    if frontier:
                        for i in range(len(frontier)):
                            if self.heuristic(frontier[i], goal) > self.heuristic(neighbor, goal):
                                frontier.insert(i, neighbor)
                                break
                            else:
                                frontier.append(neighbor)
                    else:
                        frontier.append(neighbor)
            if len(frontier) == 0:
                return None, 0, len(explored), 0
        path = [goal]
        while path[0] != start:
            # neighbors = self.get_neighbors(path[0])
            # for e in explored:
            #     if e in neighbors:
            #         path.insert(0, e)
            #         break
            path.insert(0, parents[path[0]])
        return path, len(frontier), len(explored), cost[goal]

    def a_star(self, start, goal):
        """
        Performs A* Search to find the optimal path to the goal.

        Returns:
            tuple: (path, frontier_count, explored_count, path_cost)
                   - path: List of tuples representing the path from start to goal.
                   - frontier_count: Number of nodes in the frontier list at the time of exiting the function.
                   - explored_count: Number of nodes in the explored list at the time of exiting the function.
                   - path_cost: Total cost of the path.
        """
        frontier = [start]
        explored = []
        parents = {}
        cost = {start:0}
        while True:
            pos = frontier.pop(0)
            explored.append(pos)
            if pos == goal:
                break
            for neighbor in self.get_neighbors(pos):
                if neighbor not in explored and neighbor not in frontier:
                    parents[neighbor] = pos
                    cost[neighbor] = cost[parents[neighbor]] + self.cost_map[neighbor]
                    
                    if frontier:
                        for i in range(len(frontier)):
                            if (cost[frontier[i]] + self.heuristic(frontier[i], goal)) > (cost[neighbor] + self.heuristic(neighbor, goal)):
                                frontier.insert(i, neighbor)
                                break
                            else:
                                frontier.append(neighbor)
                    else:
                        frontier.append(neighbor)
            if len(frontier) == 0:
                return None, 0, len(explored), 0
        path = [goal]
        while path[0] != start:
            # neighbors = self.get_neighbors(path[0])
            # for e in explored:
            #     if e in neighbors:
            #         path.insert(0, e)
            #         break
            path.insert(0, parents[path[0]])
        return path, len(frontier), len(explored), cost[goal]

    def solve(self, strategy):
        """
        Solves the yantra collection puzzle using the specified strategy.

        Args:
            strategy (str): The search strategy to use ("UCS", "GBFS", "A*").

        Returns:
            tuple: (full_path, total_frontier_nodes, total_explored_nodes, total_cost)
                   - full_path: List of tuples representing the full path to collect all yantras and exit.
                   - total_frontier_nodes: Sum of frontier_count across all searches.
                   - total_explored_nodes: Sum of explored_count across all searches.
                   - total_cost: Total cost of the path.
        """
        total_path = [self.start]
        total_frontier_length = 0
        total_explored_length = 0
        total_cost = 0
        while True:
            if self.collected_yantras == 0:
                if strategy == "UCS":
                    path, frontier_length, explored_length, cost = self.ucs(self.start, self.revealed_yantra)
                elif strategy == "GBFS":
                    path, frontier_length, explored_length, cost = self.gbfs(self.start, self.revealed_yantra)
                elif strategy == "A*":
                    path, frontier_length, explored_length, cost = self.a_star(self.start, self.revealed_yantra)
                else:
                    path, frontier_length, explored_length, cost = None, 0, 0, 0
            else:
                if strategy == "UCS":
                    path, frontier_length, explored_length, cost = self.ucs(self.yantras[self.collected_yantras], self.revealed_yantra)
                elif strategy == "GBFS":
                    path, frontier_length, explored_length, cost = self.gbfs(self.yantras[self.collected_yantras], self.revealed_yantra)
                elif strategy == "A*":
                    path, frontier_length, explored_length, cost = self.a_star(self.yantras[self.collected_yantras], self.revealed_yantra)
                else:
                    path, frontier_length, explored_length, cost = None, 0, 0, 0
            total_explored_length += explored_length
            total_frontier_length += frontier_length
            total_cost += cost
            if path:
                for i in range(1,len(path)):
                    total_path.append(path[i])
            else:
                return None
            if self.revealed_yantra == self.exit:
                break
            self.reveal_next_yantra_or_exit()
        return total_path, total_frontier_length, total_explored_length, total_cost

if __name__ == "__main__":
    grid = [
        ['P', 2  , '#' , 5  , 'Y2'],
        ['T', 2  , 3   , '#', 1   ],
        [0  , 7  , 'Y1', 4  , 2   ],
        ['#', 'T', 2   , 1  , 3   ],
        [1  , 3  , 0   , 2  , 'E' ]
    ]

    game = YantraCollector(grid)
    # strategy = "UCS"
    # strategy = "GBFS"
    strategy = "A*"
    result = game.solve(strategy)
    
    if result:
        full_path, total_frontier_nodes, total_explored_nodes, total_cost = result
        print("Path:", full_path)
        print("Total Frontier Nodes:", total_frontier_nodes)
        print("Total Explored Nodes:", total_explored_nodes)
        print("Total Cost:", total_cost)
    else:
        print("No solution found.")
