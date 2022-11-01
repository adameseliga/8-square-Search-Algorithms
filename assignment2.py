import random
from queue import PriorityQueue
from time import time_ns


class Grid:
    ## GOAL: class constant for the goal state
    ## The empty tile is represented by 0 for all Grids
    GOAL = [1, 2, 3, 8, 0, 4, 7, 6, 5]

    def __init__(self, grid):
        ## grid: a 1d array of integers representing the 3x3 grid
        ## possibleStates: list of next possible states which are stored as grid
        ## distance: # of tiles out of place with respect to GOAL
        ## depth: how far a state is from the initial state (None for initial state)
        ## parent: parent of state (None type for initial state)
        self.grid = grid
        self.possibleStates = []
        self.depth = None
        self.parent = None

        distance = 0
        for i, tile in enumerate(self.grid):
            if tile != Grid.GOAL[i]:
                distance += 1

        self.distance = distance

    ## Class operators (implemented for the PriorityQueue class)
    def __gt__(self, other):
        if self.distance > other.distance:
            return True
        return False

    def __lt__(self, other):
        if self.distance < other.distance:
            return True

        return False

    def __eq__(self, other):
        if self.grid == other.grid:
            return True
        return False

    def __ne__(self, other):
        if self.grid != other.grid:
            return True
        return False

    ## String representation of grid
    def __str__(self):
        return str(
            f"|{self.grid[0]}|{self.grid[1]}|{self.grid[2]}|\n|{self.grid[3]}|{self.grid[4]}|{self.grid[5]}|\n|{self.grid[6]}|{self.grid[7]}|{self.grid[8]}|")

    ## Input: self
    ## Output: returns the indices of the tiles adjacent to the empty tile (i.e. 0)
    def neighbor_indices(self):
        ## neighbors: list of indices of neighbors
        ## open_index: index of empty tile
        zero_index = self.getZero()

        neighbors = []
        ## Empty space is the middle tile
        if zero_index == 4:
            neighbors = [zero_index - 3, zero_index + 3, zero_index - 1, zero_index + 1]
            return neighbors
        ## Empty space has right adjacent tile
        if zero_index == 0 or zero_index == 1 or zero_index == 3 or zero_index == 6 or zero_index == 7:
            neighbors.append(zero_index + 1)
        ## Empty space has left adjacent tile
        if zero_index == 1 or zero_index == 2 or zero_index == 5 or zero_index == 7 or zero_index == 8:
            neighbors.append(zero_index - 1)
        ## Empty space has lower adjacent tile
        if zero_index in range(4) or zero_index == 5:
            neighbors.append(zero_index + 3)
        ## ## Empty space has upper adjacent tile
        if zero_index in range(5, 9) or zero_index == 3:
            neighbors.append(zero_index - 3)

        ## Return movable tiles
        return neighbors

    ## swaps two elements in a given state
    ## state: some .grid
    def swap(self, state, a, b):
        ## state: some grid
        ## a, b: positions to swap
        state[a], state[b] = state[b], state[a]  ## swap
        return state  ## return modified state

    ## returns the index of the empty tile (i.e. 0) of some Grid
    def getZero(self):
        return self.grid.index(0)

    ## Input: Grid object
    ## Output: finds next states for some Grid and appends them to possibleStates. returns possibleStates.
    def next_states(self):
        ## neighbors: list of indices of neighbors
        ## open_index: index of empty tile
        zero_index = self.getZero()
        neighbors = self.neighbor_indices()

        for neighbor in neighbors:  ## state: copy of parent state
            state = [tile for tile in self.grid]
            state = self.swap(state, zero_index, neighbor)  ## swap tiles
            self.possibleStates.append(Grid(state))  ## add next state to possibleStates

        return self.possibleStates

    def isGoal(self):  ## check if current state is goal
        if self.grid == Grid.GOAL:
            return True

        return False

    ## Input: Grid Object
    ## Output: the nillson score of some Grid
    def nillson(self):
        clockwise_elements = {0: 1, 1: 2, 2: 5, 5: 8, 8: 7, 7: 6, 6: 3,
                              3: 0}  ## Hashes each index i with index j, where
        score = 0  ## j is clockwise to i.

        if self.grid[4] != 0:
            score += 1
        for i, tile in enumerate(self.grid):
            if tile != 0 and i != 4 and (self.grid[i], self.grid[clockwise_elements[i]]) != (
                    Grid.GOAL[i], Grid.GOAL[clockwise_elements[i]]):
                score += 2
        score *= 3
        score += self.manhatten_distance()

        return score

    ## Returns the manhatten distance of a given Grid with respect to GOAL
    def manhatten_distance(self):
        if self.grid == grid.GOAL:
            return 0

        cost = 0
        for i, tile in enumerate(self.grid):
            if tile != grid.GOAL[i] and tile != 0:
                x1, x2 = int(i / 3), int(grid.GOAL.index(i) / 3)
                y1, y2 = i % 3, grid.GOAL.index(i) % 3

                cost += abs((x1 - x2) + (y1 - y2))

        return cost

    ## Input: Two Grid objects
    ## Output: # of tiles out of place in self with respect to other
    def cost(self, other):
        if self == other:
            return 0

        cost = 0
        for i, tile in enumerate(self.grid):
            if tile != other.grid[i]:
                cost += 1

        return cost

    ## Inputs: Grid object == GOAL, path: empty array
    ## Output: Recurse through each parent, starting at self, until it reaches the initial state.
    ## Appends to path each state along the way
    def findPath(self, path):
        path.append(self)
        if self.parent is not None:
            self.parent.findPath(path)

    ## Recursive function for dfs that searches through each child of some initial state, until it reaches the goal,
    ## or reaches the depth limit.
    ## Outputs path taken and length of visited nodes for a successful search
    ## Otherwise, the method returns False.
    def search(self, visited, LIMIT):
        if self.grid not in visited and not self.isGoal() and LIMIT >= 1:
            visited.append(self.grid)
            LIMIT -= 1
            for child in self.next_states():
                child.parent = self
                temp = child.search(visited, LIMIT)
                if temp:
                    return temp
            return False
        elif self.isGoal():
            path = []
            visited.append(self.grid)
            self.findPath(path)
            return path

        return False

    ## Best First Search
    def bfs(self):
        start_time = time_ns()  ## start time
        states = PriorityQueue()  ## states: priority queue for states
        states.put(self)
        visited = []  ## visited: array of each Grid.grid visited during search

        while not states.empty():  ## While the priority queue is not empty, take the first element
            state = states.get()

            ## If the state has not been visited yet, append each child to states.
            if state.grid not in visited:
                if state.isGoal():  ## If GOAL reached, append state to visited, then find the path taken.
                    visited.append(state.grid)
                    path = []
                    state.findPath(path)
                    return len(visited), path  ## return the length of visited, as well as the path.

                current_time = time_ns()  ## When the search goes on longer than 20s, method times out.
                if current_time - start_time >= 20000000000:
                    return False

                visited.append(state.grid)
                child_states = state.next_states()

                for child in child_states:  ## For each child, make the current state the parent.
                    child.parent = state
                    if child.grid not in visited:  ## Place child in states if child has not been visited yet.
                        states.put(child)

                ## Note: states is sorted with respect to the class operators written above.
                ## Those - in turn - compare two Grid objects with each other's self.distance attribute.

    ## Uniform-Cost Search
    def ucs(self):
        start_time = time_ns()  ## start time
        self.depth = 0  ## initial State has depth zero
        states = PriorityQueue()  ## states: priority queue for states
        states.put((0, self))
        visited = []  ## visited: array of each Grid.grid visited during search

        while not states.empty():
            state = states.get()[1]  ## Grab state at the top of the priority queue

            ## If the state has not been visited yet, append each child to states.
            if state.grid not in visited:

                if state.isGoal():  ## If algorithm reaches the goal state, find the path to initial state.
                    visited.append(state.grid)
                    path = []
                    state.findPath(path)
                    return len(visited), path  ## Return # of nodes visited and the path to the goal

                current_time = time_ns()  ## When the search goes on longer than 20s, the method times out.
                if current_time - start_time >= 20000000000:
                    return False

                visited.append(state.grid)
                child_states = state.next_states()

                for child in child_states:  ## For each child, set child.parent to current state
                    child.parent = state
                    if child.grid not in visited:  ## For each child not visited, place child in priority queue
                        states.put((self.cost(child), child))  ## with the integer returned from cost.

    ## A* search
    def A(self):
        start_time = time_ns()  ## start time
        self.depth = 0  ## initial State has depth zero
        states = PriorityQueue()  ## states: priority queue for states
        states.put((0, self))
        visited = []  ## visited: array of each Grid.grid visited during search

        while not states.empty():
            state = states.get()[1]  ## Grab state at the top of the priority queue

            ## If the state has not been visited yet, append each child to states.
            if state.grid not in visited:
                visited.append(state.grid)
                if state.isGoal():  ## If algorithm reaches the goal state, find the path to initial state.
                    path = []
                    state.findPath(path)
                    return len(visited), path  ## Return # of nodes visited and the path to the goal

                current_time = time_ns()  ## When the search goes on longer than 20s, the method times out.
                if current_time - start_time >= 20000000000:
                    return False

                visited.append(state.grid)
                child_states = state.next_states()

                for child in child_states:  ## For each child, set child.parent to current state
                    child.depth = state.depth + 1
                    child.parent = state
                    child_total_cost = child.depth + child.nillson()  ## total cost: sum of nillson score and child depth
                    if child.grid not in visited:  ## For each child not visited, place child in priority queue
                        states.put((child_total_cost, child))

    ## Depth-First Search (Limited)
    def dfs(self, LIMIT):
        visited = []
        path = self.search(visited, LIMIT)
        if path:  ## If search is successful, return length of visited nodes and the path to goal
            return len(visited), path
        return False  ## Otherwise, return false


def isSolvable(grid):  ## Check if randomly generated grid is solvable
    inversions = 0
    for i, tile in enumerate(grid):  ## for each tile in grid
        for j, otherTile in enumerate(grid):  ## compare tile at i with tile j.
            ## Increment inversions by 1 if and only if:
            ## i != j and neither tile is the empty space
            ## tile at i is greater than the tile at j
            ## i is less than j

            if tile != 0 and otherTile != 0 and tile > otherTile and i < j:
                inversions += 1

    ## The puzzle is solvable if and only if the number of inversions is odd.
    ## Otherwise, the puzzle is not solvable.
    if inversions % 2 == 0:
        return False
    return True


def random_grid():  ## Generate random (solvable) grid
    grid = [0, 1, 2, 3, 4, 5, 6, 7, 8]
    random.shuffle(grid)

    ## While the grid is not a solvable puzzle, shuffle until it grid reaches a solvable state.
    while not isSolvable(grid):
        random.shuffle(grid)

    return grid


if __name__ == '__main__':  ## Driver
    while 1:  ## Sentinel loop

        ## user_input: variable that stores whatever the user inputs
        user_input = 0

        ## Prompt user to change initial configuration or continue
        while user_input != 1:
            print("--------------------------------")
            starting_state = random_grid()  ## Generate random solvable grid
            grid = Grid(starting_state)  ## Instantiate initial state
            print(grid)  ## print initial state
            try:
                user_input = int(input("Press 1 to continue\nPress any other key to generate another initial state  "))
            except ValueError:
                user_input = 0

        ## Reset user_input
        ## set variable "algorithm" to None, dictionary "options" to the choices a user may choose
        ## set limit to 26 (outside the max value)
        user_input = 0
        algorithm = None
        options = {2: grid.ucs, 3: grid.bfs, 4: grid.A}
        limit = 26

        ## Prompt user for algorithm to search with
        print("--------------------------------")
        print("1: Depth-First Search (very slow, very low chance of success)")
        print("2: Uniform Cost Search (slow, low chance of success)")
        print("3: (Greedy) Best-First Search")
        print("4: A* Search")
        while user_input not in [1, 2, 3, 4]:
            try:
                user_input = int(input("Please select an algorithm: "))
            except ValueError:
                user_input = 0

        ## If bfs is selected, ask for depth limit.
        if user_input == 1:
            while limit > 25:
                try:
                    limit = int(input("Please enter a depth limit no greater than 25 for Depth-First Search: "))
                except ValueError:
                    limit = 26
        print("--------------------------------")
        print("Searching...")
        ## Assign algorithm based on user_input
        algorithm = grid.dfs if user_input == 1 else options[user_input]

        ## Take the starting time
        start_time = time_ns()
        algorithm = algorithm() if algorithm != grid.dfs else algorithm(limit)  ## Special case for dfs selection
                                                                            ## (since there must be a limit passed in)
        ## end time
        end_time = time_ns()
        total_time = end_time - start_time ## total time the algorithm ran

        ## If no solution to the algorithm is found in the case of bfs
        if not algorithm and user_input == 1:
            print(f"No solution found, given depth limit {limit}.")

        ## If any other algorithm not bfs timed out
        elif not algorithm:
            print("Algorithm timed out (> 20s)")
        ## Otherwise, the algorithm completed the search.
        ## Print the path taken, as well as the number of nodes visited.
        else:
            print(f"Solution: ")
            for element in algorithm[1][::-1]:
                print(element)
                print()
            print(f"Nodes Visited: {algorithm[0]}")
            print(f"Number of moves: {len(algorithm[1])}")
            print(f"Time taken: {total_time} nanoseconds")
        print("--------------------------------")

        user_input = ""

        while user_input != "y" and user_input != "n":
            user_input = input("New problem? (Y/N):  ")
            user_input.lower()

        if user_input.lower() == "n":
            break
