import random
import matplotlib.pyplot as plt
import time
import tracemalloc

class Grid:
    def __init__(self, rows, cols):
        self.rows = rows  
        self.cols = cols  
        self.grid = self.create_grid()  
        self.start, self.goal = self.set_start_goal()  
        self.generate_ramp()  

    def create_grid(self):
        
        random.seed(42)
        grid = [[1 if random.random() < 0.15 else 0 for _ in range(self.cols)] for _ in range(self.rows)]
        return grid

    def set_start_goal(self):
        
        start = (5, 5)
        goal = (19, 19)
        return start, goal

    def generate_ramp(self):
       
        random.seed(42)
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 0 and random.random() < 0.1:
                    self.grid[r][c] = 2

    def display_grid(self):
        
        print("Start node:", self.start)
        print("Goal node:", self.goal)
        print("Grid:")
        for row in self.grid:
            print(row)


class VisualiseProgram:
    def __init__(self, grid, start, goal):
        self.grid = grid  
        self.start = start  
        self.goal = goal  
        self.fig, self.ax = self.grid_visualisation()  

    def grid_visualisation(self):
        
        rows, cols = len(self.grid), len(self.grid[0])
        fig, ax = plt.subplots()
        ax.set_xlim(0, cols)
        ax.set_ylim(0, rows)
        ax.invert_yaxis()  
        ax.set_xticks(range(cols)), ax.set_yticks(range(rows))
        ax.grid(True)  

        
        for i in range(rows):
            for j in range(cols):
                if self.grid[i][j] == 1:  
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
                elif self.grid[i][j] == 2:  
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='orange'))

        
        ax.add_patch(plt.Rectangle((self.start[1], self.start[0]), 1, 1, color='green'))  
        ax.add_patch(plt.Rectangle((self.goal[1], self.goal[0]), 1, 1, color='red'))  

        return fig, ax

    def visualise_search(self, current):
        
        if current != self.start and current != self.goal:
            if self.grid[current[0]][current[1]] == 2:  
                self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='sienna'))
            else:  
                self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='lightblue'))
            plt.pause(0.05)

    def visualise_shortest_path(self, path):
        
        for node in path:
            
            if node != self.start and node != self.goal:
                if self.grid[node[0]][node[1]] == 2:  
                    self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='goldenrod'))
                else:  
                    self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow'))
                plt.pause(0.1)

        plt.show()

class DepthFirstSearch:
    def __init__(self, grid, start, goal, visualisation):
        self.grid = grid  # Grid representation
        self.start = start  # Start position
        self.goal = goal  # Goal position
        self.visualisation = visualisation  # Visualization instance

    def search_algorithm(self):
        # Perform Depth First Search to find a path
        rows, cols = len(self.grid), len(self.grid[0])
        stack = [(self.start, [self.start])]  # Stack to manage exploration
        visited = set()  # Keep track of visited nodes

        while stack:
            current, path = stack.pop()  # Get the current node and path

            if current in visited:  # Skip if already visited
                continue

            visited.add(current)  # Mark node as visited
            #self.visualisation.visualise_search(current)  # Visualize search progress

            if current == self.goal:  # If goal is found
                total_cost = 0
                #self.visualisation.visualise_shortest_path(path)  # Visualize the path
                for node in path:
                    if self.grid[node[0]][node[1]] == 0:
                        total_cost += 1
                    else:
                        total_cost += 2

                print("Total Cost:", total_cost)        
                return path  # Return the found path

            # Explore neighbors (down, up, right, left)
            neighbors = [
                (current[0] + 1, current[1]),
                (current[0] - 1, current[1]),
                (current[0], current[1] + 1),
                (current[0], current[1] - 1)
            ]

            for neighbor in neighbors:
                r, c = neighbor
                if 0 <= r < rows and 0 <= c < cols and self.grid[r][c] != 1 and neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))  # Add valid neighbors to the stack

        plt.show()
        return None  # Return None if no path is found

# ProgramLoop class manages the overall execution of the program
class ProgramLoop:
    def __init__(self):
        self.rows, self.cols = 20, 20  # Grid dimensions

    def run(self):
        # Create grid and display its structure
        grid = Grid(self.rows, self.cols)
        grid.display_grid()

        # Initialize visualization
        visualisation = VisualiseProgram(grid.grid, grid.start, grid.goal)

        # Perform Depth First Search
        dfs = DepthFirstSearch(grid.grid, grid.start, grid.goal, visualisation)
        start = time.time()
        tracemalloc.start()
        path = dfs.search_algorithm()
        end = time.time()
        print("Time Taken:", end - start, "Seconds")
        current, peak = tracemalloc.get_traced_memory()
        print("Current Memory:",(current / 10**6) , "Peak Memory:", (peak / 10**6))
        tracemalloc.stop()
        # Print the results
        if path:
            print(f"\nNumber of nodes in the shortest path: {len(path)}")
            print("\nPath found:")
            for node in path:
                print(node)
        else:
            print("\nNo path found.")

# Run the program
if __name__ == "__main__":
    ProgramLoop().run()
