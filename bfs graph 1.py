import random
import matplotlib.pyplot as plt
import time
import tracemalloc


#this program creates a 20x20 grid of 1s (wall nodes) and 0s (traversable nodes), 
#each time the program is ran, walls are spanwed randomly on the grid and the position of the start and goal nodes is also randomnly spawned

class Grid:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = self.create_grid()
        self.start, self.goal = self.set_start_goal()
        self.generate_ramp()

    def create_grid(self): #function to create grid
        random.seed(42) #seed for reproducibility (keeps grid the same every time)
        grid = [[1 if random.random() < 0.15 else 0 for _ in range(self.cols)] for _ in range(self.rows)] #spawns 1 (wall) at a 0.15 chance rate and the rest is 0 (traversable node)
        return grid

    def set_start_goal(self): #function to set the positions of start and goal nodes 
        start = (5, 5) #position of start node
        goal = (19, 19) #position of goal node
        return start, goal

    def generate_ramp(self): #function to generate ramps (cost == 2) on map
        random.seed(42) #seed for reproducibility (keeps ramp pistion the same every time)
        for r in range(self.rows): #for each node in row,
            for c in range(self.cols): #and for each node in column:
                if self.grid[r][c] == 0 and random.random() < 0.1: #if the current node == 0 and random value is below 0.1:
                    self.grid[r][c] = 2 #set current node to 2 (traversable node, but costs 2 to traversal)

    def display_grid(self): #function to print grid
        print("Start node:", self.start)
        print("Goal node:", self.goal)
        print("Grid:")
        for row in self.grid: #for each row in grid,
            print(row) #print row


# Visualisation Program Class
class VisualiseProgram:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.fig, self.ax = self.grid_visualisation()

    def grid_visualisation(self):  # function to visualize the grid
        rows, cols = len(self.grid), len(self.grid[0])
        fig, ax = plt.subplots()  # fig is the canvas, ax is the drawing on the canvas
        ax.set_xlim(0, cols)  # sets the limit of columns in visualization so that it matches the 2D grid
        ax.set_ylim(0, rows)  # sets the limit of rows in visualization so that it matches the 2D grid
        ax.invert_yaxis()  # inverts y-axis so that the visualization is kept consistent with the 2D grid
        ax.set_xticks(range(cols)), ax.set_yticks(range(rows))  # sets the ticks in the range of the cols and rows
        ax.grid(True)  # displays lines on the grid to separate each node in the visualization

        # draw the initial grid
        for i in range(rows):  # iterate over every cell in rows
            for j in range(cols):  # iterate over every cell in cols
                if self.grid[i][j] == 1:  # if there's a wall
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))  # plot a black rectangle
                elif self.grid[i][j] == 2:  # if the node is a ramp,
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='orange'))  #plot an orange rectangle to represent a ramp

        ax.add_patch(plt.Rectangle((self.start[1], self.start[0]), 1, 1, color='green'))  # start node in green
        ax.add_patch(plt.Rectangle((self.goal[1], self.goal[0]), 1, 1, color='red'))  # goal node in red
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event:
                                     [exit(0) if event.key == 'escape'
                                      else None])
        plt.pause(0.1)  # gives a small timeframe for the visualization to update

        return fig, ax

    def visualise_search(self, current):  # visualizes how the search algorithm visits nodes
        if current != self.start and current != self.goal:  # if current != start or goal
            if self.grid[current[0]][current[1]] == 2:
                self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='sienna')) #then plot a 1x1 sienna (mix of orange and blue) rectangle on that node to represent a checked ramp
            else:
                self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='lightblue'))
            plt.pause(0.05)  # sets the speed of the search visualization

    def visualise_shortest_path(self, path):  # visualizes the shortest path found by the algorithm
        for node in path:  # loops through each node in the path
            if node != self.start and node != self.goal:  # if node != start or goal
                if self.grid[node[0]][node[1]] == 2: #if current node == 2,
                    self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='goldenrod')) #then plot a 1x1 goldenrod (mix of yellow and sienna) rectangle
                else:
                    self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow')) #then plot a 1x1 yellow rectangle
                plt.pause(0.1) #timeframe between each yellow rectangle being plotted
        plt.show()  # displays the final visualization of the grid


# Breadth-First Search Planner
class BreadthFirstSearchPlanner:
    def __init__(self, grid): 
        self.grid = grid
        self.xwidth = len(grid)
        self.ywidth = len(grid[0])
        self.motion = self.get_motion_model_4n()

    class Node:
        def __init__(self, x, y, cost, parent_index, parent):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
            self.parent = parent

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy, visualiser=None):
        start_node = self.Node(sx, sy, 0.0, -1, None)
        goal_node = self.Node(gx, gy, 0.0, -1, None)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        cost_map = {self.calc_grid_index(start_node): 0}


        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            current = open_set.pop(list(open_set.keys())[0])
            c_id = self.calc_grid_index(current)
            closed_set[c_id] = current

           # if visualiser:
            #    visualiser.visualise_search((current.x, current.y))

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Found goal!")
                goal_node.parent_index = current.parent_index  # Set the goal node's parent
                goal_node.parent = current  # Assign the parent node to goal node
                break

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id, None)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if (n_id not in closed_set) and (n_id not in open_set):
                    node.parent = current
                    open_set[n_id] = node

        rx, ry, total_cost = self.calc_final_path(goal_node)
        print(f'total cost: {total_cost}')
        #if visualiser:
          #  visualiser.visualise_shortest_path(list(zip(rx, ry)))
        return rx, ry

    def calc_final_path(self, goal_node):
        total_cost = 0
        rx, ry = [], []
        n = goal_node
        while n.parent is not None:
            rx.append(n.parent.x)
            ry.append(n.parent.y)
            if self.grid[n.x][n.y] == 0:
                total_cost += 1    
            else:
                total_cost += 2
            n = n.parent
            
        return rx[::-1], ry[::-1], total_cost

    def calc_grid_index(self, node):
        return node.y * self.xwidth + node.x

    def verify_node(self, node):
        if node.x < 0 or node.y < 0 or node.x >= self.xwidth or node.y >= self.ywidth:
            return False
        if self.grid[node.x][node.y] == 1:  # Check if there's an obstacle
            return False
        return True

    def get_motion_model_4n(self):
        return [[1, 0, 1], 
                [0, 1, 1], 
                [-1, 0, 1], 
                [0, -1, 1]]


# Main function to run the BFS and visualize the path
def main():
    # Define environment parameters

#    print("time start to calculate" )

    # Create a grid with specific parameters
    grid = Grid(rows=20, cols=20)
    grid.display_grid()

    # Create a visualization object
    visualiser = VisualiseProgram(grid.grid, grid.start, grid.goal)

    bfs = BreadthFirstSearchPlanner(grid.grid) 
    tracemalloc.start()
    start = time.time()
    sx, sy = grid.start
    gx, gy = grid.goal

    print(f"Start: {sx}, {sy}")
    print(f"Goal: {gx}, {gy}")

    # Run BFS planner and visualize the search and the path
    rx, ry = bfs.planning(sx, sy, gx, gy, visualiser=visualiser)
        ## zip two list to form coordinate 
    new_rx = map(int, rx)
    new_ry = map(int, ry)
    
    road = [list(a) for a in zip(new_rx, new_ry)]
    
    ##output path 
    print("Length of Path:", len(road))
    print("road from start to goal", road)
    ##calculate time to run 
    end = time.time()
#    print("time end to calculate time")
    print("time used", end - start)
    
    ##calculate storage memory
    current, peak = tracemalloc.get_traced_memory()
    print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
    tracemalloc.stop()

    # Final grid and path visualization
    plt.show()


if __name__ == '__main__':
    main()