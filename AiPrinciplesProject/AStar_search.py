import random
import matplotlib.pyplot as plt
import heapq 
import time
import tracemalloc 

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

class VisualiseProgram:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.fig, self.ax = self.grid_visualisation()

    def grid_visualisation(self): #function to visualise the grid
        rows, cols = len(self.grid), len(self.grid[0])
        fig, ax = plt.subplots() #fig is the canvas, ax is the the 'drawing' on the canvas
        ax.set_xlim(0, cols) #sets the limit of colums in visualisation so that it matches the 2D grid
        ax.set_ylim(0, rows) #sets the limit of rows in visualisation so that it matches the 2D grid
        ax.invert_yaxis() #invertys y-axis so that the visualisation is kept consistant with the 2D grid
        ax.set_xticks(range(cols)), ax.set_yticks(range(rows)) #sets the ticks in the range of the cols and rows so that each node is visualised properly
        ax.grid(True) #displays lines on the grid to seperate each node in the visualisation

        #draw the initial grid
        for i in range(rows): #iterate over every cell in rows
            for j in range(cols): #iterate over every cell in cols
                if self.grid[i][j] == 1: #if either one of the nodes in rows or cols == 1 (wall),
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black')) #then plot a black rectangle of size 1x1 at position (j, i)
                elif self.grid[i][j] == 2:  # if the node is a ramp,
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='orange'))  #plot an orange rectangle to represent a ramp

        ax.add_patch(plt.Rectangle((self.start[1], self.start[0]), 1, 1, color='green')) #plot a 1x1 green rectangle that represents the start node
        ax.add_patch(plt.Rectangle((self.goal[1], self.goal[0]), 1, 1, color='red')) #plot a 1x1 red rectagle that represents the goal node
        plt.pause(0.1) #giive a small timeframe for the visualisation to be updated before shown

        return fig, ax 

    def visualise_search(self, current): #function to visualise how the search algorithm visits nodes
        if current != self.start and current != self.goal:
            if self.grid[current[0]][current[1]] == 2:
                self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='sienna')) #then plot a 1x1 sienna (mix of orange and blue) rectangle on that node to represent a checked ramp
            else:
                self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='lightblue')) #then plot a 1x1 light blue rectangle to represent a checked node
            plt.pause(0.05) #sets the pause between each rectangle drawn (in short: sets the speed of the search visualisation)

    def visualise_shortest_path(self, path): #visualises the shortest path found by the algorithm
        for node in path: #loops through each node in the path found by the search algorithm
            if node != self.start and node != self.goal: #if node != start or goal,
                if self.grid[node[0]][node[1]] == 2: #if current node == 2,
                    self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='goldenrod')) #then plot a 1x1 goldenrod (mix of yellow and sienna) rectangle
                else:
                    self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow')) #then plot a 1x1 yellow rectangle
                plt.pause(0.1) #timeframe between each yellow rectangle being plotted

        plt.show() #displays the final visualisation of the grid




class AStarSearch:
    def __init__(self,grid, start, goal, visualisation,):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.visualisation = visualisation

    def Manhattan_Distance(self, node1, node2): #Manhattan Distance is Used as the agent only moves in 4 directions
        return (abs(node1[0] - node2[0]) + abs(node1[1] - node2[1]))

    def Graphsearch(self):
        
        #Define Empty Lists 
        visited = set()
        unvisited_queue = []

        #Keeps Track of Parent Nodes (Maps Each Visited Node to its Parent Node)
        parent_nodes = {self.start: None}

        node_f_values = {self.start: 0 }

        heapq.heappush(unvisited_queue,(0,self.start)) #Puts Start Node into queue (Heuristic f = 0)
        
        num_rows = len(self.grid)
        num_col = len(self.grid[0])
        while len(unvisited_queue) > 0:     #While there are still nodes to visit

            f, node = heapq.heappop(unvisited_queue) #Chooses to Visit "Best Node" (Lowest F Value)
            
            if node in visited:
                continue #If node has been visited, do not execute rest of loop

            visited.add(node) #Adds node to set of visited nodes

            #self.visualisation.visualise_search(node) #Adds to environments visualisation

            if node == self.goal: #if goal has been reached
                totalcost = 0 #Initialises Cost
                full_path = [] #Initialises path 
                while node is not None:
                    full_path.append(node)
                    if self.grid[node[0]][node[1]] == 0:
                        totalcost += 1
                    else:
                        totalcost += 2    
                    node = parent_nodes[node] #Finds Parent Node of the current node in dictionary

                #self.visualisation.visualise_shortest_path(full_path[::-1])
                print("Cost:", totalcost) 
                print("Nodes in Path:", len(full_path))
                return full_path[::-1] #Returns Full Path     
            
            #Find Possible Nodes to Traverse Next (4 Possible Directions)
            next_nodes = { (node[0], node[1] + 1), (node[0], node[1] - 1), (node[0] + 1, node[1]), (node[0] - 1, node[1]) }
            
            for nextnode in next_nodes:   #For Each Possible Next Node
                #Check that it is a valid move (Check it is in grid first then check it is not an obstacle)
                if (nextnode[0] >= 0 and nextnode[0] < num_rows) and (nextnode[1] >= 0 and nextnode[1] < num_col) and self.grid[nextnode[0]][nextnode[1]] != 1:
                    h = self.Manhattan_Distance(nextnode,self.goal) #Calculate Heuristic Value with Manhattan Distance
                    if self.grid[nextnode[0]][nextnode[1]] == 0: #Normal Node
                        c = 1 
                    else: #Ramp (Cost is 2 for ramps)
                        c = 2


                    new_f = h + c #Add Heuristic and Cost together to get final f value 

                    if nextnode not in node_f_values or new_f < node_f_values[nextnode] : #If The Node Has No Mapped Total Heuristic Cost or the new F Value is lower
                        heapq.heappush(unvisited_queue,(new_f,nextnode)) #Add Node to List of Nodes to Choose From
                        node_f_values[nextnode] = new_f         #Adds Cost to Map (Or New Lower Cost if found)
                        parent_nodes[nextnode] = node #Adds Node to Map with its parent node (Original Node) 



    def Treesearch(self): #Removes the visited list to implement tree Search
        
        
        unvisited_queue = []

        #Keeps Track of Parent Nodes (Maps Each Visited Node to its Parent Node)
        parent_nodes = {self.start: None}

        node_f_values = {self.start: 0 }

        heapq.heappush(unvisited_queue,(0,self.start)) #Puts Start Node into queue (Heuristic f = 0)
        
        num_rows = len(self.grid)
        num_col = len(self.grid[0])
        while len(unvisited_queue) > 0:     #While there are still nodes to visit

            f, node = heapq.heappop(unvisited_queue) #Chooses to Visit "Best Node" (Lowest F Value)

            self.visualisation.visualise_search(node) #Adds to environments visualisation

            if node == self.goal: #if goal has been reached
                totalcost = 0
                full_path = [] #Initialises path 
                
                while node is not None:
                    full_path.append(node)
                    if self.grid[node[0]][node[1]] == 0:
                        totalcost += 1
                    else:
                        totalcost += 2  
                    node = parent_nodes[node] #Finds Parent Node of the current node in dictionary

                self.visualisation.visualise_shortest_path(full_path[::-1])
                print("Cost:", totalcost)
                print("Nodes in Path:", len(full_path)) 
                return full_path[::-1] #Returns Full Path     
            
            #Find Possible Nodes to Traverse Next (4 Possible Directions)
            next_nodes = { (node[0], node[1] + 1), (node[0], node[1] - 1), (node[0] + 1, node[1]), (node[0] - 1, node[1]) }
            
            for nextnode in next_nodes:   #For Each Possible Next Node
                #Check that it is a valid move (Check it is in grid first then check it is not an obstacle)
                if (nextnode[0] >= 0 and nextnode[0] < num_rows) and (nextnode[1] >= 0 and nextnode[1] < num_col) and self.grid[nextnode[0]][nextnode[1]] != 1:
                    h = self.Manhattan_Distance(nextnode,self.goal) #Calculate Heuristic Value with Manhattan Distance
                    if self.grid[nextnode[0]][nextnode[1]] == 0: #Normal Node
                        c = 1 
                    else: #Ramp (Cost is 2 for ramps)
                        c = 2


                    new_f = h + c #Add Heuristic and Cost together to get final f value 

                    if nextnode not in node_f_values or new_f < node_f_values[nextnode] : #If The Node Has No Mapped Total Heuristic Cost or the new F Value is lower
                        heapq.heappush(unvisited_queue,(new_f,nextnode)) #Add Node to List of Nodes to Choose From
                        node_f_values[nextnode] = new_f         #Adds Cost to Map (Or New Lower Cost if found)
                        parent_nodes[nextnode] = node #Adds Node to Map with its parent node (Original Node)                     
                        
                
                      
        
        

def findPath():
    
    

    #Initialise Grid and Visualisation
    rows = 20
    cols = 20
    grid = Grid(rows, cols)
    grid.display_grid()


    visualisation = VisualiseProgram(grid.grid, grid.start,grid.goal)
    search = AStarSearch(grid.grid, grid.start,grid.goal,visualisation)
    start = time.time()
    tracemalloc.start()
    print("Path:",search.Graphsearch())
    #print("Path:",search.Treesearch())
    end = time.time()
    print("Time Taken:", end - start, "Seconds")
    current, peak = tracemalloc.get_traced_memory()
    print("Current Memory:",(current / 10**6) , "Peak Memory:", (peak / 10**6))

findPath()    