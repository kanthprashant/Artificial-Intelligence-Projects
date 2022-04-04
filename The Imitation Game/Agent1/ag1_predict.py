#from queue import PriorityQueue
import numpy as np
#from itertools import chain
import csv
import time
import gridSolvable
import sys
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
#from sklearn.preprocessing import normalize
#from google.colab import drive

#function to calculate manhattan distance (used to calculate heuristics)
def manhattan_distance(ag_X, ag_Y):
    dist = abs(ag_X - (dim_x-1)) + abs(ag_Y - (dim_y-1))
    return dist

#get formatted 'child_x,childy' to easily search in closed_knowledge
def _format(child_x, child_y):
    return str(child_x) + "," + str(child_y)

# function to get 5X5 window
def get_window(node, move_grid):
    x, y = node[0], node[1]
    temp_window = np.random.choice([1, -1], p=[0, 1], size=(5, 5))
    #(x, y)
    temp_window[2, 2] = 0
    #(x-2, y-2)
    if x-1>0 and y-1>0:
        temp_window[0,0] = move_grid[x-2, y-2]
    #(x-2, y-1)
    if x-1>0 and y>0:
        temp_window[0,1] = move_grid[x-2, y-1]
    #(x-2, y)
    if x-1>0:
        temp_window[0,2] = move_grid[x-2, y]
    #(x-2, y+1)
    if x-1>0 and y<(dim_y-1):
        temp_window[0,3] = move_grid[x-2, y+1]
    #(x-2, y+2)
    if x-1>0 and y+1<(dim_y-1):
        temp_window[0,4] = move_grid[x-2, y+2]
    #(x-1, y-2)
    if x>0 and y-1>0:
        temp_window[1,0] = move_grid[x-1, y-2]
    #(x-1, y-1)
    if x>0 and y>0:
        temp_window[1, 1] = move_grid[x-1, y-1]
    #(x-1, y)
    if x>0:
        temp_window[1, 2] = move_grid[x-1, y]
    #(x-1, y+1)
    if x>0 and y<(dim_y-1):
        temp_window[1, 3] = move_grid[x-1, y+1]
    #(x-1, y+2)
    if x>0 and y+1<(dim_y-1):
        temp_window[1, 4] = move_grid[x-1, y+2]
    #(x, y-2)
    if y-1>0:
        temp_window[2, 0] = move_grid[x, y-2]
    #(x, y-1)
    if y>0:
        temp_window[2, 1] = move_grid[x, y-1]
     #(x, y+1)
    if y<(dim_y-1):
        temp_window[2, 3] = move_grid[x, y+1]
    #(x, y+2)
    if y+1<(dim_y-1):
        temp_window[2, 4] = move_grid[x, y+2]
    #(x+1, y-2)
    if x<dim_x-1 and y-1>0:
        temp_window[3, 0] = move_grid[x+1, y-2]
    #(x+1, y-1)
    if x < (dim_x - 1) and y > 0:
        temp_window[3, 1] = move_grid[x+1, y-1]
    #(x+1, y)
    if x<(dim_x-1):
        temp_window[3, 2] = move_grid[x+1, y]
    #(x+1, y+1)
    if x < (dim_x - 1) and y < (dim_y - 1):
        temp_window[3, 3] = move_grid[x+1, y+1]
    #(x+1, y+2)
    if x < (dim_x - 1) and y+1 < (dim_y - 1):
        temp_window[3, 4] = move_grid[x+1, y+2]
    #(x+2, y-2)
    if x+1<(dim_x-1) and y-1>0:
        temp_window[4, 0] = move_grid[x+2, y-2]
    #(x+2, y-1)
    if x+1 < (dim_x - 1) and y > 0:
        temp_window[4, 1] = move_grid[x+2, y-1]
    #(x+2, y)
    if x+1<(dim_x-1):
        temp_window[4, 2] = move_grid[x+2, y]
    #(x+2, y+1)
    if x+1 < (dim_x - 1) and y < (dim_y - 1):
        temp_window[4, 3] = move_grid[x+2, y+1]
    #(x+2, y+2)
    if x+1 < (dim_x - 1) and y+1 < (dim_y - 1):
        temp_window[4, 4] = move_grid[x+2, y+2]

    return temp_window

def execution(direction, current, discovered_grid, trajectory_path, final_path, visited, bumps):
    # prev position
    prev_x, prev_y = current[0], current[1]
    # current position holder
    x, y = prev_x, prev_y

    # direction for movement
    direc = 'NA'
    if direction == 0:
        direc = 'down'
        x += 1
    elif direction == 1:
        direc  = 'left'
        y -= 1
    elif direction == 2:
        direc = 'right'
        y += 1
    else:
        direc = 'up'
        x -= 1

    #print("prev position: {}, predicted direction: {}\ncurrent position: {}".format(_format(prev_x, prev_y), direc, _format(x, y)))

    # if current position is a block
    if maze[x, y] == -1:
        bumps += 1
        #print("Block Encountered at: {}".format(_format(x, y)))
        # update discovered grid
        discovered_grid[x, y] = -1
        visited[_format(x,y)] = 'visited'
        # set prev position as current position parent
        x, y = prev_x, prev_y
        #print("Current position: {}".format(_format(x, y)))
        # append in trajectory
        trajectory_path.append(_format(x, y))
        move_grid = discovered_grid.copy()
        move_grid[x, y] = 0
        # get window for current position
        window = get_window((x, y), move_grid).flatten()
        window = np.asarray([window])
        #window = normalize(window)
        window = sc_X.transform(window)
        window = np.reshape(window, (-1, 5, 5, 1)) # for CNN

        return 0, (x, y), window, trajectory_path, final_path, visited, bumps
    
    # if reached at destination
    elif (x == dim_x - 1) and (y == dim_y - 1):
        # visited
        discovered_grid[x, y] = 2
        visited[_format(x,y)] = 'visited'
        # append in trajectory path
        trajectory_path.append(_format(x, y))
        # append in final path
        final_path.append(_format(x, y))
        move_grid = discovered_grid.copy()
        move_grid[x, y] = 0
        # get window for current position
        window = get_window((x, y), move_grid).flatten()
        window = np.asarray([window])
        #window = normalize(window)
        window = sc_X.transform(window)
        window = np.reshape(window, (-1, 5, 5, 1)) # for CNN

        return 1, (x, y), window, trajectory_path, final_path, visited, bumps
    
    else:
        # visited
        visited[_format(x,y)] = 'visited'
        discovered_grid[x, y] = 2

        #blocked (x - 1) child
        if x > 0 and maze[x - 1, y] == -1:
            discovered_grid[x - 1, y] = -1
        #blocked (x + 1) child
        if x < dim_x-1 and maze[x + 1, y] == -1:
            discovered_grid[x + 1, y] = -1
        #blocked (y + 1) child
        if y < dim_y-1 and maze[x, y + 1] == -1:
            discovered_grid[x, y + 1] = -1
        #blocked (y - 1) child
        if y > 0 and maze[x, y - 1] == -1:
            discovered_grid[x, y - 1] = -1
        
        #unblocked (x - 1) child
        if x > 0 and discovered_grid[x - 1, y] != -1:
            if _format(x-1,y) not in visited:
                discovered_grid[x - 1, y] = 3
        #unblocked (x + 1) child
        if x < dim_x-1 and discovered_grid[x + 1, y] != -1:
            if _format(x+1,y) not in visited:
                discovered_grid[x + 1, y] = 3
        #unblocked (y + 1) child
        if y < dim_y-1 and discovered_grid[x, y + 1] != -1:
            if _format(x,y+1) not in visited:
                discovered_grid[x, y + 1] = 3
        #unblocked (y - 1) child
        if y > 0 and discovered_grid[x, y - 1] != -1:
            if _format(x,y-1) not in visited:
                discovered_grid[x, y - 1] = 3

        # append in trajectory path
        trajectory_path.append(_format(x, y))
        # append in final path
        final_path.append(_format(x, y))

        move_grid = discovered_grid.copy()
        move_grid[x, y] = 0
        # get window for current position
        window = get_window((x, y), move_grid).flatten()
        window = np.asarray([window])
        #window = normalize(window)
        window = sc_X.transform(window)
        window = np.reshape(window, (-1, 5, 5, 1)) # for CNN

        #print("Window Generated")

        return 0, (x, y), window, trajectory_path, final_path, visited, bumps
                

def plan(current, window, model, discovered_grid, trajectory_path, final_path, visited):
    count = {}
    count[_format(current[0], current[1])] = 1
    solved = 0
    n_x = 0
    n_y = 0
    bumps = 0

    while not solved:
        y_pred  = model.predict(window)
        direction = np.argmax(y_pred[0])
        #print("current in plan: {}".format(current))
        
        if count[_format(current[0], current[1])] > 8:
            direction = np.argsort(y_pred[0])[-2]

        if direction == 0: n_x += 1
        elif direction == 1: n_y -= 1
        elif direction == 2: n_y += 1
        else: n_x -= 1
        if n_x > dim_x-1 or n_y > dim_y-1 or n_x < 0 or n_y < 0:
             direction = np.argsort(y_pred[0])[-1]

        status, current, window, trajectory_path, final_path, visited, bumps = execution(direction, current, discovered_grid, trajectory_path, final_path, visited, bumps)
        solved = status
        
        # stop model from solving infinitely
        if _format(current[0], current[1]) in count:
            count[_format(current[0], current[1])] = count[_format(current[0], current[1])] + 1
        else: count[_format(current[0], current[1])] = 1

        if status == 1:
            #print("Success! Path Found.")
            return 1, trajectory_path, final_path, bumps
        
        # 4 neighbours * 3 error trials = 12
        for value in count.values():
            if value > 12: return 0, trajectory_path, final_path, bumps

    #return 0, trajectory_path, final_path

# load model
#drive.mount('/content/drive/')
#model = keras.models.load_model('/content/drive/My Drive/Imitation Game/My_9629_model')
model = keras.models.load_model('My5b5_convAg1_9917_model') # ag1_Keras_9_8_8_4

# Importing the dataset
dataset = pd.read_csv('ag1_5by5dGrid.csv', names = range(26))
X = dataset.iloc[:, :-1].values
y = dataset.iloc[:, -1].values
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.1, random_state =0)

sc_X = StandardScaler()
X_train = sc_X.fit_transform(X_train)

# take dimensions input
dim_x, dim_y = map(int, input("Enter matrix size for maze (rows x cols): ").split())
n = int(input("Number of repetitions: "))

# writer to write solved grids in a csv
a = open('NewData/ag1_KerasSolvedGrids.csv', 'w')
writer = csv.writer(a)

dense_traject = [[],[]]
dense_discpath = [[],[]]
density_bumps = [[],[]]
density_time = [[],[]]
grid_solve = [[],[]]

for i in range(1,31,1):
    avg_trajectory = 0
    trajectory_cost = 0
    avg_shortest_cost = 0
    shortest_cost = 0
    n_bumps = 0
    avg_bumps = 0
    runtime = 0
    avg_runtime = 0
    avg_solve = 0
    solve_counter = 0
    count_sol = 0
    count_noSol = 0

    # Run for N repetitions:
    while solve_counter != n:
        #generate random maze
        maze = np.random.choice([1, -1], p=[0.7, 0.3], size=(dim_x, dim_y))

        #ensure top left and bottom right are not blocked
        if maze[0,0] != 1:
            maze[0,0] = 1
        if maze[dim_x-1,dim_y-1] != 1:
            maze[dim_x-1,dim_y-1] = 1

        #initialize source
        source_x = source_y = 0
        # Find if maze is solvable
        solvable, path = gridSolvable.target_search(maze, (source_x, source_y), (dim_x-1, dim_y-1))

        if solvable:
            solve_counter += 1

            print("Epoch {}: Maze is solvable:{}".format(i, solve_counter))

            #free space assumption of discovered grid
            discovered_grid = np.random.choice([1, -1], p=[1, 0], size=(dim_x, dim_y))
            if discovered_grid[0,0] != 1:
                maze[0,0] = 1
            if discovered_grid[dim_x-1,dim_y-1] != 1:
                maze[dim_x-1,dim_y-1] = 1

            trajectory_path = []
            final_path = []
            visited = {}
            visited[_format(source_x, source_y)] = 'visited'
            trajectory_path.append(_format(source_x, source_y))
            final_path.append(_format(source_x, source_y))

            discovered_grid[source_x, source_y] = 2
            move_grid = discovered_grid.copy()
            move_grid[source_x, source_y] = 0
            # get window for current position
            window = get_window((source_x, source_y), move_grid).flatten()
            window = np.asarray([window])
            #np.reshape(X_test, (-1, 3, 3, 1))
            #window = normalize(window)
            window = sc_X.transform(window)
            window = np.reshape(window, (-1, 5, 5, 1)) # for CNN

            start = time.time()
            status, trajectory_path, final_path, bumps = plan((source_x, source_y), window, model, discovered_grid, trajectory_path, final_path, visited)
            end = time.time()

            if status:
                print("Success!! Path Found.")
                count_sol += 1
                writer.writerow(maze.flatten())
                n_bumps = n_bumps + bumps
                avg_bumps = n_bumps/count_sol
                trajectory_cost = trajectory_cost + (len(trajectory_path) - 1)
                avg_trajectory = trajectory_cost/count_sol
                shortest_cost = shortest_cost + (len(final_path) - 1)
                avg_shortest_cost = shortest_cost/count_sol
                runtime = runtime + float("%.7f" % (end-start))
                avg_runtime = runtime/count_sol
                
            else:
                print("Could not reach the target!!")
                count_noSol += 1

            #print("Trajectory: {}".format(trajectory_path))
            #print("Final Path: {}".format(final_path))


        else:
            print("** Maze is not solvable ** Generating again ...")
    
    density_time[0].append(i)
    density_time[1].append(avg_runtime)
    dense_traject[0].append(i)
    dense_traject[1].append(avg_trajectory)
    dense_discpath[0].append(i)
    dense_discpath[1].append(avg_shortest_cost)
    density_bumps[0].append(i)
    density_bumps[1].append(avg_bumps)
    grid_solve[0].append(i)
    grid_solve[1].append(count_sol)

print("Epoch {} - Solved: {}, Unsolved: {}".format(i, count_sol, count_noSol))

with open('NewData/AG1_traj.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(dense_traject[0])
        writer.writerow(dense_traject[1])

with open('NewData/AG1_discpath.csv', 'w', encoding='UTF8', newline='') as g:
        writer = csv.writer(g)
        writer.writerow(dense_discpath[0])
        writer.writerow(dense_discpath[1])

with open('NewDataAG1_bumps.csv', 'w', encoding='UTF8', newline='') as h:
        writer = csv.writer(h)
        writer.writerow(density_bumps[0])
        writer.writerow(density_bumps[1])

with open('NewData/AG1_time.csv', 'w', encoding='UTF8', newline='') as i:
        writer = csv.writer(i)
        writer.writerow(density_time[0])
        writer.writerow(density_time[1])

with open('NewData/AG1_solve.csv', 'w', encoding='UTF8', newline='') as j:
        writer = csv.writer(j)
        writer.writerow(grid_solve[0])
        writer.writerow(grid_solve[1])