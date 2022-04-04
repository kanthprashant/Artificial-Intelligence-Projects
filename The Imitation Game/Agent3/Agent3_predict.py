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
    if child_x == None and child_y == None:
        return 'None,None'
    else:
        return str(child_x) + "," + str(child_y)

#return path
def get_path(goal_X, goal_Y, grid):
    discovered_path = [(goal_X, goal_Y)]
    while grid[_format(discovered_path[-1][0], discovered_path[-1][1])]['p'] != (None, None):
        discovered_path.append(grid[_format(discovered_path[-1][0], discovered_path[-1][1])]['p'])
    discovered_path.reverse()
    return discovered_path

#Add to knowledge Infer
def generate(child_x, child_y, current, temp_base):
    temp_base.append(_format(child_x, child_y))

def infer(current_x, current_y, k_infer, infer_base, p_infer):
    
    level = 3
    x = current_x
    y = current_y
    
    #print("Inferring at:",_format(current_x, current_y))

    #re-store p_infer at all current true status of cells
    for node in k_infer[_format(current_x, current_y)]:
        node_x, node_y = map(int, node.split(','))
        if _format(node_x, node_y) not in p_infer:
            p_infer[_format(node_x, node_y)] = {'hx'}

    #begin inferring new info       
    while _format(x, y) != 'None,None' and level >= 0 and visited[_format(x,y)] == 'visited':
        
        bx = ex = hx = 0
        
        #re-count bx ex and hx in p_infer for (x,y)
        for node in k_infer[_format(x, y)]:
            if p_infer[node] == {'bx'}: bx = bx + 1
            if p_infer[node] == {'ex'}: ex = ex + 1
            if p_infer[node] == {'hx'}: hx = hx + 1
            
        #set correct values in infer_base for (x,y) ex bx and hx
        infer_base[_format(x, y)]['ex'] = ex
        infer_base[_format(x, y)]['bx'] = bx
        infer_base[_format(x, y)]['hx'] = hx

        #cx = bx, update all hx as empty
        if infer_base[_format(x, y)]['hx'] != 0:
            if infer_base[_format(x, y)]['cx'] == infer_base[_format(x, y)]['bx']:
                for node in k_infer[_format(x, y)]:
                    node_x, node_y = map(int, node.split(','))
                    if p_infer[_format(node_x, node_y)] == {'hx'}:
                        p_infer[_format(node_x, node_y)] = {'ex'}
                        infer_base[_format(x, y)]['hx'] -= 1
                        infer_base[_format(x, y)]['ex'] += 1
                        if _format(node_x,node_y) not in visited:
                            discovered_grid[node_x, node_y] = 3
            
            #nx - cx = ex, update all hx as blocked
            elif infer_base[_format(x, y)]['nx'] - infer_base[_format(x, y)]['cx'] == infer_base[_format(x, y)]['ex']:
                for node in k_infer[_format(x, y)]:
                    node_x, node_y = map(int, node.split(','))
                    if p_infer[_format(node_x, node_y)] == {'hx'}:
                        p_infer[_format(node_x, node_y)] = {'bx'}
                        infer_base[_format(x, y)]['hx'] -= 1
                        infer_base[_format(x, y)]['bx'] += 1
                        discovered_grid[node_x, node_y] = -1
        
        temp_x = x
        temp_y = y

        pos = final_path.index(_format(temp_x, temp_y))
        x, y = map(int, final_path[pos-1].split(','))
        if x == 0 and y == 0:
            x = y = 'None'
        level = level - 1
             

def sense(current_x, current_y, k_infer, infer_base, p_infer):
    if _format(current_x, current_y) not in infer_base:
        nx = cx = bx = ex = 0
        temp_base = []
        #sense unblocked (x - 1) child
        if current_x > 0 and maze[current_x - 1, current_y] != -1:
            generate(current_x - 1, current_y, (current_x, current_y), temp_base)
            nx = nx + 1
        #sense blocked (x - 1) child
        if current_x > 0 and maze[current_x - 1, current_y] == -1:
            generate(current_x - 1, current_y, (current_x, current_y), temp_base)
            nx = nx + 1
            cx = cx + 1
        #sense unblocked (x + 1) child
        if current_x < dim_x-1 and maze[current_x + 1, current_y] != -1:
            generate(current_x + 1, current_y, (current_x, current_y), temp_base)
            nx = nx + 1   
        #sense blocked (x + 1) child
        if current_x < dim_x-1 and maze[current_x + 1, current_y] == -1:
            generate(current_x + 1, current_y, (current_x, current_y), temp_base)
            nx = nx + 1
            cx = cx + 1
        #sense unblocked (x - 1), (y - 1) child
        if current_x > 0 and current_y > 0 and maze[current_x - 1, current_y - 1] != -1:
            generate(current_x - 1, current_y - 1, (current_x, current_y), temp_base)
            nx = nx + 1
        #sense blocked (x - 1), (y - 1) child
        if current_x > 0 and current_y > 0 and maze[current_x - 1, current_y - 1] == -1:
            generate(current_x - 1, current_y - 1, (current_x, current_y), temp_base)
            nx = nx + 1
            cx = cx + 1
        #sense unblocked (x - 1), (y + 1) child
        if current_x > 0 and current_y < (dim_y - 1) and maze[current_x - 1, current_y + 1] != -1:
            generate(current_x - 1, current_y + 1, (current_x, current_y), temp_base)
            nx = nx + 1
        #sense blocked (x - 1), (y + 1) child
        if current_x > 0 and current_y < (dim_y - 1) and maze[current_x - 1, current_y + 1] == -1:
            generate(current_x - 1, current_y + 1, (current_x, current_y), temp_base)
            nx = nx + 1
            cx = cx + 1
        #sense unblocked (x + 1), (y - 1) child
        if current_x < (dim_x - 1) and current_y > 0 and maze[current_x + 1, current_y - 1] != -1:
            generate(current_x + 1, current_y - 1, (current_x, current_y), temp_base)
            nx = nx + 1
        #sense blocked (x + 1), (y - 1) child
        if current_x < (dim_x - 1) and current_y > 0 and maze[current_x + 1, current_y - 1] == -1:
            generate(current_x + 1, current_y - 1, (current_x, current_y), temp_base)
            nx = nx + 1
            cx = cx + 1
        #sense unblocked (x + 1), (y + 1) child
        if current_x < (dim_x - 1) and current_y < (dim_y - 1) and maze[current_x + 1, current_y + 1] != -1:
            generate(current_x + 1, current_y + 1, (current_x, current_y), temp_base)
            nx = nx + 1
        #sense blocked (x + 1), (y + 1) child
        if current_x < (dim_x - 1) and current_y < (dim_y - 1) and maze[current_x + 1, current_y + 1] == -1:
            generate(current_x + 1, current_y + 1, (current_x, current_y), temp_base)
            nx = nx + 1
            cx = cx + 1
        #sense unblocked (y + 1) child
        if current_y < dim_y-1 and maze[current_x, current_y + 1] != -1:
            generate(current_x, current_y + 1, (current_x, current_y), temp_base)
            nx = nx + 1
        #sense blocked (y + 1) child
        if current_y < dim_y-1 and maze[current_x, current_y + 1] == -1:
            generate(current_x, current_y + 1, (current_x, current_y), temp_base)
            nx = nx + 1
            cx = cx + 1
        #sense unblocked (y - 1) child
        if current_y > 0 and maze[current_x, current_y - 1] != -1:
            generate(current_x, current_y - 1, (current_x, current_y), temp_base)
            nx = nx + 1
        #sense blocked (y - 1) child
        if current_y > 0 and maze[current_x, current_y - 1] == -1:
            generate(current_x, current_y - 1, (current_x, current_y), temp_base)
            nx = nx + 1
            cx = cx + 1
        infer_base[_format(current_x, current_y)] = {
                'nx': nx,
                'cx': cx,
                'bx': bx,
                'ex': ex,
                'hx': nx
            }
        k_infer[_format(current_x, current_y)] = temp_base
        
    else:
        bx = ex = hx = 0
        for node in k_infer[_format(current_x, current_y)]:
            node_x, node_y = map(int, node.split(','))
            if p_infer[_format(node_x, node_y)] == {'ex'}: ex = ex + 1  
            if p_infer[_format(node_x, node_y)] == {'bx'}: bx = bx + 1
            if p_infer[_format(node_x, node_y)] == {'hx'}: hx = hx + 1
                
        infer_base[_format(current_x, current_y)]['ex'] = ex
        infer_base[_format(current_x, current_y)]['bx'] = bx
        infer_base[_format(current_x, current_y)]['hx'] = hx

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

    #print("window for {}:\n {}".format(node, temp_window))
    return temp_window

# execution of Agent 3
def execution(direction, current, discovered_grid, trajectory_path, final_path, k_infer, infer_base, p_infer, visited, bumps):
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
    if maze[x,y] == -1:
        bumps += 1
        print("Maze block: ({},{}):{}".format(x,y,maze[x,y]))
        print("Block Encountered at: {}".format(_format(x, y)))
        # update discovered grid
        discovered_grid[x, y] = -1
        visited[_format(x,y)] = 'visited'
        # set prev position as current position parent
        x, y = prev_x, prev_y
        print("Current position: {}".format(_format(x, y)))
        # append in trajectory
        trajectory_path.append(_format(x, y))
        p_infer[_format(x,y)] = {'bx'}
        
        infer(x, y, k_infer, infer_base, p_infer)
        
        move_grid = discovered_grid.copy()
        move_grid[x, y] = 0
        # get window for current position
        window = get_window((x, y), move_grid).flatten()
        window = np.asarray([window])
        #window = normalize(window)
        window = np.asarray([np.append(window, infer_base[_format(x,y)]['cx'])])
        window = sc_X.transform(window)

        return 0, (x, y), window, trajectory_path, final_path, visited, bumps
    
    elif (x == dim_x - 1) and (y == dim_y - 1):
        discovered_grid[x, y] = 2
        p_infer[_format(x,y)] = {'ex'}

        visited[_format(x,y)] = 'visited'
        # append in trajectory path
        trajectory_path.append(_format(x, y))
        # append in final path
        final_path.append(_format(x, y))

        sense(x, y, k_infer, infer_base, p_infer)
        infer(x, y, k_infer, infer_base, p_infer)

        move_grid = discovered_grid.copy()
        move_grid[x, y] = 0
        # get window for current position
        window = get_window((x, y), move_grid).flatten()
        window = np.asarray([window])
        #window = normalize(window)
        window = np.asarray([np.append(window, infer_base[_format(x,y)]['cx'])])
        window = sc_X.transform(window)

        return 1, (x, y), window, trajectory_path, final_path, visited, bumps
    
    else:
         discovered_grid[x, y] = 2
         p_infer[_format(x,y)] = {'ex'}

         # visited
         visited[_format(x,y)] = 'visited'
         # append in trajectory path
         trajectory_path.append(_format(x, y))
         # append in final path
         final_path.append(_format(x, y))
                
         sense(x, y, k_infer, infer_base, p_infer)
         infer(x, y, k_infer, infer_base, p_infer)

         move_grid = discovered_grid.copy()
         move_grid[x, y] = 0
         # get window for current position
         window = get_window((x, y), move_grid).flatten()
         window = np.asarray([window])
         #window = normalize(window)
         window = np.asarray([np.append(window, infer_base[_format(x,y)]['cx'])])
         window = sc_X.transform(window)

         return 0, (x, y), window, trajectory_path, final_path, visited, bumps
                

def plan(current, window, model, discovered_grid, trajectory_path, final_path, k_infer, infer_base, p_infer, visited):
    count = {}
    count[_format(current[0], current[1])] = 1
    solved = 0
    n_x = 0
    n_y = 0
    bumps =0

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
        
        status, current, window, trajectory_path, final_path, visited, bumps = execution(direction, current, discovered_grid, trajectory_path, final_path, k_infer, infer_base, p_infer, visited, bumps)
        
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
            if value > 12: 
                return 0, trajectory_path, final_path, bumps
        
# load model
model = keras.models.load_model('My5b5_Ag3_9843_model')

#importing the dataset
dataset = pd.read_csv('ag3_5by5dGrid.csv', names = range(27))
X = dataset.iloc[:, :-1].values
y = dataset.iloc[:, -1].values
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.1, random_state =0)

sc_X = StandardScaler()
X_train = sc_X.fit_transform(X_train)

#take dimensions input
dim_x, dim_y = map(int, input("Enter matrix size for maze (rows x cols): ").split())
n = int(input("Number of repetitions: "))

# writer to write solved grids in a csv
a = open('CNNData/ag1_KerasSolvedGrids.csv', 'w')
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
            #print(maze)

            #free space assumption of discovered grid
            discovered_grid = np.random.choice([1, -1], p=[1, 0], size=(dim_x, dim_y))
            if discovered_grid[0,0] != 1:
                maze[0,0] = 1
            if discovered_grid[dim_x-1,dim_y-1] != 1:
                maze[dim_x-1,dim_y-1] = 1

            infer_base = {}

            p_infer = {'0,0':{'ex'}}
            k_infer = {}

            trajectory_path = []
            final_path = []
            visited = {}
            visited[_format(source_x, source_y)] = 'visited'
            trajectory_path.append(_format(source_x, source_y))
            final_path.append(_format(source_x, source_y))

            sense(source_x, source_y, k_infer, infer_base, p_infer)
            infer(source_x, source_y, k_infer, infer_base, p_infer)

            discovered_grid[source_x, source_y] = 2
            move_grid = discovered_grid.copy()
            move_grid[source_x, source_y] = 0
            # get window for current position
            window = get_window((source_x, source_y), move_grid).flatten()
            window = np.asarray([window])

            window = np.asarray([np.append(window, 0)])
            window = sc_X.transform(window)
            #print(window)

            start = time.time()
            status, trajectory_path, final_path, bumps = plan((source_x, source_y), window, model, discovered_grid, trajectory_path, final_path, k_infer, infer_base, p_infer, visited)
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

            ##print("Trajectory: {}".format(trajectory_path))
            ##print("Final Path: {}".format(final_path))


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

with open('CNNData/AG1_traj.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(dense_traject[0])
        writer.writerow(dense_traject[1])

with open('CNNData/AG1_discpath.csv', 'w', encoding='UTF8', newline='') as g:
        writer = csv.writer(g)
        writer.writerow(dense_discpath[0])
        writer.writerow(dense_discpath[1])

with open('CNNData/AG1_bumps.csv', 'w', encoding='UTF8', newline='') as h:
        writer = csv.writer(h)
        writer.writerow(density_bumps[0])
        writer.writerow(density_bumps[1])

with open('CNNData/AG1_time.csv', 'w', encoding='UTF8', newline='') as i:
        writer = csv.writer(i)
        writer.writerow(density_time[0])
        writer.writerow(density_time[1])

with open('CNNData/AG1_solve.csv', 'w', encoding='UTF8', newline='') as j:
        writer = csv.writer(j)
        writer.writerow(grid_solve[0])
        writer.writerow(grid_solve[1])
