from queue import PriorityQueue
import numpy as np
from itertools import chain
import csv
import time
import gridSolvable
import sys
import pandas as pd

#function to calculate manhattan distance (used to calculate heuristics)
def manhattan_distance(ag_X, ag_Y):
    dist = abs(ag_X - (dim_x-1)) + abs(ag_Y - (dim_y-1))
    return dist

#get formatted 'child_x,childy' to easily search in closed_knowledge
def _format(child_x, child_y):
    return str(child_x) + "," + str(child_y)

#return path
def get_path(goal_X, goal_Y, grid):
    discovered_path = [(goal_X, goal_Y)]
    while grid[_format(discovered_path[-1][0], discovered_path[-1][1])]['p'] != (None, None):
        discovered_path.append(grid[_format(discovered_path[-1][0], discovered_path[-1][1])]['p'])
    discovered_path.reverse()
    return discovered_path

#generate valid child and include in knowledge set and fringe
def valid_child(child_x , child_y, current):
    heuristic = manhattan_distance(child_x, child_y)
    child_cost = k_base[_format(current[1], current[2])]['g'] + 1
    dist = child_cost + heuristic
    
    #if not present in knowledge grid, generate child and insert with appropriate calculations,
    #insert in fringe
    if _format(child_x, child_y) not in k_base:
        k_base[_format(child_x, child_y)] = {
            'g': child_cost,
            'h': heuristic,
            'p': (current[1], current[2])
        }
        fringe.put((dist, child_x, child_y))
        
    #if encountered cost of child is less than what 
    #we have in knowledge grid, update the child information
    elif _format(child_x, child_y) in k_base:
        if k_base[_format(child_x, child_y)]['g'] > child_cost:
            k_base.update({_format(child_x, child_y):{
            'g': child_cost,
            'h': heuristic,
            'p': (current[1], current[2])
            }})
            fringe.put((dist, child_x, child_y))

def exec_grid_valid_child(child_x , child_y, current):    
    #discovered grid, keep updating as cells are encountered, keeping track of min cost of child
    discovered_grid[child_x, child_y] = 3
    heuristic = manhattan_distance(child_x, child_y)
    child_cost = k_base[_format(current[0], current[1])]['g'] + 1
    if _format(child_x, child_y) not in k_grid:
        if k_grid[_format(current[0], current[1])]['g'] < k_base[_format(current[0], current[1])]['g']:
            k_grid[_format(child_x, child_y)] = {
                'g': k_grid[_format(current[0], current[1])]['g'] + 1,
                'h': manhattan_distance(child_x, child_y),
                'p': (current[0], current[1])
            }
        else:
            k_grid[_format(child_x, child_y)] = {
                'g': child_cost,
                'h': manhattan_distance(child_x, child_y),
                'p': (current[0], current[1])
            }
            
        
    elif _format(child_x, child_y) in k_grid and k_grid[_format(child_x, child_y)]['g'] != sys.maxsize:
        if k_grid[_format(child_x, child_y)]['g'] > child_cost:
            k_grid.update({_format(child_x, child_y):{
            'g': child_cost,
            'h': heuristic,
            'p': (current[0], current[1])
            }})

#define blocked cells, include in knowledge set
def blocked_child(child_x, child_y, current):
    discovered_grid[child_x, child_y] = -1
    heuristic = manhattan_distance(child_x, child_y)
    child_cost = k_base[_format(current[0], current[1])]['g'] + 1
    dist = child_cost + heuristic
    if _format(child_x, child_y) not in k_grid:
        k_grid[_format(child_x, child_y)] = {
                'g': sys.maxsize,
                'h': heuristic,
                'p': (current[0], current[1]) 
        }
    #second occurence of same blocked cell,
    #update parent to help in replanning if required
    elif _format(child_x, child_y) in k_grid:
        k_grid[_format(child_x, child_y)] = {
            'g': sys.maxsize,
            'h': heuristic,
            'p': (current[0], current[1])
        }


def execution(planned_path, discovered_grid, k_base, k_grid, movements, flag):
    for node in planned_path:
        x = node[0]
        y = node[1]
        if maze[x, y] == -1:
            discovered_grid[x, y] = -1
            parentX_current = k_grid[_format(x, y)]['p'][0]
            parentY_current = k_grid[_format(x, y)]['p'][1]
            heuristic = k_grid[_format(x, y)]['h']
            k_grid[_format(x, y)] = {
                'g': sys.maxsize,
                'h': heuristic,
                'p': (parentX_current, parentY_current) 
                }
            # move_grid = discovered_grid.copy()
            # move_grid[parentX_current, parentY_current] = 0
            # writer.writerow(move_grid.flatten().tolist())
            return 0, parentX_current, parentY_current, flag
        
        elif (x == dim_x - 1) and (y == dim_y - 1):
            flag += 1
            move_grid = discovered_grid.copy()
            move_grid[x, y] = 0
            movements[flag] = move_grid.flatten().tolist()
            if len(movements) > 1:
                if movements[flag-1].index(0) == movements[flag].index(0) + 1:
                    movements[flag-1].append("left")
                elif movements[flag-1].index(0) == movements[flag].index(0) - 1:
                    movements[flag-1].append("right")
                elif movements[flag-1].index(0) == movements[flag].index(0) + dim_x:
                    movements[flag-1].append("up")
                elif movements[flag-1].index(0) == movements[flag].index(0) - dim_x:
                    movements[flag-1].append("down")
            return 1, x, y, flag
        
        else:
            discovered_grid[x, y] = 2

            #blocked (x - 1) child
            if x > 0 and maze[x - 1, y] == -1:
                blocked_child(x - 1, y, (x, y))
            #blocked (x + 1) child
            if x < dim_x-1 and maze[x + 1, y] == -1:
                blocked_child(x + 1, y, (x, y))
            #blocked (y + 1) child
            if y < dim_y-1 and maze[x, y + 1] == -1:
                blocked_child(x, y + 1, (x, y))
            #blocked (y - 1) child
            if y > 0 and maze[x, y - 1] == -1:
                blocked_child(x, y - 1, (x, y))
            #unblocked (x - 1) child
            if x > 0 and discovered_grid[x - 1, y] != -1:
                exec_grid_valid_child(x - 1, y, (x, y))
           
            #unblocked (x + 1) child
            if x < dim_x-1 and discovered_grid[x + 1, y] != -1:
                exec_grid_valid_child(x + 1, y, (x, y))
           
            #unblocked (y + 1) child
            if y < dim_y-1 and discovered_grid[x, y + 1] != -1:
                exec_grid_valid_child(x, y + 1, (x, y))
           
            #unblocked (y - 1) child
            if y > 0 and discovered_grid[x, y - 1] != -1:
                exec_grid_valid_child(x, y - 1, (x, y))
            
            flag += 1
            move_grid = discovered_grid.copy()
            move_grid[x, y] = 0
            movements[flag] = move_grid.flatten().tolist()
            if len(movements) > 1:
                if movements[flag-1].index(0) == movements[flag].index(0) + 1:
                    movements[flag-1].append("left")
                elif movements[flag-1].index(0) == movements[flag].index(0) - 1:
                    movements[flag-1].append("right")
                elif movements[flag-1].index(0) == movements[flag].index(0) + dim_x:
                    movements[flag-1].append("up")
                elif movements[flag-1].index(0) == movements[flag].index(0) - dim_x:
                    movements[flag-1].append("down")
                

def plan(fringe, discovered_grid, k_base, k_grid, movements, flag):
    trajectory_path= []
    
    while not fringe.empty():
        
        current = fringe.get()
        
        current_x = current[1]
        current_y = current[2]
        
        if (current_x == dim_x - 1) and (current_y == dim_y - 1):
            
            planned_path = get_path(current_x, current_y, k_base)
            #print("Planned Path:", planned_path)
            status, node_x, node_y, flag = execution(planned_path, discovered_grid, k_base, k_grid, movements, flag)
            if status == 1:
                final_path = get_path(node_x, node_y, k_grid)
                #print("Final Path: {}", final_path)
                trajectory_path.append(get_path(node_x, node_y, k_base))
                print("Success! Path Found.")
                return 1, trajectory_path, final_path, flag
            
            else:
                trajectory_path.append(get_path(node_x, node_y, k_base))
                k_base.clear()
                while not fringe.empty():
                    fringe.get()
                    
                fn = k_grid[_format(node_x, node_y)]['g'] + k_grid[_format(node_x, node_y)]['h']
                fringe.put((fn, node_x, node_y))
                
                k_base[_format(node_x, node_y)] = {
                'g': k_grid[_format(node_x, node_y)]['g'],
                'h': k_grid[_format(node_x, node_y)]['h'],
                'p': (None, None)
                }
                
                
        else:
            #unblocked (x - 1) child
            if current_x > 0 and discovered_grid[current_x - 1, current_y] != -1:
                valid_child(current_x - 1, current_y, current)
           
            #unblocked (x + 1) child
            if current_x < dim_x-1 and discovered_grid[current_x + 1, current_y] != -1:
                valid_child(current_x + 1, current_y, current)
           
            #unblocked (y + 1) child
            if current_y < dim_y-1 and discovered_grid[current_x, current_y + 1] != -1:
                valid_child(current_x, current_y + 1, current)
           
            #unblocked (y - 1) child
            if current_y > 0 and discovered_grid[current_x, current_y - 1] != -1:
                valid_child(current_x, current_y - 1, current)
    
    trajectory_path.append(get_path(current_x, current_y, k_base))
    print("No Path Found!!")
    return 0, trajectory_path, "No Path", flag
        

#take dimensions input
dim_x, dim_y = map(int, input("Enter matrix size for maze (rows x cols): ").split())
n = int(input("Number of repetitions: "))
solve_counter = 0
b = open('ag1_trueGrid.csv', 'w')
writer_b = csv.writer(b)
a = open('ag1_dGrid.csv', 'w')
writer = csv.writer(a)

#Run for N repetitions:
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
        writer_b.writerow(maze.flatten().tolist())

        print("Maze is solvable:",solve_counter)
        #print(maze)

        heuristic = manhattan_distance(source_x, source_y)

        #free space assumption of discovered grid
        discovered_grid = np.random.choice([1, -1], p=[1, 0], size=(dim_x, dim_y))
        if discovered_grid[0,0] != 1:
            maze[0,0] = 1
        if discovered_grid[dim_x-1,dim_y-1] != 1:
            maze[dim_x-1,dim_y-1] = 1

        flag = 0
        movements = {}
        k_grid = {'0,0': {
                    'g': 0,
                    'h': heuristic,
                    'p': (None, None)
                    }
        }

        fringe = PriorityQueue()
        fringe.put((heuristic, source_x, source_y))

        k_base = {'0,0': {
                    'g': 0,
                    'h': heuristic,
                    'p': (None, None)
                    }
        }

        start = time.time()
        status, trajectory, final, flag = plan(fringe, discovered_grid, k_base, k_grid, movements, flag)
        end = time.time()
        
        #movements[flag].append("reached")
        for key in movements:
            writer.writerow(movements[key])

    else:
        print("** Maze is not solvable ** Generating again ...")

a.close()
b.close()
print("Reading source data")
df = pd.read_csv('ag1_dGrid.csv', names=range((dim_x*dim_y) + 1))
print("Removing NA")
df.dropna(inplace = True)
df.to_csv('ag1_newdGrid.csv', header=False, index = False)
print("Data Cleaned")