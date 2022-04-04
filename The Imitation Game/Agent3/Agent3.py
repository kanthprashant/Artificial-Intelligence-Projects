from queue import PriorityQueue
import numpy as np
from itertools import chain
import gridSolvable
import csv
import time
import sys
import pandas as pd

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
    while _format(x, y) != 'None,None' and level >= 0 and k_grid[_format(x, y)]['visited'] == 1: 
        
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

        x = k_grid[_format(temp_x, temp_y)]['p'][0]
        y = k_grid[_format(temp_x, temp_y)]['p'][1]
        if x == None and y == None:
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


def execution(planned_path, discovered_grid, k_base, k_grid, k_infer, infer_base, p_infer, movements, flag):
    for node in planned_path:
        x = node[0]
        y = node[1]
        #print("Current position: {}".format(node))
        if maze[x, y] == -1:
            discovered_grid[x, y] = -1
            parentX_current = k_base[_format(x, y)]['p'][0]
            parentY_current = k_base[_format(x, y)]['p'][1]
            heuristic = k_base[_format(x, y)]['h']
            k_grid[_format(x, y)] = {
                'g': sys.maxsize,
                'h': heuristic,
                'p': (parentX_current, parentY_current),
                'i': 'bx',
                'visited': 1
                }
            p_infer[_format(x,y)] = {'bx'}
            
            infer(parentX_current, parentY_current, k_infer, infer_base, p_infer)
            
            return 0, parentX_current, parentY_current, flag
        
        elif (x == dim_x - 1) and (y == dim_y - 1):
            p = k_base[_format(x, y)]['p']
            k_grid[_format(x, y)] = {
                    'g': k_base[_format(p[0], p[1])]['g'] + 1,
                    'h': manhattan_distance(x, y),
                    'p': p,
                    'i': 'ex',
                    'visited': 1
                    }
            p_infer[_format(x,y)] = {'ex'}
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
             parentX_current = k_base[_format(x, y)]['p'][0]
             parentY_current = k_base[_format(x, y)]['p'][1] 
            
             if _format(x, y) not in k_grid:
                 p = k_base[_format(x, y)]['p']
                 k_grid[_format(x, y)] = {
                    'g': k_base[_format(p[0], p[1])]['g'] + 1,
                    'h': manhattan_distance(x, y),
                    'p': p,
                    'i': 'ex',
                    'visited': 1
                    }
                 p_infer[_format(x,y)] = {'ex'}
                 
             elif _format(x, y) in k_grid and k_grid[_format(x, y)]['g'] != sys.maxsize:
                 if k_grid[_format(x, y)]['g'] > k_base[_format(x, y)]['g']:
                    k_grid.update({_format(x, y):{
                        'g': k_base[_format(x, y)]['g'],
                        'h': manhattan_distance(x, y),
                        'p': k_base[_format(x, y)]['p'],
                        'i': 'ex',
                        'visited': 1
                        }})
                    p_infer[_format(x,y)] = {'ex'}
                    
             sense(x, y, k_infer, infer_base, p_infer)
             infer(x, y, k_infer, infer_base, p_infer)
             
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

             #check in advance if any blocked identified in planned path
             for bl in range(planned_path.index(node),len(planned_path)):
                b_x, b_y = planned_path[bl][0], planned_path[bl][1]
                if discovered_grid[b_x, b_y] == -1:
                    #print("Position: {}, Block inferred at {}, Rerouting..".format(node, planned_path[bl]))
                    return 0, x, y, flag
                

def plan(fringe, discovered_grid, k_base, k_grid, k_infer, infer_base, p_infer, movements, flag):
    trajectory_path= []
    
    while not fringe.empty():
        
        current = fringe.get()
        
        current_x = current[1]
        current_y = current[2]
        
        if (current_x == dim_x - 1) and (current_y == dim_y - 1):
            
            planned_path = get_path(current_x, current_y, k_base)
            #print("planned path: {}".format(planned_path))
            status, node_x, node_y, flag = execution(planned_path, discovered_grid, k_base, k_grid, k_infer, infer_base, p_infer, movements, flag)
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
b = open('ag3_trueGrid.csv', 'w')
writer_b = csv.writer(b)
a = open('ag3_dGrid.csv', 'w')
writer = csv.writer(a)

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
                    'p': (None, None),
                    'i': 'ex',
                    'visited': 1
                    }
        }

        infer_base = {}

        fringe = PriorityQueue()
        fringe.put((heuristic, source_x, source_y))

        k_base = {'0,0': {
                    'g': 0,
                    'h': heuristic,
                    'p': (None, None),
                    'i': 'ex',
                    'visited': 1
                    }
        }
        p_infer = {'0,0':{'ex'}}
        k_infer = {}

        start = time.time()
        status, trajectory, final, flag = plan(fringe, discovered_grid, k_base, k_grid, k_infer, infer_base, p_infer, movements, flag)
        end = time.time()

        #movements[flag].append("reached")
        for key in movements:
            writer.writerow(movements[key])

        k_grid.clear()
        k_base.clear()
        p_infer.clear()
        k_infer.clear()
        infer_base.clear()

    else:
        print("** Maze is not solvable ** Generating again ...")

a.close()
b.close()
print("Reading source data")
df = pd.read_csv('ag3_dGrid.csv', names=range(2501))
print("Removing NA")
df.dropna(inplace = True)
df.to_csv('ag3_newdGrid.csv', header=False, index = False)
print("Data Cleaned")