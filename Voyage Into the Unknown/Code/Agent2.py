from queue import PriorityQueue
import numpy as np
from itertools import chain
import csv
import time
import sys

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


def execution(planned_path, discovered_grid, k_base, k_grid, bumps):
    for node in planned_path:
        x = node[0]
        y = node[1]
        
        if maze[x, y] == -1:
            bumps = bumps + 1
            discovered_grid[x, y] = -1
            parentX_current = k_base[_format(x, y)]['p'][0]
            parentY_current = k_base[_format(x, y)]['p'][1]
            heuristic = k_base[_format(x, y)]['h']
            k_grid[_format(x, y)] = {
                'g': sys.maxsize,
                'h': heuristic,
                'p': (parentX_current, parentY_current) 
                }
           
            return 0, parentX_current, parentY_current, bumps
        
        elif (x == dim_x - 1) and (y == dim_y - 1):
            
            p = k_base[_format(x, y)]['p']
            k_grid[_format(x, y)] = {
                    'g': k_base[_format(p[0], p[1])]['g'] + 1,
                    'h': manhattan_distance(x, y),
                    'p': p
                    }
            
            return 1, x, y, bumps
        
        else:
             if _format(x, y) not in k_grid:
                 p = k_base[_format(x, y)]['p']
                 k_grid[_format(x, y)] = {
                    'g': k_base[_format(p[0], p[1])]['g'] + 1,
                    'h': manhattan_distance(x, y),
                    'p': p
                    }

             elif _format(x, y) in k_grid and k_grid[_format(x, y)]['g'] != sys.maxsize:
                 if k_grid[_format(x, y)]['g'] > k_base[_format(x, y)]['g']:
                    k_grid.update({_format(x, y):{
                    'g': k_base[_format(x, y)]['g'],
                    'h': manhattan_distance(x, y),
                    'p': k_base[_format(x, y)]['p']
                    }})

def plan(fringe, discovered_grid, k_base, k_grid):
    trajectory_path= []
    bumps = 0
    processed = 0
    
    while not fringe.empty():
        
        current = fringe.get()
        processed = processed + 1
        
        current_x = current[1]
        current_y = current[2]
        
        if (current_x == dim_x - 1) and (current_y == dim_y - 1):
            
            planned_path = get_path(current_x, current_y, k_base)
            
            status, node_x, node_y, bumps = execution(planned_path, discovered_grid, k_base, k_grid, bumps)
            if status == 1:
                final_path = get_path(node_x, node_y, k_grid)
                print("Final Path: {}", final_path)
                trajectory_path.append(get_path(node_x, node_y, k_base))
                print("Success! Path Found.")
                return 1, trajectory_path, final_path, bumps, processed
            
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
    return 0, trajectory_path, "No Path", bumps, processed
        


#take dimensions input
dim_x, dim_y = map(int, input("Enter matrix size for maze (rows x cols): ").split())
n = int(input("Number of repetitions: "))

dense_traject = [[],[]]
dense_traj_discpath = [[],[]]
dense_discpath = [[],[]]
density_cells = [[],[]]
density_bumps = [[],[]]
density_time = [[],[]]

for i in range(0, 34, 1):
    
    avg_trajectory = 0
    trajectory_cost = 0
    avg_shortest_cost = 0
    shortest_cost = 0
    avg_processed = 0
    cells_proc = 0
    n_bumps = 0
    avg_bumps = 0
    runtime = 0
    avg_runtime = 0
    
    #Run for N repetitions:
    for j in range(1, n+1):
            
        #generate random maze
        maze = np.random.choice([1, -1], p=[1-(i/100), (i/100)], size=(dim_x, dim_y))
        
        #ensure top left and bottom right are not blocked
        if maze[0,0] != 1:
            maze[0,0] = 1
        if maze[dim_x-1,dim_y-1] != 1:
            maze[dim_x-1,dim_y-1] = 1
        
        print(maze)
        
        #initialize source
        source_x = source_y = 0
        heuristic = manhattan_distance(source_x, source_y)
          
        #free space assumption of discovered grid
        discovered_grid = np.random.choice([1, -1], p=[1, 0], size=(dim_x, dim_y))
        if discovered_grid[0,0] != 1:
            maze[0,0] = 1
        if discovered_grid[dim_x-1,dim_y-1] != 1:
            maze[dim_x-1,dim_y-1] = 1
        print(discovered_grid)
        
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
        status, trajectory, final, bumps, processed = plan(fringe, discovered_grid, k_base, k_grid)
        end = time.time()

        runtime = runtime + float("%.7f" % (end-start))
        avg_runtime = runtime/j

        if final == "No Path":
            shortest_cost = shortest_cost + 1
        elif final != "No Path":
            shortest_cost = shortest_cost + (len(final) - 1)
        avg_shortest_cost = shortest_cost/j
        
        flatten_trajectory = list(chain.from_iterable(trajectory))
        trajectory_cost = trajectory_cost + (len(flatten_trajectory) - 1)
        avg_trajectory = trajectory_cost/j
        
        n_bumps = n_bumps + bumps
        avg_bumps = n_bumps/j
        
        cells_proc = cells_proc + processed
        avg_processed = cells_proc/j
        
        k_grid.clear()
        k_base.clear()
        print("Prob: {}, Rep: {}".format(i/100, j))
    
    dense_traj_discpath[0].append(i/100)
    dense_traj_discpath[1].append(avg_trajectory/avg_shortest_cost)
    density_time[0].append(i/100)
    density_time[1].append(avg_runtime)
    dense_traject[0].append(i/100)
    dense_traject[1].append(avg_trajectory)
    dense_discpath[0].append(i/100)
    dense_discpath[1].append(avg_shortest_cost)
    density_cells[0].append(i/100)
    density_cells[1].append(avg_processed)
    density_bumps[0].append(i/100)
    density_bumps[1].append(avg_bumps)
    
    with open('/Users/prashantkanth/Desktop/Intro to AI-CS520/Projects/Partial Sensing/Output Data/AG2_density_v_traj.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(dense_traject[0])
        writer.writerow(dense_traject[1])
    
    with open('/Users/prashantkanth/Desktop/Intro to AI-CS520/Projects/Partial Sensing/Output Data/AG2_density_v_discpath.csv', 'w', encoding='UTF8', newline='') as g:
        writer = csv.writer(g)
        writer.writerow(dense_discpath[0])
        writer.writerow(dense_discpath[1])
        
    with open('/Users/prashantkanth/Desktop/Intro to AI-CS520/Projects/Partial Sensing/Output Data/AG2_density_v_bumps.csv', 'w', encoding='UTF8', newline='') as h:
        writer = csv.writer(h)
        writer.writerow(density_bumps[0])
        writer.writerow(density_bumps[1])
    
    with open('/Users/prashantkanth/Desktop/Intro to AI-CS520/Projects/Partial Sensing/Output Data/AG2_density_v_time.csv', 'w', encoding='UTF8', newline='') as i:
        writer = csv.writer(i)
        writer.writerow(density_time[0])
        writer.writerow(density_time[1])

    with open('/Users/prashantkanth/Desktop/Intro to AI-CS520/Projects/Partial Sensing/Output Data/AG2_density_v_proc_cell.csv', 'w', encoding='UTF8', newline='') as k:
        writer = csv.writer(k)
        writer.writerow(density_cells[0])
        writer.writerow(density_cells[1])

    with open('/Users/prashantkanth/Desktop/Intro to AI-CS520/Projects/Partial Sensing/Output Data/AG2_density_v_traj_discpath.csv', 'w', encoding='UTF8', newline='') as m:
        writer = csv.writer(m)
        writer.writerow(dense_traj_discpath[0])
        writer.writerow(dense_traj_discpath[1])
