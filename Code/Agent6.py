# AGENT-6

from queue import PriorityQueue
import gridSolvable
import numpy as np
from itertools import chain
import sys

# terrain identifier
terrain_id = {1:'flat', 2:'hilly', 3:'forest'}

# function to calculate manhattan distance (used to calculate heuristics)
def manhattan_distance(cell, agent):
    dist = abs(agent[0] - cell[0]) + abs(agent[1] - cell[1])
    return dist

# helper function to get string formatted 'child_x,childy'
def _format(child_x, child_y):
    return str(child_x) + "," + str(child_y)

# function to provide shortest traversed path
def get_path(goal_X, goal_Y, grid):
    discovered_path = [(goal_X, goal_Y)]
    while grid[_format(discovered_path[-1][0], discovered_path[-1][1])]['p'] != (None, None):
        discovered_path.append(grid[_format(discovered_path[-1][0], discovered_path[-1][1])]['p'])
    discovered_path.reverse()
    return discovered_path

# function to initialize probabilistic model
def _init_model(pb_model, dim):
    for x in range(dim[0]):
        for y in range(dim[1]):
            pb_model[_format(x,y)] = 1/(dim[0] * dim[1])

# function to randomly generate result for a examined cell
def get_result(p_false):
    n = np.random.choice([0, 1], p = [p_false, 1-p_false])
    return n

# generate valid child and include in knowledge set and fringe
def valid_child(child_x , child_y, current, target):
    heuristic = manhattan_distance(target, (child_x, child_y))
    child_cost = k_base[_format(current[1], current[2])]['g'] + 1
    dist = child_cost + heuristic
    
    # if not present in knowledge grid, generate child and insert with appropriate calculations,
    # insert in fringe
    if _format(child_x, child_y) not in k_base:
        k_base[_format(child_x, child_y)] = {
            'g': child_cost,
            'p': (current[1], current[2])
        }
        fringe.put((dist, child_x, child_y))
        
    # if encountered cost of child is less than what 
    # we have in knowledge grid, update the child information
    elif _format(child_x, child_y) in k_base:
        if k_base[_format(child_x, child_y)]['g'] > child_cost:
            k_base.update({_format(child_x, child_y):{
            'g': child_cost,
            'p': (current[1], current[2])
            }})
            fringe.put((dist, child_x, child_y))

# function to update pb_model if blocked or no path
def discard_cell(pb_model, target):
    # update Px,y
    pb_model[_format(target[0], target[1])] = 0
    
    # update Pi,j(containing target)
    total = sum(pb_model.values())
    for key in pb_model:
        if key != _format(target[0], target[1]):
            pb_model[key] = pb_model[key]/total

# function to examine target
def examine_target(pb_model, false_terrain, target, terrain):
    result = 0
    
    # check if originally target was here, if it was, 
    # use false negative to get result and then update Px,y(containing target)
    if target[0] == tx and target[1] == ty:
        result = get_result(false_terrain[terrain])
        # if target found, set Px,y = 1 and all other Pi,j = 0
        if result == 1:
            pb_model[_format(target[0], target[1])] = 1
            for key in pb_model:
                if key != _format(target[0], target[1]):
                    pb_model[key] = 0
            return result
        # if target not found, update Px,y(containing target)
        else:
            pb_model[_format(target[0], target[1])] = pb_model[_format(target[0], target[1])] * false_terrain[terrain]
    
    # if target didn't exist here in true maze, update Px,y(containing target)
    else:
        pb_model[_format(target[0], target[1])] = pb_model[_format(target[0], target[1])] * false_terrain[terrain]
        
    # update Pi,j(containing target)
    total = sum(pb_model.values())
    for key in pb_model:
        if key != _format(target[0], target[1]):
            pb_model[key] = pb_model[key]/total
            
    return result
         
# function to identify target cell
def target_xy(pb_model, agent):
    # get max p value
    _pMax = max(pb_model.values())
    
    # get all cells with p = _pMax
    pmax_cells = list()
    for key, value in pb_model.items():
        if value == _pMax:
            pmax_cells.append(key)
    
    # if multiple cells with p = _pMax, find least distant among those
    if len(pmax_cells) == 1:
        return pmax_cells[0], _pMax
    else:
        edist_cells = list()
        min_dist = sys.maxsize
        for cell in pmax_cells:
            x, y = map(int, cell.split(","))
            xy_dist = manhattan_distance((x,y), agent)
            if xy_dist < min_dist:
                edist_cells = [cell]
                min_dist = xy_dist
            elif xy_dist == min_dist:
                edist_cells.append(cell)

    # if multiple equi-distant cells, randomly select one cell as target
    if len(edist_cells) == 1:
        return edist_cells[0], _pMax
    else:
        return np.random.choice(edist_cells), _pMax

# function to implement execution phase
def execute(planned_path, d_grid, target, t_pMax, k_base, exams, bumps):
    for node in planned_path:
        current_x = node[0]
        current_y = node[1]
        
        # if block encountered, update pb_model, check max Pi,j and identify if target needs to be changed
        if maze[current_x, current_y] == -1:
            # update discovered grid
            d_grid[current_x, current_y] = -1
            # get details of parent for blocked cell to make it new source
            parent_x = k_base[_format(current_x, current_y)]['p'][0]
            parent_y = k_base[_format(current_x, current_y)]['p'][1]
            parent_g = k_base[_format(parent_x, parent_y)]['g']
            parent_h = manhattan_distance(target, (parent_x, parent_y))
            # update probabilities
            discard_cell(pb_model, (current_x, current_y))
            # take bump count into consideration for total action
            bumps += 1
            
            # check Px,y of target, if dropped to 0, change target [if identified target was block]
            if pb_model[_format(target[0], target[1])] == 0:
                return 1,0,exams,bumps,(parent_x, parent_y),target,t_pMax
                # target_x, target_y = map(int, target_xy(pb_model, (parent_x, parent_y)).split(","))
                # target = (target_x, target_y)
                # parent_h = manhattan_distance(target, (parent_x, parent_y))
            
            # check if pMax changed midway, if it did, shift target and plan a path to new target
            temp_targ, temp_pMax = target_xy(pb_model, (parent_x, parent_y))
            if (temp_pMax > t_pMax) and (temp_targ != _format(target[0], target[1])):
                targ_x, targ_y = map(int, temp_targ.split(","))
                print("Old Traget {}, pMax: {}\n New Target: {}, pMax:{}".format(target,pb_model[_format(target[0], target[1])],temp_targ,temp_pMax))
                t_pMax = temp_pMax
                target = (targ_x, targ_y)
                print("Identified target shifted:{}\nRerouting..".format(target))
            
            return 0,0,exams,bumps,(parent_g, parent_h, parent_x, parent_y),target,t_pMax
        
        # if reached target, examine cell
        elif (current_x == target[0]) and (current_y == target[1]):
            # update terrain on discovered grid
            d_grid[current_x, current_y] = maze[current_x, current_y]
            # examine cell given terrain type
            terrain = terrain_id[maze[current_x, current_y]]
            res = examine_target(pb_model, false_terrain, target, terrain)
            # increment examine count by 1
            exams += 1
            
            # if examine succeded
            if res == 1:
                return 1,1,exams,bumps,(current_x, current_y),target,t_pMax
            #if examine failed
            else:
                return 1,0,exams,bumps,(current_x, current_y),target,t_pMax
            
        # traverse the planned path
        else:
            # update terrain type on discovered grid
            d_grid[current_x, current_y] = maze[current_x, current_y]
            # check if pMax changed midway, if it did, shift target and plan a path to new target
            temp_targ, temp_pMax = target_xy(pb_model, (current_x, current_y))
            if (temp_pMax > t_pMax) and (temp_targ != _format(target[0], target[1])):
                targ_x, targ_y = map(int, temp_targ.split(","))
                print("Old Traget {}, pMax: {}\n New Target: {}, pMax:{}".format(target,pb_model[_format(target[0], target[1])],temp_targ,temp_pMax))
                t_pMax = temp_pMax
                target = (targ_x, targ_y)
                g = k_base[_format(current_x, current_y)]['g']
                h = manhattan_distance(target, (current_x, current_y))
                print("Identified target shifted:{}\nRerouting..".format(target))
                return 0,0,exams,bumps,(g, h, current_x, current_y),target,t_pMax


# function to implement planning phase
def plan(fringe, target, t_pMax, d_grid, k_base):
    # variable to capture movement on one planned path
    trajectory_path = []
    # examine + bump counts
    exams = 0
    bumps = 0
    while not fringe.empty():
        # get first element from fringe
        current = fringe.get()
        current_x = current[1]
        current_y = current[2]
        
        # if reached target on discovered grid while planning 
        if (current_x == target[0]) and (current_y == target[1]):
            # get route of planned path
            planned_path = get_path(current_x, current_y, k_base)
            
            # execute planned path
            r_idTarget, f_target, exams, bumps, node, target, t_pMax = execute(planned_path, d_grid, target, t_pMax, k_base, exams, bumps)
            
            # if block encountered before reaching identified target
            if r_idTarget == 0:
                trajectory_path.append(get_path(node[2], node[3], k_base))
                # reset k_base and fringe to new source
                k_base.clear()
                while not fringe.empty():
                    fringe.get()
                
                k_base[_format(node[2], node[3])] = {
                                'g': node[0],
                                'p': (None, None)
                                }
                
                fringe.put((node[0] + node[1], node[2], node[3]))
                
            # if reached identified target but examine failed
            elif (r_idTarget == 1) and (f_target == 0):
                trajectory_path.append(get_path(node[0], node[1], k_base))
                return 0, trajectory_path, exams, bumps, node
            
            # if reached identified target and examine succeeded
            elif (r_idTarget == 1) and (f_target == 1):
                trajectory_path.append(get_path(node[0], node[1], k_base))
                print("Target Found!!")
                return 1, trajectory_path, exams, bumps, node
            
            
        else:
            # unblocked (x - 1) child
            if current_x > 0 and d_grid[current_x - 1, current_y] != -1:
                valid_child(current_x - 1, current_y, current, target)
           
            # unblocked (x + 1) child
            if current_x < dim_x-1 and d_grid[current_x + 1, current_y] != -1:
                valid_child(current_x + 1, current_y, current, target)
           
            # unblocked (y + 1) child
            if current_y < dim_y-1 and d_grid[current_x, current_y + 1] != -1:
                valid_child(current_x, current_y + 1, current, target)
           
            # unblocked (y - 1) child
            if current_y > 0 and d_grid[current_x, current_y - 1] != -1:
                valid_child(current_x, current_y - 1, current, target)
    
    # if there is no path to identified target
    node = get_path(current_x, current_y, k_base)
    # last position of agent from where no path was planned for execution phase
    trajectory_path.append(node[0])
    # set Px,y of target to 0 and update pb_model
    # unable to reach target, we can discard without examination, no increment in count
    discard_cell(pb_model, target)
    return 0, trajectory_path, exams, bumps, node[0]
    

# take dimension input
dim_x, dim_y = map(int, input("Enter matrix size for maze (rows x cols): ").split())
# density of blocks (-1)
p0 = 0.3

# -1 - block
#  1 - flat terrain
#  2 - hilly terrain
#  3 - forest terrain
maze = np.random.choice([-1, 1, 2, 3], p=[p0, ((1-p0)/3), ((1-p0)/3), ((1-p0)/3)], size=(dim_x, dim_y))

# randomly generate agent's position
ax = np.random.randint(dim_x-1)
ay = np.random.randint(dim_y-1)

# randomly generate target's position
tx = np.random.randint(dim_x-1)
ty = np.random.randint(dim_y-1)

# ensure that agent and target are not at blocked cell
if maze[ax,ay] == -1:
    maze[ax,ay] = np.random.choice([1, 2, 3], p=[1/3, 1/3, 1/3])
if maze[tx,ty] == -1:
    maze[tx,ty] = np.random.choice([1, 2, 3], p=[1/3, 1/3, 1/3])

# show initial maze
print(maze)
# tell agent's initial position
print("Agent spawned at: ({},{}), Terrain: {}".format(ax,ay,terrain_id[maze[ax,ay]]))
# tell original target's position
print("Original Target at: ({},{}), Terrain: {}".format(tx,ty,terrain_id[maze[tx,ty]]))

# Find if maze is solvable
solvable, path = gridSolvable.target_search(maze, (ax,ay), (tx,ty))

# if solvable, start processing
if solvable:
    print("Grid is solvable, Path: {}\n".format(path))
    
    #free space assumption of discovered grid
    d_grid = np.ones((dim_x, dim_y), dtype = int)
    
    #Initialize false negative probabilities
    false_terrain = {
        'flat': 0.2,
        'hilly': 0.5,
        'forest': 0.8
        }
    
    # initialize probabilistic model
    pb_model = {}
    _init_model(pb_model, (dim_x, dim_y))
    
    # store agent's position
    agent_x, agent_y = ax, ay
    
    #set flag = false, run loop until target is found, then set flag = true
    #one loop for one identified target based on max Pi,j
    flag = False
    ag6_moves = 0
    ag6_exams = 0
    ag6_cost = 0
    while not(flag):
        #used in planning phase for movement, resets on block and after one planning completes
        k_base = {_format(agent_x, agent_y): {
                        'g': 0,
                        'p': (None, None)
                        }
            }
        
        # Identify probable target position, returns taerget cell as string and pMax of all cells
        target, t_pMax = target_xy(pb_model, (agent_x, agent_y))
        targ_x, targ_y = map(int, target.split(","))
        print("Target identiifed at position: ({})".format(target))
        
        # Initialise fringe
        heuristic = manhattan_distance((targ_x, targ_y), (agent_x, agent_y))
        dist = k_base[_format(agent_x,agent_y)]['g'] + heuristic
        fringe = PriorityQueue()
        fringe.put((dist, agent_x, agent_y))
        
        # plan a path to identified target and execute
        p_status, trajectory, exams, bumps, agent_p = plan(fringe, (targ_x, targ_y), t_pMax, d_grid, k_base)
        
        # current position of agent after one execution
        agent_x, agent_y = agent_p[0], agent_p[1]
        print("Agent reached: ({},{})\n".format(agent_x, agent_y))
        
        # keep counting total action = movement + examination
        flatten_trajectory = list(chain.from_iterable(trajectory))
        ag6_moves = ag6_moves + (len(flatten_trajectory) - 1) + bumps
        ag6_exams = ag6_exams + exams
        ag6_cost = ag6_moves + ag6_exams
        
        # if target found, set flag = true 
        #else identify new target at the beginning of while loop
        if p_status == 1:
            print("Target's original position: ({},{})".format(tx,ty))
            print("Agent's position: ({},{})".format(agent_p[0], agent_p[1]))
            print("Movements: {}, Examinations: {}, Total action: {}".format(ag6_moves, ag6_exams, ag6_cost))
            flag = True

# grid is unsolvable, generate again
else:
    print("Grid is unsolvable")
