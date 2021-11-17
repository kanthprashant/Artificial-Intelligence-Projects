from queue import PriorityQueue

# function to calculate manhattan distance (used to calculate heuristics)
def manhattan_distance(cell, target):
    dist = abs(target[0] - cell[0]) + abs(target[1] - cell[1])
    return dist

# helper function to get string formatted 'child_x,childy'
def _format(child_x, child_y):
    return str(child_x) + "," + str(child_y)

def get_path(goal_X, goal_Y, grid):
    discovered_path = [(goal_X, goal_Y)]
    while grid[_format(discovered_path[-1][0], discovered_path[-1][1])]['p'] != (None, None):
        discovered_path.append(grid[_format(discovered_path[-1][0], discovered_path[-1][1])]['p'])
    discovered_path.reverse()
    return discovered_path

def full_valid_child(child_x , child_y, current, full_fringe, full_closed_knowledge, target):
    heuristic = manhattan_distance((child_x, child_y), target)
    child_cost = full_closed_knowledge[_format(current[1], current[2])]['g'] + 1
    dist = child_cost + heuristic
    if _format(child_x, child_y) not in full_closed_knowledge:
        full_closed_knowledge[_format(child_x, child_y)] = {
            'g': child_cost,
            'h': heuristic,
            'p': (current[1], current[2])
        }
        full_fringe.put((dist, child_x, child_y))
        
    elif _format(child_x, child_y) in full_closed_knowledge:
        if full_closed_knowledge[_format(child_x, child_y)]['g'] > child_cost:
            full_closed_knowledge.update({_format(child_x, child_y):{
            'g': child_cost,
            'h': heuristic,
            'p': (current[1], current[2])
            }})

def target_search(maze, agent, target):
    row, col = maze.shape
    #define fringe
    full_fringe = PriorityQueue()
    
    heuristic = manhattan_distance(agent, target)
    full_fringe.put((heuristic, agent[0], agent[1]))
    full_closed_knowledge = {_format(agent[0],agent[1]):{
            'g': 0,
            'h': heuristic,
            'p': (None, None)
            }
        }
    
    while not full_fringe.empty():
        current = full_fringe.get()
        current_x = current[1]
        current_y = current[2]
        
        if current_x == target[0] and current_y == target[1]:
            full_grid_path = get_path(current_x, current_y, full_closed_knowledge)
            return True, full_grid_path
        
        else:
            #unblocked (x - 1) child
            if current_x > 0 and maze[current_x - 1, current_y] != -1:
                full_valid_child(current_x - 1, current_y, current, full_fringe, full_closed_knowledge, target)
                
            #unblocked (x + 1) child
            if current_x < row-1 and maze[current_x + 1, current_y] != -1:
                full_valid_child(current_x + 1, current_y, current, full_fringe, full_closed_knowledge, target)
                
            #unblocked (y + 1) child
            if current_y < col-1 and maze[current_x, current_y + 1] != -1:
                full_valid_child(current_x, current_y + 1, current, full_fringe, full_closed_knowledge, target)
                
            #unblocked (y - 1) child
            if current_y > 0 and maze[current_x, current_y - 1] != -1:
                full_valid_child(current_x, current_y - 1, current, full_fringe, full_closed_knowledge, target)
    
    return False, "Not Found"
        
        