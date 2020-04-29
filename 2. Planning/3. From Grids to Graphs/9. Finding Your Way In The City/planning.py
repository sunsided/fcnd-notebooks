from enum import Enum
from queue import PriorityQueue


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    LEFT = (-1, 0, 10)
    RIGHT = (1, 0, 10)
    UP = (0, -1, 10)
    DOWN = (0, 1, 10)
    
    UP_RIGHT = (1, -1, 14)
    UP_LEFT = (-1, -1, 14)
    DOWN_RIGHT = (1, 1, 14)
    DOWN_LEFT = (-1, 1, 14)

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
        elif self == self.UP_RIGHT:
            return '/'
        elif self == self.DOWN_LEFT:
            return '/'
        elif self == self.UP_LEFT:
            return '\\'
        elif self == self.DOWN_RIGHT:
            return '\\'

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, 
             Action.RIGHT, Action.UP_RIGHT, Action.DOWN_RIGHT,
             Action.LEFT, Action.UP_LEFT, Action.DOWN_LEFT,
             Action.DOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle
    
    off_left = x - 1 < 0
    off_right = x + 1 > n
    
    off_top = y - 1 < 0
    off_bottom = y + 1 > m

    if off_left or grid[x - 1, y] == 1:
        valid.remove(Action.LEFT)
    if off_right or grid[x + 1, y] == 1:
        valid.remove(Action.RIGHT)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.UP)
    if off_bottom or grid[x, y + 1] == 1:
        valid.remove(Action.DOWN)
        
    if off_right or off_top or grid[x + 1, y - 1] == 1:
        valid.remove(Action.UP_RIGHT)
    if off_left or off_top or grid[x - 1, y - 1] == 1:
        valid.remove(Action.UP_LEFT)
    if off_right or off_bottom or grid[x + 1, y + 1] == 1:
        valid.remove(Action.DOWN_RIGHT)
    if off_left or off_bottom or grid[x - 1, y + 1] == 1:
        valid.remove(Action.DOWN_LEFT)

    return valid


def a_star(grid, h, start, goal, direction_change_cost):
    if direction_change_cost is None:
        return a_star_regular(grid, h, start, goal)
    else:
        return a_star_penalized(grid, h, start, goal, direction_change_cost)


def a_star_regular(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def a_star_penalized(grid, h, start, goal, direction_change_cost):

    # We're going to add a cost to CHANGING the direction
    # of motion - this will help in two situations:
    #
    # 1.) zig-zagging of same cost is discouraged,
    # 2.) using motion of the same type helps with pruning.
    #
    # For this, we also store the action delta that led
    # to the according node within every node.
        
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, (start, False, None)))
    visited = set([(start, False)])

    branch = {}
    found = None
        
    while not queue.empty():
        item = queue.get()
        current_node, previous_had_dc, previous_da = item[1]

        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[(current_node, previous_had_dc)][0]
            
        if current_node == goal:
            print('Found a path.')
            found = (current_node, previous_had_dc)
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                
                # Penalize direction changes
                had_dc = False
                if da != previous_da:
                    branch_cost += direction_change_cost
                    had_dc = True
                
                queue_cost = branch_cost + h(next_node, goal)
                next_item = (queue_cost, (next_node, had_dc, da))
                
                # Since all direction changes have the same penalty, we're not going
                # to store state from all directions, but rather only explore the
                # same node under a) no direction change and b) some direction change.
                if (next_node, had_dc) not in visited:
                    visited.add((next_node, had_dc))
                    branch[(next_node, had_dc)] = (branch_cost, (current_node, previous_had_dc), action)
                    queue.put(next_item)
             
    if found is not None:
        # retrace steps
        print('Backtracking from', found)
        n = found
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1][0] != start:
            path.append(branch[n][1][0])
            n = branch[n][1]
        path.append(branch[n][1][0])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

