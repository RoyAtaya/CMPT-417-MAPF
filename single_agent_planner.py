import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0,0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table = []
    maxTimeStep = 0
    for currConstraint in constraints:
        if agent == currConstraint['agent']:
            maxTimeStep = max(maxTimeStep, currConstraint['time_step'])
    table = [[] for x in range(maxTimeStep + 1)]
    for currConstraint in constraints:
        if agent == currConstraint['agent']:
            table[currConstraint['time_step']].append(currConstraint)
    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time >= len(constraint_table):
        return False
    for constraint in constraint_table[next_time]:
        if len(constraint['loc']) == 1:
            if next_loc == constraint['loc'][0]:
                return True
        else:
            if curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]:
                return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    constraintTable = build_constraint_table(constraints, agent) # Task 1.2
    # print("current constraint for agent", agent, " : ", constraintTable)

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0} # Task 1.1.1 added new key/value pair for time step
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root # Task 1.1.2 use tuples of (cell, time step) contains the expanded nodes
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:
            # return get_path(curr)
            isFinished = True
            if len(constraintTable) >= curr['time_step']:
                for nextTime in range(curr['time_step'] + 1, len(constraintTable)):
                    if is_constrained(goal_loc, goal_loc, nextTime, constraintTable):
                        isFinished = False
            if isFinished:
                return get_path(curr)

        for dir in range(5): # Task 1.1.1
            child_loc = move(curr['loc'], dir)
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr, 
                    'time_step': curr['time_step'] + 1} # Task 1.1.1 added new key/value pair for time step. child time step is 1 more than parent node
            if is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, constraintTable): # Task 1.2
                continue
            if (child['loc'], child['time_step']) in closed_list: # Task 1.1.2 use tuples of (cell, time step)
                existing_node = closed_list[(child['loc'], child['time_step'])] # Task 1.1.2 use tuples of (cell, time step)
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child # Task 1.1.2 use tuples of (cell, time step)
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['time_step'])] = child # Task 1.1.2 use tuples of (cell, time step)
                push_node(open_list, child)

    return None  # Failed to find solutions
