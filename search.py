# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    You DO NOT need to implement any heuristic, but you DO have to call it.
    The heuristic function is "manhattanHeuristic" from searchAgent.py.
    It will be pass to this function as second argument (heuristic).
    """
    "*** YOUR CODE HERE FOR TASK 2 ***"

    open_list = util.PriorityQueue()
    closed = []
    # use the format of (current state, past costs' sum, [past records])
    init_state = (problem.getStartState(), 0, [])
    open_list.update(init_state, 0)
    best = {problem.getStartState(): (problem.getStartState(), 0, [])}
    action_list = []
    while not open_list.isEmpty():
        curr_state = open_list.pop()
        if problem.isGoalState(curr_state[0]):
            action_list = curr_state[2]
            break
        if curr_state[0] not in closed or curr_state[1] < best[curr_state[0]][1]:
            closed.append(curr_state[0])
            best[curr_state[0]] = (curr_state[0], curr_state[1], curr_state[2])
            curr_state = best[curr_state[0]]
            for successor in problem.getSuccessors(curr_state[0]):
                current_path = curr_state[2].copy()
                new_cost = curr_state[1] + successor[2]
                priority = new_cost + heuristic(successor[0], problem)
                open_list.update((successor[0], curr_state[1] + successor[2], current_path + [successor[1]]), priority)
    return action_list
        


# Extensions Assignment 1
def enforcedHillClimbing(problem, heuristic=nullHeuristic):
    """
    Local search with heuristic function.
    You DO NOT need to implement any heuristic, but you DO have to call it.
    The heuristic function is "manhattanHeuristic" from searchAgent.py.
    It will be pass to this function as second argument (heuristic).
    """
    "***YOUR CODE HERE FOR TASK 1***"

    # use the format of ((current state, current direction, cost), [past records])
    # set the initial state as start
    init_state = ((problem.getStartState(), '', 0), [])
    actions = []
    # loop until find the goal state
    while not problem.isGoalState(init_state[0][0]):
        # always keep the better state
        init_state = improve(init_state, problem, heuristic)
    return init_state[1][1:] + [init_state[0][1]]

def improve(init_state, problem, heuristic):
    # use queue to store all possible states
    queue = util.Queue()
    queue.push(init_state)
    # list of all visited states
    closed = []
    while queue:
        curr_node = queue.pop()
        if curr_node[0][0] not in closed:
            closed.append(curr_node[0][0])
            # loop until find a better state
            if heuristic(curr_node[0][0], problem) < heuristic(init_state[0][0], problem):
                return curr_node
            # add all successors to the queue
            for successor in problem.getSuccessors(curr_node[0][0]):
                current_path = curr_node[1].copy()
                queue.push((successor, current_path + [curr_node[0][1]]))
    

    
def jumpPointSearch(problem, heuristic=nullHeuristic):
    """
    Search by pruning non-critical neighbors with jump points.
    You DO NOT need to implement any heuristic, but you DO have to call it.
    The heuristic function is "manhattanHeuristic" from searchAgent.py.
    It will be pass to this function as second argument (heuristic).
    """
    "*** YOUR CODE HERE FOR TASK 3 ***"

    
    open_list = util.PriorityQueue()
    closed = []
    # use the format of (current state, [past records], past costs' sum)
    init_state = (problem.getStartState(), [], 0)
    curr_state = None
    open_list.update(init_state, 0)
    best = {problem.getStartState(): (problem.getStartState(), [], 0)}
    action_list = []
    while not open_list.isEmpty():
        curr_state = open_list.pop()
        if problem.isGoalState(curr_state[0]):
            action_list = curr_state[1]
            break
        if curr_state[0] not in closed or curr_state[2] < best[curr_state[0]][2]:
            closed.append(curr_state[0])
            best[curr_state[0]] = (curr_state[0], curr_state[1], curr_state[2])
            curr_state = best[curr_state[0]]
            for successor in identify_successors(curr_state, problem):
                if successor != None:
                    priority = successor[2] + heuristic(successor[0], problem)
                    open_list.update(successor, priority)
    return action_list


def identify_successors(curr, problem):
    successors = []
    neighbours = problem.getSuccessors(curr[0])
    for neighbour in neighbours:
        dx = neighbour[0][0] - curr[0][0]
        dy = neighbour[0][1] - curr[0][1]
        jump_point = jump(curr, dx, dy, neighbour[1], problem)
        if jump_point != None:
            successors.append(jump_point)
    return successors

def jump(curr, dx, dy, direction, problem):
    
    curr_node = curr[0]
    neighbourX = curr_node[0] + dx
    neighbourY = curr_node[1] + dy
    neighbour = (neighbourX, neighbourY)
    current_path = curr[1].copy()
    
    if not checkNodeAvail(neighbour, problem):
        return None
        
    if problem.isGoalState(neighbour):
        return (neighbour, current_path + [direction], curr[2] + 1)

    if dx != 0:
        if not checkNodeAvail((curr_node[0], curr_node[1] + 1), problem):
            if checkNodeAvail((neighbour[0], curr_node[1] + 1), problem):
                return (neighbour, current_path + [direction], curr[2] + 1)

        if not checkNodeAvail((curr_node[0], curr_node[1] - 1), problem):
            if checkNodeAvail((neighbour[0], curr_node[1] - 1), problem):
                return (neighbour, current_path + [direction], curr[2] + 1)

        if jump((neighbour, [], 0), 0, 1, direction, problem) != None or jump((neighbour, [], 0), 0, -1, direction, problem) != None:
            return (neighbour, current_path + [direction], curr[2] + 1)

    elif dy != 0:
        if not checkNodeAvail((curr_node[0] + 1, curr_node[1]), problem):
            if checkNodeAvail((curr_node[0] + 1, neighbour[1]), problem):
                return (neighbour, current_path + [direction], curr[2] + 1)

        if not checkNodeAvail((curr_node[0] - 1, curr_node[1]), problem):
            if checkNodeAvail((curr_node[0] - 1, neighbour[1]), problem):
                return (neighbour, current_path + [direction], curr[2] + 1)
                
    return jump((neighbour, current_path + [direction], curr[2] + 1), dx, dy, direction, problem)
    

def checkNodeAvail(node, problem):
    height = problem.walls.height
    width = problem.walls.width
    if node[0] <= 0:
        return False
    if node[0] > width:
        return False
    if node[1] <= 0:
        return False
    if node[1] > height:
        return False
    if problem.walls[node[0]][node[1]]:
        return False
    return True

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ehc = enforcedHillClimbing
jps = jumpPointSearch
