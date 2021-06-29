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
    # return goal in the format of diections. 

    #Check start goal
    if problem.isGoalState(problem.getStartState()):
        return []

    # setup
    visited = set()
    stack = util.Stack()
    path = {}
    start = (problem.getStartState(), None, 0)
    stack.push(start)
    goal = None

    # Loop stack
    while not stack.isEmpty():

        cur = stack.pop()
        if cur[0] in visited:
            continue

        if problem.isGoalState(cur[0]):
            goal = cur
            break

        visited.add(cur[0])

        # save all the state to path and push them into the stack
        for state in problem.getSuccessors(cur[0]):
            if not state[0] in visited:
                path[state] = cur
                stack.push(state)
    # end stack loop


    result = []
    # load Path in reverse order
    while goal != start:
        result.append(goal[1])
        goal = path[goal]

    result.reverse()
    return result      

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    
    "Same as DFS, use queuei instead of stack"
    if problem.isGoalState(problem.getStartState()):
        return []

    visited = set()
    queue = util.Queue()
    path = {}
    start = (problem.getStartState(), None, 0)
    queue.push(start)
    goal = None
    while not queue.isEmpty():
        cur = queue.pop()
        if cur[0] in visited:
            continue

        if problem.isGoalState(cur[0]):
            goal = cur
            break

        visited.add(cur[0])
        for state in problem.getSuccessors(cur[0]):
            if not state[0] in visited:
                path[state] = cur
                queue.push(state)

    result = []
    while goal != start:
        result.append(goal[1])
        goal = path[goal]
    result.reverse()
    return result
    

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    "Use a Priority Queue"
    visited = set()
    pq = util.PriorityQueue()
    path = {}
    start = (problem.getStartState(), None, 0)

    """
    pq(a, b)
    a: tuple contains a location and the cost of this path so far.
    b: the cost of this path so far
    """
    pq.update((start, 0), 0)
    goal = None
    """
    checked: a dict that keep tracks the lowest cost to get to every state.
    """
    checked = {start[0]: 0}

    while not pq.isEmpty():
        cur, totalCost = pq.pop()
        if cur[0] in visited:
            continue

        if problem.isGoalState(cur[0]):
            goal = cur
            break

        visited.add(cur[0])

        for state in problem.getSuccessors(cur[0]):
            cost = state[2] + totalCost
            # if the state has not beening visited 
            # and the cost is lower than the cost in checked
            if not (state[0] in visited or (state[0] in checked and cost > checked[state[0]])):
                path[state] = cur
                checked[state[0]] = cost
                pq.update((state, cost), cost)

    result = []
    while goal != start:
        result.append(goal[1])
        goal = path[goal]
    result.reverse()
    return result

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited = set()
    pq = util.PriorityQueue()
    path = {}
    start = (problem.getStartState(), None, 0)
    pq.update((start, 0), 0)
    goal = None
    checked = {start[0]: 0}
    while not pq.isEmpty():
        cur, totalCost = pq.pop()
        if cur in visited:
            continue

        if problem.isGoalState(cur[0]):
            goal = cur
            break

        visited.add(cur[0])

        for state in problem.getSuccessors(cur[0]):
            cost = state[2] + totalCost
            if not (state[0] in visited or (state[0] in checked and cost > checked[state[0]])):
                path[state] = cur
                checked[state[0]] = cost
                pq.update((state, cost), cost + heuristic(state[0], problem))

    result = []
    # load Path
    while goal != start:
        result.append(goal[1])
        goal = path[goal]

    # print(result)
    result.reverse()
    return result


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
