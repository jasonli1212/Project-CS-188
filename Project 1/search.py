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

    # Setup
    visited = []
    Stack = util.Stack()
    goal = None
    start = (problem.getStartState(), None, 0)
    Stack.push((start, []))

    while not Stack.isEmpty():
        cur, path = Stack.pop()
        if cur[0] in visited:
            continue

        if problem.isGoalState(cur[0]):
            return path
        visited.append(cur[0])
        for state in problem.getSuccessors(cur[0]):
            if not state[0] in visited:
                newPath = path + [state[1]]
                Stack.push((state, newPath))
    
    return []     

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    
    "Same as DFS, use queuei instead of stack"
    if problem.isGoalState(problem.getStartState()):
        return []

    visited = []
    queue = util.Queue()
    goal = None
    start = (problem.getStartState(), None, 0)
    queue.push((start, []))

    while not queue.isEmpty():
        cur, path = queue.pop()
        if cur[0] in visited:
            continue

        if problem.isGoalState(cur[0]):
            return path
        visited.append(cur[0])
        for state in problem.getSuccessors(cur[0]):
            if not state[0] in visited:
                newPath = path + [state[1]]
                queue.push((state, newPath))
    
    return []
    

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    "Use a Priority Queue"
    visited = []
    pq = util.PriorityQueue()
    start = (problem.getStartState(), None, 0)

    """
    pq(a, b)
    a: tuple contains a location and the cost of this path so far and the path.
    b: the cost of this path so far
    """
    pq.update((start, 0, []), 0)

    while not pq.isEmpty():
        cur, totalCost, path = pq.pop()
        if cur[0] in visited:
            continue

        if problem.isGoalState(cur[0]):
            return path

        visited.append(cur[0])

        for state in problem.getSuccessors(cur[0]):
            cost = state[2] + totalCost
            newPath = path + [state[1]]
            pq.update((state, cost, newPath), cost)
    
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    "Use a Priority Queue"
    visited = []
    pq = util.PriorityQueue()
    start = (problem.getStartState(), None, 0)

    """
    pq(a, b)
    a: tuple contains a location and the cost of this path so far and the path.
    b: the cost of this path so far
    """
    pq.update((start, 0, []), 0)

    while not pq.isEmpty():
        cur, totalCost, path = pq.pop()
        if cur[0] in visited:
            continue

        if problem.isGoalState(cur[0]):
            return path

        visited.append(cur[0])

        for state in problem.getSuccessors(cur[0]):
            cost = state[2] + totalCost
            # if the state has not beening visited 
            # and the cost is lower than the cost in checked
            newPath = path + [state[1]]
            pq.update((state, cost, newPath), cost + heuristic(state[0], problem))
    
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
