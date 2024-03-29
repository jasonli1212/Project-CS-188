# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)

        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]
        """
        print("successorGameState")
        print(type(successorGameState))
        print("newPos")
        print(type(newPos))
        print("newFood")
        print(type(newFood))
        print("newGhostStates")
        print(type(newGhostStates))
        print("newScaredTimes")
        print(type(newScaredTimes))
        """

        "*** YOUR CODE HERE ***"

        """
        1. The farther away the ghost the better
        2. Distance to closer to food better.
        3. If scaredTime > 0 we can turn the ghost time positive.
        """

        def scoreFood(newPos, newFood):
            minDistFood = float('inf')
            foods = newFood.asList()
            totFoods = 0
            for food in foods:
                dist = manhattanDistance(newPos, food)
                totFoods += dist
                if dist < minDistFood:
                    minDistFood = dist
            size = len(foods) if len(foods) == 0 else 1
            return  minDistFood

        def scoreGhost(newPos, newGhostStates):
            minDistGhost = float('inf')
            for ghost in newGhostStates:
                if ghost.scaredTimer == 0:
                    ghostPos = ghost.getPosition()
                    dist = manhattanDistance(newPos, ghostPos)
                    if dist < minDistGhost:
                        minDistGhost = dist
            if minDistGhost == float('inf'):
                return 0
            return minDistGhost
            
        # Find the closest ghost

        value = scoreGhost(newPos, newGhostStates) + 100 / scoreFood(newPos, newFood) + 200*successorGameState.getScore()
        return value

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"

        """
        def max_value(state):
            initialize v= -inf
            for each successor of state:
                v = max(v, min-value(successor))
            return v
        """

        """
        def min_value(state):
            initialize v = +inf
            for each successor of state:
                v = min(v, max_value(successor))
            return v
        """

        def max_value(state, agentIndex, depth):
            # If we reach the end of the the tree
            if state.isWin() or state.isLose() or depth == 0:
                return (self.evaluationFunction(state), 0)

            v = float('-inf')
            bestAction = None
            legalMoves = state.getLegalActions(0)
            for action in legalMoves:
                # this next agent is a ghost.
                nextGameState = state.generateSuccessor(0, action)
                score = min_value(nextGameState, 1, depth)
                if score > v:
                    v = score
                    bestAction = action
            return (v, bestAction)

        def min_value(state, agentIndex, depth):
            # If we reach the end of the the tree
            if state.isWin() or state.isLose() or depth == 0:
                return self.evaluationFunction(state)

            v = float('inf')
            legalMoves = state.getLegalActions(agentIndex)
            for action in legalMoves:
                nextAgent = agentIndex + 1
                # if this next agent is a ghost.
                nextGameState = state.generateSuccessor(agentIndex, action)
                if nextAgent != state.getNumAgents(): 
                    v = min(v, min_value(nextGameState, nextAgent, depth))
                # if the next agent is pacman
                # We reach the end of this state so we go to the next depth
                else:
                    v = min(v, max_value(nextGameState, 0, depth - 1)[0])
            return v
        
        return max_value(gameState, 0, self.depth)[1]
        

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"

        """
        def max_value(state, alpha, beta):
            initialize v= '-inf'
            for each successor of state:
                v = max(v, vlaue(successor, alpha, beta))
                if v > beta:
                    return v
                alpha = max(alpha, v)
            return v
        
        def min_value(state, alpha, beta):
            initialize v = inf
            for each successor of state:
                v = min(v, value(successor, alpha, beta))
                if v < alpha return v
                beta = min(beta, v)
            return v
        """

        def max_value(state, agentIndex, depth, alpha, beta):
            # If we reach the end of the the tree
            if state.isWin() or state.isLose() or depth == 0:
                return (self.evaluationFunction(state), 0)

            v = float('-inf')
            bestAction = None
            legalMoves = state.getLegalActions(0)
            for action in legalMoves:
                # this next agent is a ghost.
                nextGameState = state.generateSuccessor(0, action)
                score = min_value(nextGameState, 1, depth, alpha, beta)
                if score > v:
                    v = score
                    bestAction = action
                if score > beta:
                    return (v, bestAction)
                alpha = max(alpha, v)
            return (v, bestAction)

        def min_value(state, agentIndex, depth, alpha, beta):
            # If we reach the end of the the tree
            if state.isWin() or state.isLose() or depth == 0:
                return self.evaluationFunction(state)

            v = float('inf')
            legalMoves = state.getLegalActions(agentIndex)
            nextAgent = agentIndex + 1
            for action in legalMoves:
                # if this next agent is a ghost.
                nextGameState = state.generateSuccessor(agentIndex, action)
                if nextAgent != state.getNumAgents(): 
                    v = min(v, min_value(nextGameState, nextAgent, depth, alpha, beta))
                # if the next agent is pacman
                # We reach the end of this state so we go to the next depth
                else:
                    v = min(v, max_value(nextGameState, 0, depth - 1, alpha, beta)[0])
                if v < alpha:
                    return v
                beta = min(beta, v)
            return v

        return max_value(gameState, 0, self.depth, float('-inf'), float('inf'))[1]

        util.raiseNotDefined()

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"

        # Take the max value out of all the expecti nodes
        def maxAgent(state, agentIndex, depth):
            if state.isWin() or state.isLose() or depth == 0:
                return self.evaluationFunction(state)

            score = float('-inf')
            for action in state.getLegalActions(0):
                score = max(score, expectimax(state.generateSuccessor(0, action), 1, depth))
            return score

        # Sum the value of all outcomes and take the avg, seen the ghost are uniformly at random.
        def expectimax(state, agentIndex, depth):
            # If we reach the end of the the tree
            if state.isWin() or state.isLose() or depth == 0:
                return self.evaluationFunction(state)
            

            v = 0
            legalMoves = state.getLegalActions(agentIndex)
            nextAgent = agentIndex + 1

            # if the next agent is pacman then we need to go next depth.
            if nextAgent == state.getNumAgents():
                nextAgent = 0
                depth -= 1
            for action in legalMoves:
                nextGameState = state.generateSuccessor(agentIndex, action)

                #Call maxer if it is pacman
                if nextAgent == 0:
                    v += maxAgent(nextGameState, nextAgent, depth)
                else:
                    v += expectimax(nextGameState, nextAgent, depth)

            return v / len(legalMoves)

        result = float("-inf")
        bestAction = None
        for action in gameState.getLegalActions(0):
            # this next agent is a ghost.
            nextGameState = gameState.generateSuccessor(0, action)
            score =  expectimax(nextGameState, 1, self.depth)
            if score > result:
                result = score
                bestAction = action
        return bestAction

        util.raiseNotDefined()

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: same as Q1
    """
    "*** YOUR CODE HERE ***"

    #successorGameState = currentGameState.generatePacmanSuccessor(action)
    newPos = currentGameState.getPacmanPosition()
    newFood = currentGameState.getFood()
    newGhostStates = currentGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    # distance to the closest foods.
    def scoreFood(newPos, newFood):

        foods = newFood.asList()
        minFood = float("inf")
        for food in foods:
            minFood = min(minFood, manhattanDistance(newPos, food))
        return  minFood

    #Total distance from Ghost.
    def scoreGhost(newPos, newGhostStates, newScaredTimes):
        minGhost = float("inf")
        for ghost in newGhostStates:
            if ghost.scaredTimer == 0:
                ghostPos = ghost.getPosition()
                minGhost = min(minGhost, manhattanDistance(newPos, ghostPos))
        if (minGhost == float("inf") or minGhost == 0):
            return 1
        return minGhost

    #We need to kill the ghost to get more points, inorder to get to 1000.
    def distToCapsules(newPos, capsules):
        return 10000 * len(capsules)

    value = 1/scoreFood(newPos, newFood)\
     - 1/scoreGhost(newPos,newGhostStates,newScaredTimes)\
      + currentGameState.getScore()\
       - distToCapsules(newPos, currentGameState.getCapsules())\
       + random.uniform(0, 0.2) # if  all diractions are the same.
    return value


# Abbreviation
better = betterEvaluationFunction
