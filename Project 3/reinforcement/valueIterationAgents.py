# valueIterationAgents.py
# -----------------------
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


# valueIterationAgents.py
# -----------------------
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


import mdp, util

from learningAgents import ValueEstimationAgent
import collections

class ValueIterationAgent(ValueEstimationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A ValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100):
        """
          Your value iteration agent should take an mdp on
          construction, run the indicated number of iterations
          and then act according to the resulting policy.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state, action, nextState)
              mdp.isTerminal(state)
        """
        self.policy = {}
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0
        self.runValueIteration()
    def runValueIteration(self):
        # Write value iteration code here

        """
        The idea is to update the policy and the value at the same time.
        """
        "*** YOUR CODE HERE ***"
        for _ in range(self.iterations):
            currentRun = util.Counter()
            for state in self.mdp.getStates():
                currentState = []
                for action in self.mdp.getPossibleActions(state):
                    currentState.append(self.getQValue(state,action))
                if currentState != []:
                    currentRun[state] = max(currentState)
            self.values = currentRun

    def getValue(self, state):
        """
          Return the value of the state (computed in __init__).
        """
        return self.values[state]


    def computeQValueFromValues(self, state, action):
        """
          Compute the Q-value of action in state from the
          value function stored in self.values.
        """
        "*** Q(S,A) = sum_s`(T(s,a,s`)[R(s,a,s`)+rV*(s`)]) ***"
        "*** YOUR CODE HERE ***"
        result = 0
        # list of (nextState, prob) pairs
        sp = self.mdp.getTransitionStatesAndProbs(state, action)
        for nextState in sp:
            tValue = nextState[1]
            rValue = self.mdp.getReward(state, action, nextState[0])
            result += tValue * (rValue + self.discount * self.values[nextState[0]])
        # print(result)
        return result
        util.raiseNotDefined()

    def computeActionFromValues(self, state):
        """
          The policy is the best action in the given state
          according to the values currently stored in self.values.

          You may break ties any way you see fit.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return None.
        """
        "*** YOUR CODE HERE ***"
        if self.mdp.isTerminal(state):
            return None
        policy = util.Counter()
        for action in self.mdp.getPossibleActions(state):
            policy[action] = self.getQValue(state, action)
        return policy.argMax()
        
        util.raiseNotDefined()

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)

class AsynchronousValueIterationAgent(ValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        An AsynchronousValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs cyclic value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 1000):
        """
          Your cyclic value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy. Each iteration
          updates the value of only one state, which cycles through
          the states list. If the chosen state is terminal, nothing
          happens in that iteration.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state)
              mdp.isTerminal(state)
        """
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def runValueIteration(self):
        "*** YOUR CODE HERE ***"
        states = self.mdp.getStates()
        for i in range(self.iterations):
            index = i % len(states)
            state = states[index]
            if self.mdp.isTerminal(state):
                continue
            currentState = []
            for action in self.mdp.getPossibleActions(state):
                currentState.append(self.getQValue(state,action))
            if currentState != []:
                self.values[state] = max(currentState)


class PrioritizedSweepingValueIterationAgent(AsynchronousValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A PrioritizedSweepingValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs prioritized sweeping value iteration
        for a given number of iterations using the supplied parameters.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100, theta = 1e-5):
        """
          Your prioritized sweeping value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy.
        """
        self.theta = theta
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def runValueIteration(self):
        "*** YOUR CODE HERE ***"

        "First we need to find the predecessors"
        
        predecessor = {}
        for state in self.mdp.getStates():
            if self.mdp.isTerminal(state):
                continue
            for action in self.mdp.getPossibleActions(state):
                for nextState in self.mdp.getTransitionStatesAndProbs(state, action):
                    if nextState[0] in predecessor:
                        predecessor[nextState[0]].append(state)
                    else:
                        predecessor[nextState[0]] = [state]
        
        #print(predecessor)

        pq = util.PriorityQueue()
        for state in self.mdp.getStates():
            if self.mdp.isTerminal(state):
                continue
            maxValues = []
            for action in self.mdp.getPossibleActions(state):
                maxValues.append(self.getQValue(state, action))
            diff = abs(self.values[state] - max(maxValues))
            pq.update(state, -diff)
        
        # print(pq.heap)

        for _ in range(self.iterations):
            if pq.isEmpty():
                break
            state = pq.pop()

            maxValues = []

            "Update the value of s (if it is not a terminal state) in self.values."
            if not self.mdp.isTerminal(state):
                for action in self.mdp.getPossibleActions(state):
                    maxValues.append(self.getQValue(state, action))
                self.values[state] = max(maxValues)
            
            for p in predecessor[state]:
                if self.mdp.isTerminal(p):
                    continue
                maxValues = []
                for action in self.mdp.getPossibleActions(p):
                    maxValues.append(self.getQValue(p, action))
                diff = abs(self.values[p] - max(maxValues))
                if diff > self.theta:
                    pq.update(p, -diff)