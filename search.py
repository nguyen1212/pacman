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
REVERSE_PUSH = False
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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    frontier = util.Stack()
    explored = set()
    return graphSearch(frontier, explored, problem, 'DFS')
    # util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()
    frontier = util.Queue()
    explored = set()
    return graphSearch(frontier, explored, problem, 'BFS')

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()
    frontier = util.PriorityQueue()
    explored = set()
    return graphSearch(frontier, explored, problem, 'UCS')

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # util.raiseNotDefined()
    frontier = util.PriorityQueue()
    explored = set()
    return graphSearch(frontier, explored, problem, heuristic)

def graphSearch(frontier, explored, problem, method): 
    initState = problem.getStartState()
    if problem.isGoalState(initState):
        return []
    initNode = Node(initState)
    if method == "UCS" or type(method) != str:
        frontier.push(initNode, 0)
    else:
        frontier.push(initNode)
    while True:
        if frontier.isEmpty():
            return []
        curr = frontier.pop()
        explored.add(curr)
        if problem.isGoalState(curr.state):
            return curr.getPath()
        for node in problem.getSuccessors(curr.state):
            node = Node(node[0], curr, node[1], node[2])
            nodeList = frontier.list if (type(method) == str and method != 'UCS') else [item[2] for item in frontier.heap] 
            if node not in explored and node not in  nodeList:
                if method == 'UCS':
                    frontier.push(node, node.getPathCost())
                elif type(method) != str:
                    frontier.push(node, node.getPathCost() + method(node.state, problem))
                else:
                    frontier.push(node)
            if (method == "UCS" or type(method) != str) and node in nodeList:
                if method == "UCS":
                    frontier.update(node, node.getPathCost())
                else:
                    frontier.update(node, node.getPathCost() + method(node.state, problem))


class Node:
    def __init__(self, state, parent = None, action = "", stepcost = 0):
        self.parent = parent
        if parent is not None:
            self.pathcost = parent.pathcost + stepcost
        else:
            self.pathcost = stepcost
        self.state = state
        self.action = action
        self.stepcost = stepcost

    def __hash__(self):
        return  self.state.__hash__()

    def __eq__(self, other):
        if self.state == other.state:
            return True
        return False

    def getParent(self):
        return self.parent

    def getPathCost(self):
        return self.pathcost

    def getStepCost(self):
        return self.stepcost

    def getPath(self):
        if self.parent is None:
            return []
        return self.parent.getPath() + [self.action]
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
