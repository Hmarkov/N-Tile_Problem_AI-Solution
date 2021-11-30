import timeit
import numpy as np
import copy

#This file contains an implementation of Iterative Deepening A*(IDA*) for the solution of n-tile problem.
#The implementation of IDA* is based on the pseudo code provided here https://en.wikipedia.org/wiki/Iterative_deepening_A*
#To tweak it in my favour some modifications were made to match and process our states syntax

#Global counter for the number of yields
count=0
def count_yields(value):
    global count
    count = count+value
    if value==0:
        count=0

#region
#'threshold'        calculates the total cost of moves from current node state to end state 
#'path'             gets the current state to search
#'node'             get the current node
#'g'                the cost to reach the current node
#'f'                estimated cost of the cheapest path considering the root,current and end nodes
#'h(node)'          estimate cost of the cheapest path from the current node to goal state
#'nextnodes()'      expands the current node"state" and check for all possible move 
#endregion

#IterativeAstar on each iteration perform depth first search
#Cut off a branch when the total cost exceed the threshold 
#Increase threshold on new node
#The threshold for the new interation is the minimum cost of all states that exceeded the current threshold
def IterativeAstar(initial_state, goal_state):
    initial_node = Node(initial_state)
    threshold = distance(initial_state, goal_state)
    while 1:
        path =set([initial_node])
        tmp = search(initial_node, goal_state, 0, threshold, path)
      
        if tmp == True:
           return True, threshold,count
        elif tmp == float('inf'):
            return False, float('inf'),count
        else:
            threshold = tmp

def search(node, goal_state, g, threshold, path):
    f = g + manhattan_h(np.array(node.state),np.array( goal_state[2]))
    if f > threshold:
        return f
    if np.array_equal(node.state, goal_state[2]):
        return True
    minimum = float('inf')  
    for nodes in node.nextnodes():
        for n in nodes:
            if n not in path:
                path.add(n)
                tmp = search(n, goal_state, g + 1, threshold, path)
                if tmp == True:
                    return True
                if tmp < minimum:
                    minimum = tmp

    return minimum


#region Manhattan Heuristics 
#There are two function to calculate the distance from a node to node that return the same output
#The difference between them is the reading property
#The first part reads it as list of arrays which are transformed in a class object to make it easier from the starting syntax
#The second part reads it with the starting sintax for the only reason that the values passed are not transformed in a Class object

#part1
def manhattan_h(state1, state2):
    def distance(a, b):
        return abs(b[0] - a[0]) + abs(b[1] - a[1])
    def count_distance(number,state1, state2):
        pos1 = np.where(np.array(state1) == number)
        pos2 = np.where(np.array(state2) == number)
        return distance(pos1, pos2)
    size = range(1, len(state1) ** 2)
    distances = [count_distance(num, state1, state2) for num in size]
    return sum(distances)

#part2
def distance(state,goal):
    def make_dict(state):
        coordinate_dict = {}
        for x, row in enumerate(state):
            for y, value in enumerate(row):
                coordinate_dict[value] = (x, y)
        return coordinate_dict
    dict_start = make_dict(state[2])
    dict_goal = make_dict(goal[2])
    distance = 0
    for i in range(1, len(dict_goal)):
        distance += abs(dict_goal[i][0] - dict_start[i][0]) + abs(dict_goal[i][1] - dict_start[i][1])
    return distance
#endregion


#region Class object
#A Class object is need for a better representation of our states
class Node():
    ### Transform object in 3 parts from the start state [i,j,[state]]
    def __init__(self, state):
        self.i=state[0]
        self.j=state[1]
        self.state = state[2]

    ### return a one dimension version of the state
    def __repr__(self):
        n=np.array(self.state)
        return np.array_str(n.flatten())

    def __hash__(self):
        return hash(self.__repr__())
        
    def __eq__(self, other):
        return self.__hash__() == other.__hash__()

    ### part1 returns a list containg [i,j] that are possible movements of the ZERO in our state
    ### part2 returns a list containg possible states based on the #part1 list
    def nextnodes(self):
        zero=self.i,self.j
        
        #part1
        def move_blank (self):
            i,j = zero
            value=0
            n=len(self.state)
            movements=[]
            #down
            if i +1 < n :
                movements.append([ i +1 , j])
                value=value+1
            #up
            if i -1 >= 0:   
                movements.append ([i -1 , j])
                value=value+1
            #right
            if j +1 < n :
                movements.append ([i , j +1])
                value=value+1
            #left
            if j -1 >= 0:
                movements.append ([i ,j -1])
                value=value+1
            count_yields(value)
            yield movements
        
        #part2
        arr = []
        for movement in move_blank(self):
            for pos in movement:
                i1 , j1 = pos[0],pos[1]
                [i ,j, grid ]=copy.deepcopy([self.i,self.j,self.state])
                grid [ i ][ j ] , grid [ i1 ][ j1 ] = grid [ i1 ][ j1 ] , grid [ i ][ j ]
                arr.append(Node([i1,j1,grid]))
        yield arr
#endregion

#region STATES TO EXAMINE
states1 = [[0, 0, [[0, 7, 1], [4, 3, 2], [8, 6, 5]]],
                    [0, 2, [[5, 6, 0], [1, 3, 8], [4, 7, 2]]],
                    [2, 0, [[3, 5, 6], [1, 2, 7], [0, 8, 4]]],
                    [1, 1, [[7, 3, 5], [4, 0, 2], [8, 1, 6]]],
                    [2, 0, [[6, 4, 8], [7, 1, 3], [0, 2, 5]]],
                    ]

goal1 = [0, 2, [[3, 2, 0], [6, 1, 8], [4, 7, 5]]]


states2 = [[0, 0, [[0, 1, 8], [3, 6, 7], [5, 4, 2]]],
                    [2, 0, [[6, 4, 1], [7, 3, 2], [0, 5, 8]]],
                    [0, 0, [[0, 7, 1], [5, 4, 8], [6, 2, 3]]],
                    [0, 2, [[5, 4, 0], [2, 3, 1], [8, 7, 6]]],
                    [2, 1, [[8, 6, 7], [2, 5, 4], [3, 0, 1]]]
                    ]
goal2 = [2, 2, [[1, 2, 3], [4, 5, 6], [7, 8, 0]]]
#endregion

#is_found    return a bool based on IterariveAstar to control output
#th          return the number of threshold=depth 
#c           return the number of yield moves

#region ITERATE TROUGH STATES
print("First five instances with goal = [0, 2, [[3, 2, 0], [6, 1, 8], [4, 7, 5]]]")
for i in states1:
    start = timeit.default_timer()
    is_found, th,c = IterativeAstar((i), (goal1))
    stop = timeit.default_timer()
    if is_found is True:
        print("Depth= {}".format(th)+"    Moves={}".format(c)+'    Time= {0:.2f}'.format(stop - start))
        count_yields(0)
    else:
        print("Solution not found!")
print("Next five instances with goal = [2, 2, [[1, 2, 3], [4, 5, 6], [7, 8, 0]]]")
for i in states2:
    start = timeit.default_timer()
    is_found, th,c  = IterativeAstar((i), (goal2))
    stop = timeit.default_timer()
    if is_found is True:
        print("Depth= {}".format(th)+"    Moves={}".format(c)+'    Time= {0:.2f}'.format(stop - start))
        count_yields(0)
    else:
        print("Solution not found!")
#endregion