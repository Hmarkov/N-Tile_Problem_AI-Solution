import copy
import timeit

#This file contains an implementation of Iterative deepening depth-first search for the solution of n-tile problem.

#Global counter for the number of yields
count=0
def count_yields (value):
    global count
    count = count+1
    if value==0:
        count=0

       
#Transform state to dictionary  
def make_dict (state):
    coordinate_dict = {}
    for x, row in enumerate(state):
        for y, value in enumerate(row):
            coordinate_dict[value] = (x, y)
    return coordinate_dict

#Function that returns a state based on the current state 
def move ( state ):
    copy_state = copy.deepcopy(state)
    [i ,j, grid ] = copy_state
    n = len ( grid )
    def move_blank (i ,j , n ):
        #down
        if i +1 < n :
            count_yields(1)
            yield ( i +1 , j )
        #up
        if i -1 >= 0:
            count_yields(1)
            yield (i -1 , j )
        #right
        if j +1 < n :
            count_yields(1)
            yield (i , j +1)
        #left
        if j -1 >= 0:
            count_yields(1)
            yield (i ,j -1)
    for pos in move_blank (i ,j , n ):
        i1 , j1 = pos
        grid [ i ][ j ] , grid [ i1 ][ j1 ] = grid [ i1 ][ j1 ] , grid [ i ][ j ]
        yield [ i1 , j1 , grid ]
        grid [ i ][ j ] , grid [ i1 ][ j1 ] = grid [ i1 ][ j1 ] , grid [ i ][ j ]
    


#DEPTH FIRST SEARCH
def  dfs_rec(path,goal, depth):
    if path [-1]==goal:
        return  path
    if depth <= 0:
        return None
    else:
        for nextState in move(path[-1]):
            if  nextState  not in path:
                nextPath = path+[nextState]
                solution = dfs_rec(nextPath, goal,depth -1)
                if  solution  != None:
                    return  solution
    return None

#Iterative Deepening based on DEPTH FIRST SEARCH
def IDDFS(path, goal):
    for depth in range(0, 100):
        result = dfs_rec([path], goal,depth)
        # print(f"depth {depth}")
        if result is None:
            continue
        return "",len(result)-1,count
    return 'goal not in graph with depth {}'.format(100),0


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

#msg            return a messagge based on iddfs
#moves          return the number of depth 
#c              return the number of moves
#region ITERATE TROUGH STATES
print("First five instances with goal = [0, 2, [[3, 2, 0], [6, 1, 8], [4, 7, 5]]]")
for i in states1:
    start = timeit.default_timer()
    msg,moves,c = IDDFS((i), (goal1))
    stop = timeit.default_timer()
    if msg=="":
        print("Depth= {}".format(moves)+"    Moves={}".format(c)+'    Time= {0:.2f}'.format(stop - start))
        count_yields(0)
    else:
        print("Solution not found!")
print("Next five instances with goal=[2,2,[[1,2,3],[4,5,6],[7,8,0]]]")
for i in states2:
    start = timeit.default_timer()
    msg,moves,c= IDDFS((i), (goal2))
    stop = timeit.default_timer()

    if msg=="":
        print("Depth= {}".format(moves)+"    Moves={}".format(c)+'    Time= {0:.2f}'.format(stop - start))
        count_yields(0)
    else:
        print("Solution not found!")
#endregion