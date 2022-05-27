import sys
import pandas as pd
import time
from collections import deque

#BEGINNING, CHECK FOR NUMBER OF ARGUMENTS = 3
#AND CATCH INDEXERROR

try:

    arg1 = sys.argv[1]
    arg2 = sys.argv[2]

    if len(sys.argv) == 3:
        print('Wing, Joseph, A20416312 solution:')
        print('INITIAL STATE: ', arg1)
        print('GOAL STATE: ', arg2, '\n')
    elif len(sys.argv) > 3:
        print("ERROR: Not enough or too many input arguments.")
        exit()
    elif len(sys.argv) <3:
        print("ERROR: Not enough or too many input arguments.")
        exit()

    gbfs_goal_state_reached = False;
    astar_goal_state_reached = False;

    if arg1  == arg2:
        gbfs_goal_state_reached = True;
        astar_goal_state_reached = True;

except (IndexError, NameError) as error:
    print("ERROR: Not enough or too many input arguments.\n\n")
    exit()

#READ DATA USING PANDAS

straightline_data = pd.read_csv('straightline.csv', index_col='STATE')
driving_data = pd.read_csv('driving.csv', index_col='STATE')

driving_dict = driving_data.to_dict()

init_state = arg1
goal_state = arg2

sol_path = []
path_cost = 0

curr_state = init_state

######GBFS
print('Greedy Best First Search:')

#BEGIN TIMER
start = time.time()

try:
    while gbfs_goal_state_reached == False:

        #SELECT ONLY STATES THAT ARE AVAILABLE
        filter_driv_data = driving_data[driving_data[curr_state]!=-1]

        #outputs available states and their DRIVING DISTANCES
        available_states = filter_driv_data[curr_state].to_dict()
        
        #REMOVE OWN STATE FROM AVAILABLE PATHS
        for key in list(available_states.keys()):
            if available_states[key] == 0:
                del available_states[key]
        #print('Available states and driving distances: ', available_states)

        #STRAIGHT LINE DISTANCES FROM AVAILABLE STATE TO GOAL STATE
        straightline_dist = {}
        for key in available_states.keys():
            straightline_dist[key] = straightline_data.at[key, goal_state]
        #print('Straight line distances from available states to goal state: ', straightline_dist)

        #GET MIN STRAIGHTLINE DIST FROM AVAILABLE STATES
        shortest_dist = min(list(straightline_dist.values()))

        #FIND STATE WITH SHORTEST STRAIGHT LINE DIST
        def get_key(val):
            for key, value in straightline_dist.items():
                if val == value:
                    return key

        #APPEND SELECTED STATE TO SOLUTION PATH
        sol_path.append(curr_state)

        #SET CURRENT STATE TO SELECTED STATE W/SHORTEST DIST
        curr_state = get_key(shortest_dist)
        path_cost = path_cost + available_states[curr_state]
        #print('Current state: ',curr_state)
        #print('Current path cost: ',path_cost,'\n')

        #IF GOAL STATE, EXIT WHILE LOOP
        if curr_state == goal_state:
            gbfs_goal_state_reached = True

    #APPEND GOAL STATE TO SOLUTION PATH AFTER EXITING WHILE LOOP
    sol_path.append(goal_state)
    print('Solution path: ',sol_path)

    #NUMSTATES ON PATH
    print('Number of states on a path: ', len(sol_path))

    #TOTAL PATH COST
    print('Path cost:',path_cost,'miles')

except KeyError:
    end = time.time()
    print('Solution path: [NOT FOUND]')
    print('Number of states on a path: 0')
    print('Path cost: 0 miles')

#END TIMER
end = time.time()

#CALCULATE TIME
exec_time_sec = round((end - start),6)
exec_time_microsec = round((exec_time_sec*1000000),6)
print('Execution time:',exec_time_sec, 'seconds (', exec_time_microsec, 'microseconds )\n')






######A* Algorithm
curr_state = init_state
sol_path = []
path_cost = 0

print('A* Search')

try:

    #SELECT ONLY STATES THAT ARE AVAILABLE
    filter_driv_data = driving_data[driving_data[curr_state]!=-1]

    #outputs available states of curr_state and their DRIVING DISTANCES
    available_states = filter_driv_data[curr_state].to_dict()

    #CREATE ADJAC_LIST TO_DICT()
    adjac_list = {}
    #print(driving_data)

    for key in filter_driv_data:
        select_driv_data = driving_data[driving_data[key]!=-1]
    #    print(key,':',list(select_driv_data[key]))
        filter_select_driv_data = select_driv_data[select_driv_data[key]!=0]
        data_to_dict = filter_select_driv_data[key].to_dict()
        adjac_list[key] = data_to_dict

except KeyError:
    print('Solution path: [NOT FOUND]')
    print('Number of states on a path: 0')
    print('Path cost: 0 miles')
    print('Execution time: 0 seconds')
    exit()


#CONVERT ADJAC_LIST
for key in adjac_list:
    content = []
    for x in range(len(adjac_list[key])):        
        tup = tuple([list(adjac_list[key].keys())[x], adjac_list[key][list(adjac_list[key].keys())[x]]]) 
        #print(tup)
        content.append(tup)
    adjac_list[key] = content
    content = []

#print(adjac_list)


class Graph:
    def __init__(self, adjac_list):
        self.adjac_list = adjac_list

    def get_neighbors(self, v):
        return self.adjac_list[v]

    #heuristics
    def h(self, n, goal):
        return straightline_data.at[n, goal]

    def calculate_path_cost(sol_path):
        path_cost=0
        for x in range(len(sol_path)-1):
            path_cost = path_cost + driving_data[sol_path[x]][sol_path[x+1]]
        return path_cost

    def a_star_algorithm(self, start, stop):

        #A* requires an open list and a closed list
        open_lst = set([start])
        closed_lst = set([])

        #present distances from all other nodes
        pres = {}
        pres[start] = 0
        
        #contains adjac mapping of all nodes
        par = {}
        par[start] = start


        while len(open_lst) > 0:
            n = None

            #find node with lowest val of f
            for v in open_lst:
                if n == None or pres[v] + self.h(v,stop) < pres[n] + self.h(n,stop):
                    n = v;

            if n == None:
                print('Path not found')
                return None

            #if current node is the goal
            if n == stop:
                sol_path = []

                while par[n] != n:
                    sol_path.append(n)
                    n = par[n]

                sol_path.append(start)
                sol_path.reverse()

                print('Solution path: {}'.format(sol_path))
                
                print('Number of states on a path:',len(sol_path))

                #calculate path cost
                path_cost = 0
                for x in range(len(sol_path)-1):
                    path_cost = path_cost + driving_data[sol_path[x]][sol_path[x+1]]

                print('Path cost:',path_cost,'miles')

                return sol_path

            #for all available nodes in current node...
            for (m, dist) in self.get_neighbors(n):
                #if current node is not present in both open_lst and closed_lst, add to open_lst
                if ((m not in open_lst) and (m not in closed_lst)):
                    open_lst.add(m)
                    par[m] = n
                    pres[m] = pres[n] + dist
                    
                #else, check if faster to visit n than m
                else:
                    if pres[m] > pres[n] + dist:
                        pres[m] = pres[n] + dist
                        par[m] = n

                        if m in closed_lst:
                            closed_lst.remove(m)
                            open_lst.add(m)

            open_lst.remove(n)
            closed_lst.add(n)
        print('Path does not exist')
        return None


start = time.time()

try:
    Graph(adjac_list).a_star_algorithm(arg1, arg2)
#    print(adjac_list)

except KeyError:
    print('Solution path: [NOT FOUND]')
    print('Number of states on a path: 0')
    print('Path cost: 0 miles')
    print('Execution time: 0.0 seconds')
    exit()

end = time.time()
exec_time_sec = round((end - start),6)
exec_time_microsec = round((exec_time_sec*1000000),6)

print('Execution time:',exec_time_sec,'seconds','(',exec_time_microsec,'microseconds )')





