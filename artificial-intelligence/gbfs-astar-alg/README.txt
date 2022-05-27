This is a demonstration of two things:

1.) The use and understanding of basic artificial intelligence algorithms
2.) The use and understanding of the Python programming language along with Pandas dataframes

We are given a map (Check PDF), along with two csv files, driving.csv and straightline.csv, that show all 50 states. driving.csv has the driving distances between each state, and straightline.csv has straight-line distances between each state.
Using two well-known Artificial Intelligence algorithms, Greedy Best First Search and A*, we can find the shortest distance between two states.

To run this program on a shell, do the following shell command:

	python search_alg.py [INITIAL] [GOAL]

where [INITIAL] = initial state, and [GOAL] = goal state.


The output should be as follows:

Initial state: INITIAL
	Goal state: GOAL
	
	Greedy Best First Search:
	Solution path: STATE1, STATE2, STATE3, …, STATEN-1, STATEN
	Number of states on a path: X1
	Path cost: Y1
	Execution time: T1 seconds

	A* Search:
	Solution path: STATE1, STATE2, STATE3, …, STATEN-1, STATEN
	Number of states on a path: X2
	Path cost: Y2
	Execution time: T2 seconds


If no path is found, this should be the output:

Solution path: FAILURE: NO PATH FOUND
	Number of states on a path: 0
	Path cost: 0
	Execution time: T3 seconds
