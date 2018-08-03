from __future__ import print_function
import os
import sys
sys.path.append('..')
sys.path.append('../..')
import argparse
import utils
from student_utils_sp18 import *
from ortools.linear_solver import pywraplp
"""
======================================================================
  Complete the following function.
======================================================================
"""

"""      
Floyd Warshall Algorithm that returns shortest path with an inner function to construct the actual path 
Gets all pair shortest path via Floyd Warshall Algrorithm
"""
def FloydWarshall(adjacency_matrix):
    # Ref: simple python implemetation of Floyd-Warshall alghorithm by author: J. Luis Medina
    
    # save distance in dist matrix and previous node in pred matrix
    dist = {}
    pred = {}
    
    # Initialize dist and pred matrix
    for u in range(len(adjacency_matrix)):
        dist[u] = {}
        pred[u] = {}
        for v in range(len(adjacency_matrix)):
            dist[u][v] = float('Inf')
            pred[u][v] = -1
        dist[u][u] = 0
        # if there is a valid path from u to v, then set dist[u][v] to the distance; otherwise, set the value to infinity
        for v in range(len(adjacency_matrix)):
            if(adjacency_matrix[u][v]!='x'):
                dist[u][v] = adjacency_matrix[u][v]
            else:
                dist[u][v]=float('Inf')
            pred[u][v] = u
    # check if there is a shorter path via any middle node
    for t in range(len(adjacency_matrix)):
        # given dist u to v, check if path u - t - v is shorter
        for u in range(len(adjacency_matrix)):
            for v in range(len(adjacency_matrix)):
                newdist = dist[u][t] + dist[t][v]
                if newdist < dist[u][v]:
                    # set dist to the shorter distance
                    dist[u][v] = newdist
                    # route new path through t
                    pred[u][v] = pred[t][v]
    # recursively construct the shortest path between any two nodes based on pred matrix
    def shortest_path(a,b):
        if(a==b):
            return [a]
        elif(pred[a][b]==a and adjacency_matrix[a][b]=='x'):
            return None
        else:
            x=shortest_path(a,pred[a][b])
            if(x):
                return x+[b]
            return None
    
    return dist,shortest_path


def solve(list_of_kingdom_names, starting_kingdom, adjacency_matrix, params=[]):
    """
    Input:
        list_of_kingdom_names: An list of kingdom names such that node i of the graph corresponds to name index i in the list
        starting_kingdom: The name of the starting kingdom for the walk
        adjacency_matrix: The adjacency matrix from the input file

    Output:
        Return 2 things. The first is a list of kingdoms representing the walk, and the second is the set of kingdoms that are conquered
    """
    # Define and set the index of starting kingdom based on its name
    for w in range(len(list_of_kingdom_names)):
        if(list_of_kingdom_names[w]==starting_kingdom):
            # change starting kingdom from step to step
            index_of_starting_kingdom=w
            # always store the origin
            index_of_origin=w
            
    # Record the conqure time in the array
    conquer_time=[]
    # Use Google Optimization Tools to instantiate a mixed-integer solver, naming it SolveIntegerProblem.
    solver = pywraplp.Solver('SolveIntegerProblem',
                           pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    # Stores variable names (kingdom names) and constraints for integer linear programming
    var_list=[]
    constraint=[]
    for i in range(len(adjacency_matrix)):
        conquer_time.append(adjacency_matrix[i][i])
        # each var (kingdom) is an integer in {0,1}, where 0 means not conquer and 1 means conquer.
        var_list.append(solver.IntVar(0.0,1.0,list_of_kingdom_names[i]))
        # set the constraint for each equation to make sure every kingdom is surrendered (at least one of its neighbour should be conquered)
        constraint.append(solver.Constraint(1.0,solver.infinity()))
        
    # set coefficient for each variable (kingdom) in every constraint
    for i in range(len(adjacency_matrix)):
        for j in range(len(adjacency_matrix[i])):
            if(adjacency_matrix[i][j]!='x'):
                constraint[i].SetCoefficient(var_list[j],1.0)
            else:
                constraint[i].SetCoefficient(var_list[j],0.0)
    #Case 1: Conquer kingdoms with the least total conquer time
    
    #define objective for integer linear programming
    objective = solver.Objective()
    # set conquer_time as a weight for each variable in objective function
    for i in range(len(adjacency_matrix)):
        objective.SetCoefficient(var_list[i],conquer_time[i])
    # Set a minimization integer linear programming 
    objective.SetMinimization()

    """Solve the problem and print the solution."""
    result_status = solver.Solve()
    # The problem has an optimal solution.
    assert result_status == pywraplp.Solver.OPTIMAL

    # The solution looks legit (when using solvers other than
    # GLOP_LINEAR_PROGRAMMING, verifying the solution is highly recommended!).
    assert solver.VerifySolution(1e-7, True)
    
    """
    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())

    # The objective value of the solution.
    print('Optimal objective value = %d' % solver.Objective().Value())
    print()
    
    for variable in var_list:
        print('%s = %d' % (variable.name(), variable.solution_value()))
    print()
    print('ILP ends')
    """
    # store the kingdom names that need to be conquered in conquer_list1
    conquer_list1=[]
    # store the kingdom indices that need to be conquered in conquer_index1
    conquer_index1=[]
    # initialize path_time1
    path_time1=0
    # Set values for conquer_list1, conquer_index1 and add conquer time to path_time1 (haven't added travelling time yet)
    for k in range(len(var_list)):
        if(var_list[k].solution_value()==1):
            conquer_list1.append(list_of_kingdom_names[k])
            conquer_index1.append(k)
            path_time1+=adjacency_matrix[k][k]
            
    # store the conquer tour (starts and ends at the starting_kingdom)
    index_tour=[]
    # store the kingdoms that have been conquered
    conquered=[]
    
    # initialize a counter for the following loop
    r=0
    # stored the distance matrix from FloydWarshall in dist and the inner function as path
    # dist: a 2D matrix path: a function to construct shortest path
    dist,path=FloydWarshall(adjacency_matrix)
    
    # Use a Greedy Algorithm that visits the nearest kingdom in the conquer list at every step
    while(r<len(conquer_list1)):
        min_dist=float('Inf')
        for elem in conquer_index1:
            if(elem not in conquered and dist[index_of_starting_kingdom][elem]<min_dist):
                # set the index of next kingdom to conquer
                index_of_next_kingdom=elem
                # set the min_dist based on the dist matrix from FloydWarshall
                min_dist=dist[index_of_starting_kingdom][elem]
                
        # add the sub-tour from starting_kingdom to next kingdom into index_tour
        index_tour+=path(index_of_starting_kingdom,index_of_next_kingdom)
        # pop the next kingdom to avoid double counting
        index_tour.pop()
        # add the travel time from starting_kingdom to next kingdom into path_time1
        path_time1+=dist[index_of_starting_kingdom][index_of_next_kingdom]
        # increase the counter
        r=r+1
        # update conquered kingdom list
        conquered.append(index_of_next_kingdom)
        # update the value of the starting_kingdom for the next step
        index_of_starting_kingdom=index_of_next_kingdom
        
    # add the path to go back to the starting kingdom after conquering those kingdoms in the conquer_list
    index_tour+=path(index_of_starting_kingdom,index_of_origin)
    # add the time to go back to the starting kingdom after conquering those kingdoms in the conquer_list
    path_time1+=dist[index_of_starting_kingdom][index_of_origin]
    
    # store the name of kingdoms on the tour
    tour1=[]
    # transfer from kingdom index to kingdom name
    for index in index_tour:
        tour1.append(list_of_kingdom_names[index])
    """
    print("solver's tour1=", tour1)
    print("solver's path time1= ", path_time1)
    print()
    """

    #2nd case(Conquer the least number of kingdoms):
    # Give each variable (kingdom) an equal weight in the objective function
    for i in range(len(adjacency_matrix)):
        objective.SetCoefficient(var_list[i],1)
    # Set a minimization integer linear programming 
    objective.SetMinimization()

    """Solve the problem and print the solution."""
    result_status = solver.Solve()
    # The problem has an optimal solution.
    assert result_status == pywraplp.Solver.OPTIMAL

    # The solution looks legit (when using solvers other than
    # GLOP_LINEAR_PROGRAMMING, verifying the solution is highly recommended!).
    assert solver.VerifySolution(1e-7, True)

    """
    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())

    # The objective value of the solution.
    print('Optimal objective value = %d' % solver.Objective().Value())
    print()
    
    for variable in var_list:
        print('%s = %d' % (variable.name(), variable.solution_value()))
    print()
    print('ILP ends')
    """
    # store the kingdom names that need to be conquered in conquer_list2
    conquer_list2=[]
    # store the kingdom indices that need to be conquered in conquer_index2
    conquer_index2=[]
    # initialize path_time2
    path_time2=0
    # Set values for conquer_list2, conquer_index2 and add conquer time to path_time2 (haven't added travelling time yet)
    for k in range(len(var_list)):
        if(var_list[k].solution_value()==1):
            conquer_list2.append(list_of_kingdom_names[k])
            conquer_index2.append(k)
            path_time2+=adjacency_matrix[k][k]
    
    """
    print(conquer_list2)
    print(conquer_index2)
    print("total conquer time2=", path_time2)
    """
    # store the conquer tour (starts and ends at the starting_kingdom)
    index_tour=[]
    # store the kingdoms that have been conquered
    conquered=[]
    
    # initialize a counter for the following loop
    r=0
    # stored the distance matrix from FloydWarshall in dist and the inner function as path
    # dist: a 2D matrix path: a function to construct shortest path
    dist,path=FloydWarshall(adjacency_matrix)
    
    # Use a Greedy Algorithm that visits the nearest kingdom in the conquer list at every step
    while(r<len(conquer_list2)):
        min_dist=float('Inf')
        for elem in conquer_index2:
            if(elem not in conquered and dist[index_of_starting_kingdom][elem]<min_dist):
                # set the index of next kingdom to conquer
                index_of_next_kingdom=elem
                # set the min_dist based on the dist matrix from FloydWarshall
                min_dist=dist[index_of_starting_kingdom][elem]
                
        # add the sub-tour from starting_kingdom to next kingdom into index_tour
        index_tour+=path(index_of_starting_kingdom,index_of_next_kingdom)
        # pop the next kingdom to avoid double counting
        index_tour.pop()
        # add the travel time from starting_kingdom to next kingdom into path_time1
        path_time2+=dist[index_of_starting_kingdom][index_of_next_kingdom]
        # increase the counter
        r=r+1
        # update conquered kingdom list
        conquered.append(index_of_next_kingdom)
        # update the value of the starting_kingdom for the next step
        index_of_starting_kingdom=index_of_next_kingdom
    
    # add the path to go back to the starting kingdom after conquering those kingdoms in the conquer_list
    index_tour+=path(index_of_starting_kingdom,index_of_origin)
    # add the time to go back to the starting kingdom after conquering those kingdoms in the conquer_list
    path_time2+=dist[index_of_starting_kingdom][index_of_origin]
    
    # store the name of kingdoms on the tour
    tour2=[]
    # transfer from kingdom index to kingdom name
    for index in index_tour:
        tour2.append(list_of_kingdom_names[index])
    """
    print("solver's tour2=", tour2)
    print("solver's path time2= ", path_time2)
    """
    
    # return the optimal tour based on path_time
    if(path_time1>path_time2):
        #print(tour2,conquer_list2)
        return tour2,conquer_list2
    else:
        #print(tour1,conquer_list1)
        return tour1,conquer_list1

        
"""
======================================================================
   No need to change any code below this line
======================================================================
"""

""" 
Use this function to solve a single input file.
"""
def solve_from_file(input_file, output_directory, params=[]):
    print('Processing', input_file)
    
    input_data = utils.read_file(input_file)
    number_of_kingdoms, list_of_kingdom_names, starting_kingdom, adjacency_matrix = data_parser(input_data)
    #print(len(adjacency_matrix))
    #print(len(adjacency_matrix[0]))

    closed_walk, conquered_kingdoms = solve(list_of_kingdom_names, starting_kingdom, adjacency_matrix, params=params)

    basename, filename = os.path.split(input_file)
    output_filename = utils.input_to_output(filename)
    output_file = f'{output_directory}/{output_filename}'
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)
    utils.write_data_to_file(output_file, closed_walk, ' ')
    utils.write_to_file(output_file, '\n', append=True)
    utils.write_data_to_file(output_file, conquered_kingdoms, ' ', append=True)

"""
Use this function to solve all input files.
"""
def solve_all(input_directory, output_directory, params=[]):
    input_files = utils.get_files_with_extension(input_directory, 'in')

    for input_file in input_files:
        solve_from_file(input_file, output_directory, params=params)
        os.remove(input_file)


if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Parsing arguments')
    parser.add_argument('--all', action='store_true', help='If specified, the solver is run on all files in the input directory. Else, it is run on just the given input file')
    parser.add_argument('input', type=str, help='The path to the input file or directory')
    parser.add_argument('output_directory', type=str, nargs='?', default='.', help='The path to the directory where the output should be written')
    parser.add_argument('params', nargs=argparse.REMAINDER, help='Extra arguments passed in')
    args = parser.parse_args()
    output_directory = args.output_directory
    if args.all:
        input_directory = args.input
        solve_all(input_directory, output_directory, params=args.params)
    else:
        input_file = args.input
        solve_from_file(input_file, output_directory, params=args.params)
