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
        
# Python Program for Floyd Warshall Algorithm

# Solves all pair shortest path via Floyd Warshall Algrorithm
def FloydWarshall(adjacency_matrix):
    # simple python implemetation of Floyd-Warshall alghorithm
    # https://en.wikipedia.org/wiki/Floyd%E2%80%93Warshall_algorithm#Path_reconstruction
    # source: https://jlmedina123.wordpress.com/2014/05/17/floyd-warshall-algorithm-in-python/
    # author: J. Luis Medina
    dist = {}
    pred = {}
    for u in range(len(adjacency_matrix)):
        dist[u] = {}
        pred[u] = {}
        for v in range(len(adjacency_matrix)):
            dist[u][v] = float('Inf')
            pred[u][v] = -1
        dist[u][u] = 0

        for v in range(len(adjacency_matrix)):
            if(adjacency_matrix[u][v]!='x'):
                dist[u][v] = adjacency_matrix[u][v]
            else:
                dist[u][v]=float('Inf')
            pred[u][v] = u

    for t in range(len(adjacency_matrix)):
        # given dist u to v, check if path u - t - v is shorter
        for u in range(len(adjacency_matrix)):
            for v in range(len(adjacency_matrix)):
                newdist = dist[u][t] + dist[t][v]
                if newdist < dist[u][v]:
                    dist[u][v] = newdist
                    pred[u][v] = pred[t][v] # route new path through t

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
    Write your algorithm here.
    Input:
        list_of_kingdom_names: An list of kingdom names such that node i of the graph corresponds to name index i in the list
        starting_kingdom: The name of the starting kingdom for the walk
        adjacency_matrix: The adjacency matrix from the input file

    Output:
        Return 2 things. The first is a list of kingdoms representing the walk, and the second is the set of kingdoms that are conquered
    """
    
    if(len(list_of_kingdom_names)<200):
        return [1],[1]
    
    conquer_time=[]
    # Instantiate a mixed-integer solver, naming it SolveIntegerProblem.
    solver = pywraplp.Solver('SolveIntegerProblem',
                           pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    var_list=[]
    constraint=[]
    for i in range(len(adjacency_matrix)):
        conquer_time.append(adjacency_matrix[i][i])
        # each var is an integer in {0,1}.
        var_list.append(solver.IntVar(0.0,1.0,list_of_kingdom_names[i]))
        constraint.append(solver.Constraint(1.0,solver.infinity()))
        
        # set constraint for each kingdom
    for i in range(len(adjacency_matrix)):
        for j in range(len(adjacency_matrix[i])):
            if(adjacency_matrix[i][j]!='x'):
                constraint[i].SetCoefficient(var_list[j],1.0)
            else:
                constraint[i].SetCoefficient(var_list[j],0.0)

    #Case 1: conquer kingdoms with the least total conquer time
    #set objective
    objective = solver.Objective()
    for i in range(len(adjacency_matrix)):
        objective.SetCoefficient(var_list[i],conquer_time[i])
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
    conquer_list1=[]
    conquer_index=[]
    path_time1=0
    for k in range(len(var_list)):
        if(var_list[k].solution_value()==1):
            conquer_list1.append(list_of_kingdom_names[k])
            conquer_index.append(k)
            path_time1+=adjacency_matrix[k][k]
    
    for w in range(len(list_of_kingdom_names)):
        if(list_of_kingdom_names[w]==starting_kingdom):
            index_of_starting_kingdom=w
            index_of_origin=w
    #print(conquer_list1)
    #print(conquer_index)
    
    index_tour=[]
    conquered=[]

    r=0
    dist,path=FloydWarshall(adjacency_matrix)
    
    #print("total conquer time1=", path_time1)
    
    while(r<len(conquer_list1)):
        min_dist=float('Inf')
        for elem in conquer_index:
            if(elem not in conquered and dist[index_of_starting_kingdom][elem]<min_dist):
                index_of_next_kingdom=elem
                min_dist=dist[index_of_starting_kingdom][elem]
        index_tour+=path(index_of_starting_kingdom,index_of_next_kingdom)
        index_tour.pop()
        path_time1+=dist[index_of_starting_kingdom][index_of_next_kingdom]
        r=r+1
        conquered.append(index_of_next_kingdom)
        index_of_starting_kingdom=index_of_next_kingdom
    index_tour+=path(index_of_starting_kingdom,index_of_origin)
    path_time1+=dist[index_of_starting_kingdom][index_of_origin]

    tour1=[]
    for index in index_tour:
        tour1.append(list_of_kingdom_names[index])
    #print("solver's tour1=", tour1)
    #print("solver's path time1= ", path_time1)
    #print()
    #print()

    #2nd case(conquer the least number of kingdoms):
    for i in range(len(adjacency_matrix)):
        objective.SetCoefficient(var_list[i],1)
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
    rint('Optimal objective value = %d' % solver.Objective().Value())
    print()
    
    for variable in var_list:
        print('%s = %d' % (variable.name(), variable.solution_value()))
    print()
    print('ILP ends')
    """
    
    conquer_list2=[]
    conquer_index=[]
    path_time2=0
    for k in range(len(var_list)):
        if(var_list[k].solution_value()==1):
            conquer_list2.append(list_of_kingdom_names[k])
            conquer_index.append(k)
            path_time2+=adjacency_matrix[k][k]
    
    for w in range(len(list_of_kingdom_names)):
        if(list_of_kingdom_names[w]==starting_kingdom):
            index_of_starting_kingdom=w
            index_of_origin=w
    
    #print(conquer_list2)
    #print(conquer_index)
    #print("total conquer time2=", path_time1)
    
    index_tour=[]
    conquered=[]

    r=0
    dist,path=FloydWarshall(adjacency_matrix)
    
    while(r<len(conquer_list2)):
        min_dist=float('Inf')
        for elem in conquer_index:
            if(elem not in conquered and dist[index_of_starting_kingdom][elem]<min_dist):
                index_of_next_kingdom=elem
                min_dist=dist[index_of_starting_kingdom][elem]
        index_tour+=path(index_of_starting_kingdom,index_of_next_kingdom)
        index_tour.pop()
        path_time2+=dist[index_of_starting_kingdom][index_of_next_kingdom]
        r=r+1
        conquered.append(index_of_next_kingdom)
        index_of_starting_kingdom=index_of_next_kingdom
    index_tour+=path(index_of_starting_kingdom,index_of_origin)
    path_time2+=dist[index_of_starting_kingdom][index_of_origin]

    tour2=[]
    for index in index_tour:
        tour2.append(list_of_kingdom_names[index])
    #print("solver's tour2=", tour2)
    #print("solver's path time2= ", path_time2)

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
