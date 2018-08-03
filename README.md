# CS170 project
Edited by Jierui Lin Aug 02, 2018

See the project_spec pdf for detailed instruction of this project!

This NP-hard problem looks very similar to the Travelling Salesman problem, but there are a few differeneces:

    1. TSP is generally a complete graph while our inputs can be any graph
    2. TSP only cares about travelling distance/time while we need to take conquer time into account
    3. TSP needs to visit every node while we only need to conquer a subset of nodes
    
Based on some research, we find the problem setting is close to dominating set (get the tour that conquer least number of kingdoms)
and weighted dominating set (get the tour that conquer kingdoms with least total conquer time). And I tried different optimization
tools to solve the integer linear programming, which turns out the Mixed-Integer Programming tool from Google Optimization Tools gives
us the best solution.

We also use Floyd-Warshall algorithm (an import algorithm in Dynamic Programming that finds all-pair shortest path) and Greedy Algorithm
to find the nearest kindom to visit at every step.

In the end, we manually take care of some special cases (since we need to create difficult inputs for other groups), like a line, which 
we generally avoid going back and forward.

The result for this project is quite well that we rank at top 10% in both input difficulty and output optimality.
