#!/usr/bin/env python3

import io

from slow_heristic_min_delay_ import *

import time

P = """
# this is a comment
P 3
1 1
  1
R 4
16 1 2 1
22 1 2 1
5  2 1 2
20 2 1 3
V 4
2
3
4
5
"""

def main():
    #start timer
    start = time.time()
    for i in range(0, 10):
        problem = FleetProblem()
        filename = f"public2/ex{i}.dat"
        print(f"Testing {filename}")
        with open(filename) as fh:
            problem.load(fh)
            path = problem.solve()
            print(f"Time elapsed: {time.time() - start}")
            #print the answer path
            """ path = recover_path(answer) """
            print(f"Answer path: {path}")
            #print the cost
            cost = problem.cost(path)
            print(f"Computed cost = {cost}")
            #print time elapsed since start

        
if __name__=='__main__':
    main()

# EOF
