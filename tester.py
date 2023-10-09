#!/usr/bin/env python3

import io

from solution_test import *

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
    """ for i in range(0, 10):
        problem = FleetProblem()
        filename = f"public2/ex{i}.dat"
        print(f"Testing {filename}")
        with open(filename) as fh:
            problem.load(fh)
            answer = problem.solve()
        path = list(answer.state[3])
        cost = problem.cost(path)
        print(answer.state[3])
        print(cost) """
    problem = FleetProblem()
    filename = f"test.dat"
    print(f"Testing {filename}")
    with open(filename) as fh:
        problem.load(fh)
        answer = problem.solve()
    path = list(answer.state[3])
    cost = problem.cost(path)
    print(answer.state[3])
    print(cost)

        
if __name__=='__main__':
    main()

# EOF
