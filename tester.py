#!/usr/bin/env python3

import io

from solution import *

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

""" def find_non_matching_vehicle(vehicles1, vehicles2):
    for i in range(len(vehicles1)):
        if vehicles1[i] != vehicles2[i]:
            return i
    return -1

def find_preformed_action_parameters(current_vehicle, next_vehicle, next_request_list):
    current_vehicle_requests = current_vehicle[3]
    next_vehicle_requests = next_vehicle[3]

    current_vehicle_requests_len = 0
    if current_vehicle_requests != None:
        current_vehicle_requests_len = len(current_vehicle_requests)

    next_vehicle_requests_len = 0
    if next_vehicle_requests != None:
        next_vehicle_requests_len = len(next_vehicle_requests)

    if current_vehicle_requests_len > next_vehicle_requests_len:
        #dropoff action was performed

        #find the request that was dropped off
        target_request = None
        for request in current_vehicle_requests:
            if next_vehicle_requests_len == 0 or request not in next_vehicle_requests:
                target_request = request
                break

        #find the index of the request in the request list
        target_request_index = -1
        for i in range(len(next_request_list)):
            if next_request_list[i] == (target_request[0], target_request[1], target_request[2], target_request[3], 'Completed', None):
                target_request_index = i
                break

        #time of dropoff is the time of the vehicle with the smaller list
        time = next_vehicle[2]

        #return the preformed action
        return target_request_index, 'Dropoff', time
    
    else:
        #pickup action was performed
        
        #find the request that was picked up
        target_request = None
        for request in next_vehicle_requests:
            if current_vehicle_requests_len == 0 or request not in current_vehicle_requests:
                target_request = request
                break
        
        #find the index of the request in the request list
        target_request_index = -1
        for i in range(len(next_request_list)):
            if next_request_list[i] == (target_request[0], target_request[1], target_request[2], target_request[3], 'Dropoff', None):
                target_request_index = i
                break
        
        #time of pickup is the time of the vehicle with the larger list
        time = next_vehicle[2]

        #return the preformed action
        return target_request_index, 'Pickup', time
        


def recover_path(answer):
    path = []
    #iterate over nodes in answer
    for i in range(len(answer.path()) - 1):
        current_node = answer.path()[i].state
        next_node = answer.path()[i + 1].state

        #get lists of vehicles at each node
        current_vehicles = current_node[0]
        next_vehicles = next_node[0]

        #one of the vehicles was not altered. Find the one that did
        changed_vehicle_index = find_non_matching_vehicle(current_vehicles, next_vehicles)
        current_vehicle = current_vehicles[changed_vehicle_index]
        next_vehicle = next_vehicles[changed_vehicle_index]

        #find preformed action parameters
        target_request_index, action, time = find_preformed_action_parameters(current_vehicle, next_vehicle, next_node[1])

        #add the preformed action to the path
        path.append((action, changed_vehicle_index, target_request_index, time))

    return path """

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
