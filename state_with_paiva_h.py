# Purpose: Solution class for fleet problem

# Imports
import sys
import numpy as np
import itertools

import search

# TODO: Add current_time updates to heuristic. This will involve:
#       - sorting each chain so that requests with the same next point are in group
#       - checking if location changed when iterating through requests in chain
#       - if location changed, update current_time to the time it takes to go from last location to current location
#       - should remain admissible:
#           - since we are assuming we have infinite vehicles with infinite capacity
#           - we are not accounting for dropoff actions that will appear after completing pickups

#=======================================================================================================================================

# TODO: Check how input file is passed (argument or stdin)
    #Answer: load method receives a file object as argument
# TODO: Ask what is "import search and search.Problem"
    #Answer: search is a module from the book's github repo that contains classes of algorithms that will be used in deliverables 2 and 3
# TODO: Ask how to present output of cost() method (.txt file or stdout)
    #Answer: cost() method should return J, the cost of the solution

#=======================================================================================================================================

# Solution class

class FleetProblem(search.Problem):

    #Attributes
    timeMatrix = None
    requestList = None
    vehicleList = None
    J = None

    numberOfRequests = None
    numberOfVehicles = None
    originalVehicleIndexes = None

    initial = None

    standardTimeUnit = 0

    #Methods 

    #constructor
    def __init__(self):
        self.requestList = []
        self.vehicleList = []
        self.originalVehicleIndexes = []
        self.reverseVehicleIndexes = []
        self.J = 0
        self.numberOfRequests = 0
        self.numberOfVehicles = 0
    
    #readLines method that ignores lines that start with '#'
    #reads lines until it finds a line that doesn't start with '#'
    def readLines(self, fh):
        line = fh.readline()
        splittedLine = line.split()
        #ignore lines that start with '#' and empty lines
        while len(splittedLine) < 1 or (len(splittedLine) >= 1 and splittedLine[0] == '#'):
            #if line is empty, return None => end of file
            if line == '':
                return None
            
            #read next line
            line = fh.readline()
            splittedLine = line.split()
        return line
        
    
    def load(self, fh):
        #do while loop to read lines until it finds a line that doesn't start with '#'
        currentLine = self.readLines(fh)
        while currentLine != None:
            if currentLine.split()[0] == 'P':
                self.readP(fh, currentLine)
            elif currentLine.split()[0] == 'R':
                self.readR(fh, currentLine)
            elif currentLine.split()[0] == 'V':
                self.readV(fh, currentLine)
            else:
                raise Exception("Invalid file format.")
            currentLine = self.readLines(fh)
        
        #standard time unit is the avegare time to go from one point to another, not counting the zeros in the time matrix
        #calculate standard time unit
        numberOfNonZeroElements = 0
        sumOfNonZeroElements = 0
        for i in range(len(self.timeMatrix)):
            for j in range(len(self.timeMatrix[i])):
                if self.timeMatrix[i][j] != 0:
                    numberOfNonZeroElements += 1
                    sumOfNonZeroElements += self.timeMatrix[i][j]
        self.standardTimeUnit = sumOfNonZeroElements / numberOfNonZeroElements
        #standard time unit adjustment
        self.standardTimeUnit = self.standardTimeUnit * 1

        #create initial state
        V = []
        for vindex, v in enumerate(self.vehicleList):
            V.append((int(v), 0, 0, vindex))
        self.originalVehicleIndexes = [v[3] for v in V]
        self.reverseVehicleIndexes = [v[3] for v in V]
        R = []
        for r in self.requestList:
            R.append((r[0], int(r[1]), int(r[2]), int(r[3]), "Pickup", None, None))
        R = tuple(R)
        if len(R) < len(V):
            #more requests than vehicles
            #remove vehicles so that there are the same number of vehicles as requests, removing the ones with the lowest capacity
            self.reverseVehicleIndexes = [None for v in V]
            V = sorted(V, key=lambda x: x[0], reverse=True)
            V = V[:len(R)]
            self.originalVehicleIndexes = [v[3] for v in V]
            for vindex, v in enumerate(V):
                self.reverseVehicleIndexes[v[3]] = vindex
            V = [(v[0], v[1], v[2]) for v in V]
        V = tuple(V)
        self.initial = (V, R)
        pass

    def readP(self, fh, pLine):
        #read number of points and time matrix; store in variable and matrix
        #line is "P number_of_points"
        pLine = pLine.split()
        numberOfPoints = int(pLine[1])

        #read time matrix
        self.timeMatrix = np.zeros((numberOfPoints, numberOfPoints))
        for i in range(numberOfPoints - 1):
            line = self.readLines(fh)
            line = line.split()
            #convert to int
            line = list(map(float, line))
            for j in range(numberOfPoints - 1 - i):
                self.timeMatrix[i][j + i + 1] = line[j]
                self.timeMatrix[j + i + 1][i] = line[j]
        
        pass

    def readR(self, fh, rLine):
        #read number of requests and request tuples; store in variable and list
        #line is "R number_of_requests"
        rLine = rLine.split()
        self.numberOfRequests = int(rLine[1])

        #read request tuples
        #line is "pickup_time pickup_point dropoff_point number_of_passengers"
        self.requestList = []
        for i in range(self.numberOfRequests):
            line = self.readLines(fh)
            line = line.split()
            #convert to int
            line = list(map(float, line))
            pickupTime = line[0]
            pickupPoint = line[1]
            dropoffPoint = line[2]
            numberOfPassengers = line[3]
            self.requestList.append((pickupTime, pickupPoint, dropoffPoint, numberOfPassengers))

        pass

    def readV(self, fh, vLine):
        #read number of vehicles and vehicle capacities; store in variable and list
        #line is "V number_of_vehicles"
        vLine = vLine.split()
        self.numberOfVehicles = int(vLine[1])

        #read vehicle capacities
        #line is "vehicle_capacity"
        self.vehicleList = []
        for i in range(self.numberOfVehicles):
            line = self.readLines(fh)
            line = line.split()
            #convert to int
            line = list(map(float, line))
            vehicleCapacity = line[0]
            self.vehicleList.append(vehicleCapacity)
        
        pass
    
    def result(self, state, action):    
        newV = list(state[0])
        newR = list(state[1])
        for i in range(len(newR)):
            newR[i] = list(newR[i])
        for i in range(len(newV)):
            newV[i] = list(newV[i])
        newT = None
        #discover vehicle in use
        vehicleInUseIndex = action[1]
        #discover request fulfilled
        requestFulfilledIndex = action[2]
        #state timestamp equals action timestamp
        newT = action[3]
        #add action to path
        if action[0] == "Pickup":

            newR[requestFulfilledIndex][4] = "Dropoff"
            newR[requestFulfilledIndex][5] = newT
            newR[requestFulfilledIndex][6] = vehicleInUseIndex

            newV[self.reverseVehicleIndexes[vehicleInUseIndex]][1] = newR[requestFulfilledIndex][1]
            newV[self.reverseVehicleIndexes[vehicleInUseIndex]][0] = newV[self.reverseVehicleIndexes[vehicleInUseIndex]][0] - newR[requestFulfilledIndex][3]
            newV[self.reverseVehicleIndexes[vehicleInUseIndex]][2] = newT
        else:

            newR[requestFulfilledIndex][4] = "Completed"
            newR[requestFulfilledIndex][5] = None
            newR[requestFulfilledIndex][6] = None

            newV[self.reverseVehicleIndexes[vehicleInUseIndex]][1] = newR[requestFulfilledIndex][2]
            newV[self.reverseVehicleIndexes[vehicleInUseIndex]][0] = newV[self.reverseVehicleIndexes[vehicleInUseIndex]][0] + newR[requestFulfilledIndex][3]
            newV[self.reverseVehicleIndexes[vehicleInUseIndex]][2] = newT
        for i in range(len(newR)):
            newR[i] = tuple(newR[i])
        for i in range(len(newV)):
            newV[i] = tuple(newV[i])
        newV = tuple(newV)
        newR = tuple(newR)
        newState = (newV, newR)

        return newState
    
    def actions(self, state):
        V = state[0]
        R = state[1]
    
        possibleActions = []
        for rindex, request in enumerate(R):
            if request[4] == "Pickup":
                for newvindex, vehicle in enumerate(V):
                    vindex = self.originalVehicleIndexes[newvindex]
                    #vehicle capacity is enough for requested number of passengers
                    if vehicle[0] >= request[3]:
                        #check if vehicle can reach the pickup location after request has been triggered
                        pickUpPointIndex = request[1]
                        dropOffPointIndex = request[2]
                        #first case: vehicle goes directly from vehicle["location"] to request pickup point, meaning that the action timestamp is vehicle["lastLocationTimeStamp"] + (time from vehicle location to request pickup location)
                        projectedTime = vehicle[2] + self.timeMatrix[vehicle[1]][pickUpPointIndex]
                        if projectedTime >= request[0]:
                            possibleActions.append(("Pickup", vindex, rindex, projectedTime))
                        #second case: vehicle waits for pickup, meaning that action timestamp is equal to request timestamp > vehicle["lastLocationTimeStamp"] + (time from vehicle location to request pickup location)
                        projectedTime = request[0]
                        #only valid if request pickup time is greater than vehicle["lastLocationTimeStamp"] + (time from vehicle location to request pickup location)
                        if projectedTime > vehicle[2] + self.timeMatrix[vehicle[1]][pickUpPointIndex]:
                            possibleActions.append(("Pickup", vindex, rindex, projectedTime))
            if request[4] == "Dropoff":
                #the only vehicle capable of preforming a request with "Dropoff" status is the vehicle with the corresponding request with "Pickup" status in its requestList
                vindex = request[6]
                vehicle = V[self.reverseVehicleIndexes[vindex]]
                #check if vehicle can reach the pickup location after request has been triggered
                pickUpPointIndex = request[1]
                dropOffPointIndex = request[2]
                #only one case this time: vehicle goes directly from vehicle["location"] to request pickup point, meaning that the action timestamp is vehicle["lastLocationTimeStamp"] + (time from vehicle location to request pickup location)
                projectedTime = vehicle[2] + self.timeMatrix[vehicle[1]][dropOffPointIndex]
                if projectedTime >= request[0]:
                    possibleActions.append(("Dropoff", vindex, rindex, projectedTime))

        return possibleActions
                
    def goal_test(self, state):
        for request in state[1]:
            if request[4] != "Completed":
                return False
        return True
    
    def getNextPointLocation(self, R, requestIndex):
        if R[requestIndex][4] == "Pickup":
            return R[requestIndex][1]
        elif R[requestIndex][4] == "Dropoff":
            return R[requestIndex][2]
        else:
            return None
    
    def h(self, state) :
        """"Return the heuristic value for the given state """

        # 1 - Get the current state information
        requests = state.state[1]
        vehicles = state.state[0]
        total_delay = 0.0

        # 2 - Initialize dropoff list and pickup list 
        dropoff_requests = []
        pickup_requests = []


        # 3 - Group requests by status 
        for request in requests:
            if request[4] == "Completed": #Completed
                continue

            elif request[4] == "Dropoff": #Dropoff
                dropoff_requests.append(request) #Group them by vehicle index

            else: #Pickup
                pickup_requests.append(request)


        # 4 - Iterate dropoff requests and calculate the estimated delay for each one
        for i, request in enumerate(dropoff_requests):
        
            # 4.1 - Get the vehicle index
            vehicle_index = request[6]
            vehicle_index = self.reverseVehicleIndexes[vehicle_index]
            vehicle = vehicles[vehicle_index]

            # 4.2 - Calculate the best possible time of dropoff
            td = vehicle[2] + self.timeMatrix[vehicle[1]][request[2]] #Best possible time of dropoff
            delay = td - request[5] - self.timeMatrix[request[1]][request[2]] #Estimated delay for dropoff only

            total_delay += delay #Update the total delay


        # 5 - For each vehicle, let's determine which dropoff points does it have to visit to fulfill the dropoff requests and compute a delay 
        # caused by travelling between those points

        for new_index, vehicle in enumerate(vehicles):
            i = self.originalVehicleIndexes[new_index]

            #Initialize variables
            delay = 0.0
            places = []
            num_req = 0

            #Search requests associated with that vehicle and store dropoff point
            for request in dropoff_requests:
                if request[6] == i:
                    num_req += 1

                if request[2] not in places:
                    places.append(request[2])

            #Compute a estimate total delay for that vehicle dropoff requests
            if num_req == 0:
                continue
            else:
                for j in range(len(places)-1):
                    delay += self.timeMatrix[j][j+1]
                
                total_delay += delay


        # 6 - Iterate pickup requests and calculate the best estimated delay for each one
        for request in pickup_requests:

            # 6.1 - List of possible times of pickup
            possible_times = []
        
            # 6.2 - Iterate through vehicles
            for new_index2, vehicle in enumerate(vehicles): 
                j = self.originalVehicleIndexes[new_index2]

                #Initial time (vehicle timestamp)
                tp = vehicle[2]

                # 6.3 - There is capacity
                if vehicle[0] >= request[3]:
                    tp = max(tp + self.timeMatrix[vehicle[1]][request[1]], request[0]) #Best possible time of pickup
                    possible_times.append(tp)
            
                else: # 6.4 - We have to do dropoffs first to free up capacity

                    #6.4.1 - Define starting point and final point
                    start_point = vehicle[1]
                    final_point = request[1]
                    points_to_visit = []
                    visited_points = set()
                    z = 0 #Indicator for next step

                    #6.4.2 - Create a list of points to visit (excluding the starting point and the final point)
                    for req in dropoff_requests:
                        if req[2] not in visited_points and req[6] == j:
                            points_to_visit.append(req[2])
                            visited_points.add(req[2]) #no repeated points

                    #6.4.3 - Create combos of possible routes
                    if points_to_visit == [] and start_point != final_point: #The only dropoffs to be made is in the final point or/and start point
                        routes = ([start_point, final_point],)

                    elif points_to_visit == [] and start_point == final_point: #Dropoffs to be made in the starting point and pickup in the same point
                        routes = ([start_point],)

                    elif start_point == final_point: #Dropoffs to be made in in other points (but start and final point are the same)
                        routes = []
                        z = 1 

                        for r in range(1, len(points_to_visit) + 1):
                            for combo in itertools.permutations(tuple(points_to_visit), r):
                                routes.append([start_point] + list(combo) + [final_point])

                    else: #Dropoffs to be made in in other points (start and final point are different)
                        routes = []

                        for r in range(1, len(points_to_visit) + 1):
                            for combo in itertools.permutations(tuple(points_to_visit), r):
                                routes.append([start_point] + list(combo) + [final_point])

                    
                    #6.4.4 - Initialize variables to keep track of the best route and its length

                    #print(routes)
                    best_route = None
                    min_route_length = float('inf')

                    
                    #6.4.5 - Iterate through all possible routes and calculate route lengths
                    for route in routes:

                        # Initialize available capacity
                        available_capacity = vehicle[0]

                        for i in range(len(route) - z):
                            # Check how many passengers are to be dropped off at this point
                            for req in dropoff_requests:
                                if req[2] == route[i] and req[6] == j:
                                    available_capacity += req[3]
                        
                        # Check if the available capacity allows picking up the new request
                        if available_capacity >= request[3]:
                            # Calculate the length of the route
                            route_length = sum(self.timeMatrix[route[i]][route[i + 1]] for i in range(len(route) - 1))
                        
                            # Check if this route is shorter than the current best
                            if route_length < min_route_length:
                                best_route = route
                                min_route_length = route_length
                        else:
                            continue

                    if min_route_length == float('inf'): #No route was found
                        continue
            
                    tp = max(tp + min_route_length, request[0])#Best possible time of pickup for that vehicle
                    possible_times.append(tp)


            # 6.5 - Get the best possible time of pickup from all the possible pickup times from each vehicle       
            tp = min(possible_times)
            
            # 6.6 - Calculate the actual delay for that pickup
            delay = tp - request[0] 


            #Update the total delay
            total_delay += delay


        #7 - Count the number of pending pickup requests and see how many vehicles. There will be an added delay for each request 
        # but it will be divided by the number of vehicles

        num_pickup_requests = len(pickup_requests)
        num_vehicles = len(vehicles)

        #Initialize variables
        delay = 0.0
        places = []

        #Search requests associated with that vehicle and store dropoff point
        for request in sorted(pickup_requests, key=lambda x: x[0]):    
            if request[2] not in places:
                places.append(request[2])

        #Compute a estimate total delay for that vehicle dropoff requests
        for j in range(len(places)-1):
            delay += self.timeMatrix[j][j+1]
        
        delay = delay/num_vehicles

        total_delay += delay 

        
        # 9- Return the total delay
        return total_delay
    
    def solve(self):
        # path = search.uniform_cost_search(self, display=True).solution()
        path = search.astar_search(self, display=True).solution()
   
        return path

    def path_cost(self, c, state1, action, state2):
        request = state1[1][action[2]]

        if(action[0] == "Pickup"):
            delay = action[3] - request[0]
            return c + delay
        else:
            Tod = self.timeMatrix[request[1]][request[2]]

            delay = action[3] - request[5] - Tod

            return c + delay

    def cost(self, sol):
        #filter sol to get only tuples with field in index 0 == 'Dropoff'
        try:
            filteredSol = filter(lambda x: x[0] == 'Dropoff', sol)

            filteredSol = list(filteredSol)

            #raise expetion if filteredSol is empty
            if len(filteredSol) == 0:
                raise Exception("No dropoff actions in provided solution.")
        except Exception as e:
            print(e)
        except:
            print("No dropoff actions in provided solution.")
            return 99999

        #iterate through sol list of tuples and calculate total cost
        for action in filteredSol:
            td = action[3]

            actionRequestIndex = action[2]
            actionRequest = self.requestList[actionRequestIndex]
            t = actionRequest[0]

            pickUpPointIndex = actionRequest[1]
            dropOffPointIndex = actionRequest[2]
            Tod = self.timeMatrix[int(pickUpPointIndex)][int(dropOffPointIndex)]

            dr = td - t - Tod

            self.J += dr
        
        #return total cost
        return self.J


class MyNode(search.Node):
    def __lt__(self, other):
        self_requests = self.state[1]
        other_requests = other.state[1]
        self_rating = 0
        other_rating = 0
        for request in self_requests:
            if request[4] == "Completed":
                self_rating += 2
            elif request[4] == "Dropoff":
                self_rating += 1
        for request in other_requests:
            if request[4] == "Completed":
                other_rating += 2
            elif request[4] == "Dropoff":
                other_rating += 1
        return self_rating > other_rating
    
search.Node = MyNode