# Purpose: Solution class for fleet problem

# Imports
import sys
import numpy as np
import itertools

import search

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
        if len(R) <= len(V):
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
    
    def h(self, state) :

        #recover state
        vehicles = state.state[0]
        requests = state.state[1]
        total_delay = 0.0

        #create lists of pickup and dropoff requests
        dropoff_requests = []
        pickup_requests = []

        for request in requests:
            if request[4] == "Pickup": #Completed
                pickup_requests.append(request)

            elif request[4] == "Dropoff": #Dropoff
                dropoff_requests.append(request)


        #-----------------------------Dropoff requests--------------------------------------------

        #Iterate dropoff requests and calculate the estimated delay for each one, assuming vehicle goes to each location directly
        for request in dropoff_requests:
        
            # 4.1 - Get the vehicle
            vehicle_index = request[6]
            vehicle_index = self.reverseVehicleIndexes[vehicle_index]
            vehicle = vehicles[vehicle_index]

            # 4.2 - Calculate the best possible time of dropoff
            td = vehicle[2] + self.timeMatrix[vehicle[1]][request[2]] #Best possible time of dropoff
            delay = td - request[5] - self.timeMatrix[request[1]][request[2]] #Estimated delay for dropoff only

            total_delay += delay #Update the total delay


        # But dropoffs are done in sequence, and not by the vehicle going to each location directly
        #calculate best route to visit all dropoff locations and compute the delay of travelling between them
        for new_index, vehicle in enumerate(vehicles):
            i = self.originalVehicleIndexes[new_index]

            vehicle_location = vehicle[1]

            delay = 0.0
            dropoffLocationsToVisit = []

            #Search requests associated with that vehicle and store dropoff point
            for request in dropoff_requests:

                if request[2] not in dropoffLocationsToVisit and request[2] != vehicle_location and request[6] == i:
                    dropoffLocationsToVisit.append(request[2])
            
            #find best route to pass through all locations in places
            if dropoffLocationsToVisit == []:
                continue
            else:
                possible_routes = []
                #compute all possible routes
                for route in itertools.permutations(tuple(dropoffLocationsToVisit), len(dropoffLocationsToVisit)):
                    possible_routes.append(route)

                best_route = None
                min_route_length = float('inf')

                for route in possible_routes:
                    route_length = sum(self.timeMatrix[route[i]][route[i + 1]] for i in range(len(route) - 1))

                    #go from current location to first location in route
                    route_length += self.timeMatrix[vehicle_location][route[0]]
                
                    # Check if this route is shorter than the current best
                    if route_length < min_route_length:
                        best_route = route
                        min_route_length = route_length
                    else:
                        continue

            #Compute a estimate total delay for that vehicle dropoff requests
            for j in range(len(best_route)-1):
                delay += self.timeMatrix[j][j+1]
            
            total_delay += delay

            #remove from total_delay, the time from the vehcile location to each of the places indepedently
            for location in dropoffLocationsToVisit:
                if location != dropoffLocationsToVisit[0]:
                    total_delay -= self.timeMatrix[vehicle_location][location]

        #---------------------------Pickup requests---------------------------------------------

        #Iterate pickup requests and calculate the best estimated delay for each one
        for request in pickup_requests:

            #List of possible times of pickup
            possiblePickupTimes = []
        
            #Iterate through vehicles
            for new_index2, vehicle in enumerate(vehicles): 
                vehicle_index = self.originalVehicleIndexes[new_index2]

                #Initial time (vehicle timestamp)
                tp = vehicle[2]
                
                #if the capcatiy of the vehicle when it is empty is less than the capacity of the request, then the vehicle will never be able to perform that request
                if self.vehicleList[vehicle_index] < request[3]:
                    continue

                # If vehicle has capacity, then add possible pick up time
                if vehicle[0] >= request[3]:
                    tp = max(tp + self.timeMatrix[vehicle[1]][request[1]], request[0]) #Best possible time of pickup
                    possiblePickupTimes.append(tp)
            
                #vehicle must go to other locations to dropoff passengers before picking up new ones
                else:
                    vehicle_location = vehicle[1]
                    pickupLocation = request[1]
                    points_to_visit = []
                    visited_points = set()

                    #Create a list of points to visit (excluding the starting point and the final point)
                    for req in dropoff_requests:
                        if req[2] not in visited_points and req[6] == vehicle_index:
                            points_to_visit.append(req[2])
                            visited_points.add(req[2]) #no repeated points

                    #Create all combinations of possible routes (including different sizes of routes)
                    if points_to_visit == []:
                        routes = ([vehicle_location, pickupLocation],)

                    else: #Dropoffs to be made in in other points
                        routes = []

                        for routeLength in range(len(points_to_visit)):
                            for combo in itertools.permutations(tuple(points_to_visit), routeLength + 1):
                                routes.append([vehicle_location] + list(combo) + [pickupLocation])

                    best_route = None
                    min_route_length = float('inf')

                    for route in routes:
                        # Initialize available capacity
                        available_capacity = vehicle[0]

                        for i in range(len(route)):
                            # Check how many passengers are to be dropped off at this point
                            #if start and final points are the same, do not add capacity for dropoffs twice! So ignore first
                            if i == 0 and vehicle_location == pickupLocation:
                                continue
                            for req in dropoff_requests:
                                if req[2] == route[i] and req[6] == vehicle_index:
                                    available_capacity += req[3]
                        
                        # Check if the available capacity allows picking up the new request
                        if available_capacity >= request[3]:
                            # Calculate the length of the route
                            route_length = sum(self.timeMatrix[route[i]][route[i + 1]] for i in range(len(route) - 1))
                        
                            # Check if this route is shorter than the current best
                            if route_length < min_route_length:
                                best_route = route
                                min_route_length = route_length
            
                    tp = max(tp + min_route_length, request[0])#Best possible time of pickup for that vehicle
                    possiblePickupTimes.append(tp)
      
            tp = min(possiblePickupTimes)
            delay = tp - request[0] 


            #Update the total delay
            total_delay += delay
        
        return total_delay
    
    def solve(self):
        #Before calling the search algorithm, in certain conditions, we can already know some of the actions that will be taken
        #Specifically, if there are more vehicles than requests, we can assign a vehicle to each requests, and thus each vehicle only handles one request
        
        V = list(self.initial[0])
        R = list(self.initial[1])

        answer = []

        #if there are more vehicles than requests
        if len(V) >= len(R):
            #sort requests by capacity, highest capacity first
            sorted_requests = sorted(R, key=lambda x: x[3], reverse=True)

            requestOriginalIndexes = []
            for sorted_request in sorted_requests:
                for i, request in enumerate(R):
                    if sorted_request == request:
                        requestOriginalIndexes.append(i)
                        break
                
            print("Vehicles capacity: ", [v[0] for v in V])
            print("sorted requests capacity: ", [r[3] for r in sorted_requests])


            #vehicles are sorted by capacity, highest capacity first
            #requests are sorted by capacity, highest capacity first
            #therefore, we can assign each vehicle to a request in the same order
            for rindex, request in enumerate(sorted_requests):
                pickupPointIndex = request[1]
                vindex = self.originalVehicleIndexes[rindex]
                vehicle = V[self.reverseVehicleIndexes[vindex]]

                #calculate pickup time
                pickupTime = max(vehicle[2] + self.timeMatrix[vehicle[1]][pickupPointIndex], request[0])

                #update request
                R[requestOriginalIndexes[rindex]] = (request[0], request[1], request[2], request[3], "Dropoff", pickupTime, vindex)

                #update vehicle
                V[self.reverseVehicleIndexes[vindex]] = (vehicle[0] - request[3], request[1], pickupTime)

                #add action to answer
                answer.append(("Pickup", vindex, requestOriginalIndexes[rindex], pickupTime))

            #create new initial state
            self.initial = (tuple(V), tuple(R))

        #call search algorithm

        path = search.astar_search(self, display=True).solution()

        #add actions in path to answer
        for action in path:
            answer.append(action)
   
        return answer

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

#Add new criteria to define priority of nodes
#Priority is defined by the number of completed actions
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