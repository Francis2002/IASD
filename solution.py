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

    initial = None

    standardTimeUnit = 0

    #Methods 

    #constructor
    def __init__(self):
        self.requestList = []
        self.vehicleList = []
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
        for v in self.vehicleList:
            V.append((int(v), 0, 0))
        V = tuple(V)
        R = []
        for r in self.requestList:
            R.append((r[0], int(r[1]), int(r[2]), int(r[3]), "Pickup", None, None))
        R = tuple(R)
        T = 0
        self.initial = (V, R, T)
        self.initial = (V, R, T)
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

            newV[vehicleInUseIndex][1] = newR[requestFulfilledIndex][1]
            newV[vehicleInUseIndex][0] = newV[vehicleInUseIndex][0] - newR[requestFulfilledIndex][3]
            newV[vehicleInUseIndex][2] = newT
        else:

            newR[requestFulfilledIndex][4] = "Completed"
            newR[requestFulfilledIndex][5] = None
            newR[requestFulfilledIndex][6] = None

            newV[vehicleInUseIndex][1] = newR[requestFulfilledIndex][2]
            newV[vehicleInUseIndex][0] = newV[vehicleInUseIndex][0] + newR[requestFulfilledIndex][3]
            newV[vehicleInUseIndex][2] = newT
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
                for vindex, vehicle in enumerate(V):
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
                vehicle = V[vindex]
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
    
    def h(self, state):
        current_state = state.state
        V = current_state[0]
        R = current_state[1]

        #calculate current time by iterating through vehicles and getting the maximum timestamp
        currentTime = 0
        for vehicle in V:
            if vehicle[2] > currentTime:
                currentTime = vehicle[2]

        #create chains of requests that have the next stage (pickup or dropoff) point in proximity, that is:
        #if a request has status "Pickup", the next point is the pickup point
        #if a request has status "Dropoff", the next point is the dropoff point
        #if a request has status "Completed", it has no next point
        #chains must have requests where the next point is in both time and location proximity as another request's next point
        #proximity in time means that the difference between the request time and the next point time is less than the standard time unit
        #we must have in mind that pickup actions can only be done if the request time is less than the pickup action timestamp
        #we must have in mind that dropoff actions can only be done if the corresponding pickup action has been done

        #create list of requests in proximity
        #on each index, we have a list of requests that are in proximity with the request in the index
        requestsInProximity = []
        for i in range(len(R)):
            requestsInProximity.append([])
        for i in range(len(R)):
            #determine next point for requests index i
            inextPoint = self.getNextPointLocation(R, i)
            for j in range(len(R)):
                #determine next point for requests index j
                jnextPoint = self.getNextPointLocation(R, j)
                #check if requests are in proximity
                if i != j and inextPoint != None and jnextPoint != None:
                    #check if next points are in proximity
                    if self.timeMatrix[inextPoint][jnextPoint] == 0.0:
                        requestsInProximity[i].append(j)
        
        #for each location proximity list, remove requests that are not in time proximity
        #create minimum request execution time list
        minimumRequestExecutionTime = [None]*len(R)
        for i in range(len(R)):
            #if request is pickup, minimum execution time is the request time, but only if the request time is more than the current time
            if R[i][4] == "Pickup":
                if R[i][0] > currentTime:
                    minimumRequestExecutionTime[i] = R[i][0]
                else:
                    minimumRequestExecutionTime[i] = currentTime
            #if request is dropoff, minimum execution time is the current time
            elif R[i][4] == "Dropoff":
                minimumRequestExecutionTime[i] = currentTime
        
        #for each location proximity list, remove requests that are not in time proximity
        for index, requestList in enumerate(requestsInProximity):
            for requestIndex in requestList:
                if abs(minimumRequestExecutionTime[requestIndex] - minimumRequestExecutionTime[index]) > self.standardTimeUnit:
                    requestsInProximity[index].remove(requestIndex)
        
        #now we have lists of requests that are in proximity
        #we must now create chains of requests that are in proximity
        #a chain is a list of requests where consecutive requests are in proximity with each other
        #we must create the longest possible chains
        
        #create list of chains
        chains = []
        #create list of requests that have already been added to a chain
        requestsAlreadyAdded = []
        #iterate through requests
        for i in range(len(R)):
            #if request has not been added to a chain yet
            if i not in requestsAlreadyAdded:
                #check if request can be added to an existing chain
                #iterate through chains
                for chain in chains:
                    #check if request is in proximity with last request in chain
                    if i in requestsInProximity[chain[-1]]:
                        #add request to chain
                        chain.append(i)
                        #add request to list of requests that have already been added to a chain
                        requestsAlreadyAdded.append(i)
                        break
                #if request cannot be added to an existing chain, create new chain
                if i not in requestsAlreadyAdded:
                    chains.append([i])
                    requestsAlreadyAdded.append(i)

        #sort requests in each chain by location of next point
        #baba sort
        for chain in chains:
            for request in chain:
                nextPoint = self.getNextPointLocation(R, request)
                if nextPoint != None:
                    for i in range(len(chain)):
                        if i != request:
                            inextPoint = self.getNextPointLocation(R, i)
                            if inextPoint != None:
                                if self.timeMatrix[nextPoint][inextPoint] < self.timeMatrix[nextPoint][nextPoint]:
                                    chain[i], chain[request] = chain[request], chain[i]
                                    break

        #now we have a list of chains
        #we must now calculate the cost of completing every chain

        #save current time
        originalCurrentTime = currentTime
        #create list of chain costs
        chainCosts = []
        #iterate through chains
        for chain in chains:
            #calculate chain cost
            chainCost = 0
            #iterate through requests in chain
            for i, requestIndex in enumerate(chain):
                #check if location changed
                if i > 0:
                    lastPointLocation = self.getNextPointLocation(R, chain[i - 1])
                    currentPointLocation = self.getNextPointLocation(R, requestIndex)
                    if lastPointLocation != currentPointLocation:
                        #update current time
                        currentTime = currentTime + self.timeMatrix[lastPointLocation][currentPointLocation]
                #if request is pickup, add time current_time - request_time to chain cost
                if R[requestIndex][4] == "Pickup":
                    chainCost += currentTime - R[requestIndex][0]
                #if request is dropoff, add time current_time - pickup time - Tod to chain cost
                elif R[requestIndex][4] == "Dropoff":
                    Tod = self.timeMatrix[R[requestIndex][1]][R[requestIndex][2]]
                    chainCost += currentTime - R[requestIndex][5] - Tod
            #reset current time
            currentTime = originalCurrentTime
            #add chain cost to list of chain costs
            chainCosts.append(chainCost)

        return sum(chainCosts)

    
    def value(self, state):
        return 0
    
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
