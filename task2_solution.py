# Purpose: Solution class for fleet problem

# Imports
import sys
import numpy as np

import search

# TODO: Check guidelines and conventions that the teacher wants us to follow

# TODO: Put counters inside for loops to remove the use of the index() method, which is expensive

# TODO: Replace list and tuple member access with getters and setters, that is replace request[0] with request.getTime() for example

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
    
    def solve(self):
        # path = search.uniform_cost_search(self, display=True).solution()
        path = search.uniform_cost_search(self).solution()
   
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
            sys.exit(1)

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
    
""" class MyNode(search.Node):
    #override __eq__ method to compare two states
    def __eq__(self, other):
        for i in range(len(self.state[0])):
            if(self.state[0][i][0] != other.state[0][i][0]):
                return False
            if(self.state[0][i][1] != other.state[0][i][1]):
                return False
            if(self.state[0][i][2] != other.state[0][i][2]):
                return False
            # if(self.state[0][i][3] != other.state[0][i][3]):
            #     return False
        for i in range(len(self.state[1])):
            if(self.state[1][i][0] != other.state[1][i][0]):
                return False
            if(self.state[1][i][1] != other.state[1][i][1]):
                return False
            if(self.state[1][i][2] != other.state[1][i][2]):
                return False
            if(self.state[1][i][3] != other.state[1][i][3]):
                return False
            if(self.state[1][i][4] != other.state[1][i][4]):
                return False
            if(self.state[1][i][5] != other.state[1][i][5]):
                return False        
        return True
    
search.Node = MyNode """
