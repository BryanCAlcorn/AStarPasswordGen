####################################################################################################
#A* Password Grid Generator:
#Generates a square grid of random letters, numbers, and symbols of a given size, then finds an
#optimal path to traverse the grid that results in a password.
#Uses: Print out the generated grid and memorize the generated path. Gives you a strong password
#that you can safely have written down, since you are the only one who knows the correct path.
####################################################################################################
import random;
import math;

debug = False;

#Enum representation to allow switching allowed symbols
def enum(**enums):
    return type('Enum',(), enums);
AllowedSymbols = enum(ALL=1,NOSPECIAL=2);

#Used in A* implementation to determine password complexity
lowercase = "abcdefghijklmnopqrstuvwxyz";
uppercase = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
numbers   = "0123456789";
symbols   = "!@#$%^&*()_+-=[]{};:<>./?\\|";

#Generates the grid, checks neighbors for dupes, prints it out, and finds an optimal password path
def passwordGrid(size, allowedSymbols):
    #Checks usable Symbols and builds them
    allSymbols = lowercase + uppercase + numbers;
    if(allowedSymbols != 2):
        allSymbols += symbols;
    
    #Generates the grid and checks neighbors
    grid = generateGrid(size, allSymbols);
    for i in range(size):
        for j in range(size):
            checkNeighbors(grid, i, j, allSymbols);
    
    printGrid(grid);
    
    #Generates the start, goal and PW
    points = genStartAndGoal(size);
    print(points);
    path = AStar(points[0],points[1],grid);
    print(path);
    return grid, points, path;

#Prints a pre-genned grid, and finds an optimal password path. Useful for generating a new path with the same grid
def passwordGridNoGen(grid):
    printGrid(grid);
    
    #Generates the start, goal and PW
    points = genStartAndGoal(len(grid));
    print(points);
    path = AStar(points[0],points[1],grid);
    print(path);
    return points, path;

#Generates a random square grid of allSymbols
def generateGrid(size, allSymbols):
    grid = [];
    for i in range(size):
        grid.append([]);
        for j in range(size):
            grid[i].append(random.choice(allSymbols));
    return grid;

#Checks a point in the grid for duplicate neighbors,
#   to avoid repeating characters. Recursively backtracks on changed characters
#   to check them again
def checkNeighbors(grid, x, y, allSymbols):
    for i in range(x - 1, x + 2):
        for j in range(y - 1, y + 2):
            #Avoid checking the point against itself
            if(i == x and j == y):
                continue;
            #Avoid out of bounds errors
            elif(i < 0 or j < 0 or i >= len(grid) or j >= len(grid)):
                continue;
            elif(grid[x][y] == grid[i][j]):
                if debug:
                    print("Neighbor Changed: x:" + str(i) + " y:" + str(j));
                grid[i][j] = random.choice(allSymbols);
                while(grid[x][y] == grid[i][j]):
                    grid[i][j] = random.choice(allSymbols);
                checkNeighbors(grid, i, j, allSymbols);
    return grid;

#Prints the grid into human-readable format
def printGrid(grid):
    for i in range(len(grid)):
        y = "| ";
        for j in range(len(grid)):
            y += ("" if j == 0 else " ") + grid[i][j] + " |";
        print(y);

#Test Method for AStar, will change if parameters are changed for distance measurements
def testAStar():
    grid = [["a","b","c","d"],
            ["1","2","3","4"],
            ["A","B","C","D"],
            ["!","@","#","$"]];
    expectedPath = ["a","2","B","#","$"];
    path = AStar((0,0),(3,3),grid);
    assert path == expectedPath;
    print("Test Succeeded, Generated path: " + str(path));

#A* Algorithm to search grid for an optimal path between start and goal, generates a password
def AStar(start, goal, grid):
    closedSet = [];
    openSet = [start];
    came_from = {};
    complexity = [0,0,0,0];
    #Initialize A* with start values
    g_score = {start : 0}
    f_score = {start : g_score[start] + cost_estimate(start, goal, grid, complexity)};
    if(debug):
        print("Starting f_score: " + str(f_score[start]));
    prevNode = ();
    while len(openSet) > 0:
        #gets the node with minimum score
        current = min(f_score, key=f_score.get);
        
        if(current == prevNode):
            #prevents inf. loop if the current node has the lowest score in f_score
            f_score.pop(current);
            continue;
        prevNode = current;
        
        if(debug):
            print("Current node: " + str(current[0]) + "," + str(current[1]));
            print("\tfS: " + str(f_score[current]));
        
        if(current == goal):
            if(debug):
                print("Came From: " + str(came_from));
            return reconstruct_path(came_from, goal, grid, []);

        if(current in openSet):
            openSet.remove(current);
        closedSet.append(current);

        #Check all possible adjascent nodes
        for i in range(current[0] - 1, current[0] + 2):
            for j in range(current[1] - 1, current[1] + 2):
                #Avoid checking the point against itself
                if(i == current[0] and j == current[1]):
                    continue;
                #Avoid out of bounds errors
                elif(i < 0 or j < 0 or i >= len(grid) or j >= len(grid)):
                    continue;
                
                neighbor = (i, j)
                tentative_g_score = g_score[current] + dist_between(current, neighbor, came_from, grid, complexity);
                if(debug):
                    print("\tNeighbor: " + str(i) + "," + str(j) + "\n\t\ttgS: " + str(tentative_g_score));
                #current neighbor has been visited and is farther away from goal
                if(neighbor in closedSet and tentative_g_score >= g_score[neighbor]):
                    if(debug):
                        print("\t\tskip");
                    continue;
                #current neighbor hasn't been visited or is closer to the goal
                if(neighbor not in closedSet or tentative_g_score < g_score[neighbor]):
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = tentative_g_score + cost_estimate(neighbor, goal, grid, complexity);
                    if(debug):
                        print("\t\tfS: " + str(f_score[neighbor]));
                    if(neighbor not in openSet):
                        if(debug):
                            print("\t\tneighbor opened");
                        openSet.append(neighbor);
    #returns if no path can be found. Shouldn't really happen...
    return False;
        
#Estimated cost to get to the goal from node
def cost_estimate(node, goal, grid, complexity):
    estimate = pythag(node, goal);

    #reduce estimate if complexity is higher, encourages complex passwords
    totalComplexity = 0;
    for i in range(len(complexity)):
        if(complexity[i] > 0):
            totalComplexity += 1;
    estimate -= totalComplexity;

    return estimate;
    
#distance between node1 and node2
def dist_between(node1, node2, came_from, grid, complexity):
    cost = 1;
    #increase cost:
    if(len(came_from) > len(grid)):
        #If path gets too long
        cost += 1;
    if(len(came_from) < len(grid)):
        #if path is too short
        cost += 2;
    if((node1[0] != node2[0] or node1[0] != node2[1]) or
        (node1[1] != node2[0] or node1[1] != node2[1])):
        #if path turns
        cost += 4;

    #decrease cost:
    #if the complexity of the pw increases between node1 and node2, reduce cost
    fC = 2;
    lC = 1;
    if(grid[node2[0]][node2[1]] in lowercase):
        cost -= fC if complexity[0] == 1 else lC
        complexity[0] += 1;
    elif(grid[node2[0]][node2[1]] in uppercase):
        cost -= fC if complexity[1] == 1 else lC
        complexity[1] += 1;
    elif(grid[node2[0]][node2[1]] in numbers):
        cost -= fC if complexity[2] == 1 else lC
        complexity[2] += 1;
    elif(grid[node2[0]][node2[1]] in symbols):
        cost -= fC if complexity[3] == 1 else lC
        complexity[3] += 1;

    return cost;

#Builds the password from the path that A* generated
def reconstruct_path(came_from, current, grid, passw):
    while(current in came_from):
        passw.append(grid[current[0]][current[1]]);
        current = came_from[current];
    passw.append(grid[current[0]][current[1]]);
    passw.reverse();
    return passw;

#pythagorean theorem to find distance between two nodes
def pythag(node1, node2):
    a = node1[0] - node2[0];
    b = node1[1] - node2[1];
    c = pow(a, 2) + pow(b, 2);
    return round(math.sqrt(c), 2);

#Generates start and goal nodes based on size of the grid
def genStartAndGoal(size):
    sides = [1,2,3,4];
    side1 = random.choice(sides);
    sides.remove(side1);
    side2 = random.choice(sides);

    start = genSidePoint(side1, size);
    goal = genSidePoint(side2, size);

    #If start and goal are too close, generate a new goal
    while(pythag(start, goal) <= (size / 2)):
        if(debug):
            print("Generating new goal. Original was too close");
        goal = genSidePoint(random.choice(sides), size);

    return (start,goal);

#Gets a point on the side of the grid
def genSidePoint(side, size):
    if(side == 1):
        x = 0;
        y = random.randint(0, size - 1);
    elif(side == 2):
        x = size - 1;
        y = random.randint(0, size - 1);
    elif(side == 3):
        x = random.randint(0, size - 1);
        y = 0;
    elif(side == 4):
        x = random.randint(0, size - 1);
        y = size - 1;
    return (x, y);
