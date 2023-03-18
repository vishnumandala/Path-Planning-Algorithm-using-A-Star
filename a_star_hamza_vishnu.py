import numpy as np
import time
import cv2
import math
from queue import PriorityQueue

# Define the move functions
def move_p60(point, L):
    x = np.ceil(point[0] - L * math.cos(math.radians(60 + point[2])))
    y = np.ceil(point[1] - L * math.sin(math.radians(60 + point[2])))
    o = (point[2] - 60) % 360
    return (x, y, o), 1

def move_p30(point, L):
    x = np.ceil(point[0] - L * math.cos(math.radians(30 + point[2])))
    y = np.ceil(point[1] - L * math.sin(math.radians(30 + point[2])))
    o = (point[2] - 30) % 360
    return (x, y, o), 1

def move_forward(point, L):
    x = np.ceil(point[0] + L * math.cos(math.radians(point[2])))
    y = np.ceil(point[1] + L * math.sin(math.radians(point[2])))
    o = point[2]
    return (x, y, o), 1

def move_m30(point, L):
    x = np.ceil(point[0] + L * math.cos(math.radians(-30 + point[2])))
    y = np.ceil(point[1] + L * math.sin(math.radians(-30 + point[2])))
    o = (point[2] + 30) % 360
    return (x, y, o), 1

def move_m60(point, L):
    x = np.ceil(point[0] + L * math.cos(math.radians(-60 + point[2])))
    y = np.ceil(point[1] + L * math.sin(math.radians(-60 + point[2])))
    o = (point[2] + 60) % 360
    return (x, y, o), 1

map_width, map_height = 600, 250

def obstacles(clearance, radius):
    #Define the Obstacle Equations and Map Parameters
    eqns = {
        "Rectangle1": lambda x, y: 0 <= y <= 100 and 100 <= x <= 150,
        "Rectangle2": lambda x, y: 150 <= y <= 250 and 100 <= x <= 150,
        "Hexagon": lambda x, y: (75/2) * abs(x-300)/75 + 50 <= y <= 250 - (75/2) * abs(x-300)/75 - 50 and 235 <= x <= 365,
        "Triangle": lambda x, y: (200/100) * (x-460) + 25 <= y <= (-200/100) * (x-460) + 225 and 460 <= x <= 510
    }

    clearance = clearance + radius
    pixels = np.full((map_height, map_width, 3), 255, dtype=np.uint8)

    for i in range(map_height):
        for j in range(map_width):
            is_obstacle = any(eqn(j, i) for eqn in eqns.values())
            if is_obstacle:
                pixels[i, j] = [0, 0, 0]  # obstacle
            else:
                is_clearance = any(
                    eqn(x, y)
                    for eqn in eqns.values()
                    for y in range(i - clearance, i + clearance + 1)
                    for x in range(j - clearance, j + clearance + 1)
                    if (x - j)**2 + (y - i)**2 <= clearance**2
                )
                if i < clearance or i >= map_height - clearance or j < clearance or j >= map_width - clearance:
                    pixels[i, j] = [192, 192, 192]  # boundary
                elif is_clearance:
                    pixels[i, j] = [192, 192, 192]  # clearance
                else:
                    pixels[i, j] = [255, 255, 255]  # free space
    return pixels

# Define a function to check if current node is in range
def is_in_range(node):
    x, y, o = node
    y = map_height - y - 1
    return 0 <= x < map_width and 0 <= y < map_height and (pixels[int(y), int(x)] == [255, 255, 255]).all() and o%30 == 0

def is_valid_node(node, visited):
    if not is_in_range(node):
        return False  # out of range
    x, y, _ = node
    y = map_height - y - 1
    if not (pixels[int(y), int(x)] == [255, 255, 255]).all():
        return False  # in obstacle space
    # Check if the node is within threshold distance from any visited nodes
    threshold_x = 0.5
    threshold_y = 0.5
    threshold_theta = math.radians(30)
    for i in range(-1, 2):
        for j in range(-1, 2):
            for k in range(-1, 2):
                neighbor_node = (x + i * threshold_x, y + j * threshold_y, k * threshold_theta)
                if neighbor_node in visited:
                    return False  # too close to a visited node
    return True

# Define a function to check if current node is the goal node
def is_goal(current_node, goal_node):
    return np.sqrt((goal_node[0]-current_node[0])**2 + (goal_node[1]-current_node[1])**2) <= 1.5

# Define a function to find the optimal path
def backtrack_path(parents, start_node, goal_node):
    path, current_node = [goal_node], goal_node
    while current_node != start_node:
        path.append(current_node)
        current_node = parents[current_node]
    path.append(start_node)
    return path[::-1]

# Define a function to calculate the euclidean distance
def euclidean_distance(node1, node2):
    x1, y1, _ = node1
    x2, y2, _ = node2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Define the A* algorithm
def a_star(start_node, goal_node, display_animation=True):
    threshold = 0.5 # threshold distance
    rows = int(map_height / threshold)  # number of rows
    cols = int(map_width / threshold)   # number of columns
    angles = int(360 / 30)           # number of angles
    V = [[[False for _ in range(angles)] for _ in range(cols)] for _ in range(rows)]    # visited nodes matrix 

    open_list = PriorityQueue()
    closed_list = set()
    cost_to_come = {start_node: 0}
    cost_to_go = {start_node: euclidean_distance(start_node, goal_node)}
    cost = {start_node: cost_to_come[start_node] + cost_to_go[start_node]}
    parent = {start_node: None}
    open_list.put((cost[start_node], start_node))
    visited = set([start_node])
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('animation.mp4', fourcc, 15.0, (map_width, map_height))

    while not open_list.empty():
        _, current_node = open_list.get()
        closed_list.add(current_node)
        x, y, theta = current_node
        V[int(y / threshold)][int(x / threshold)][int(theta / 30)] = True   # Mark current node as visited
        out.write(pixels)
        if display_animation:
            cv2.imshow('Explored', pixels)
            cv2.waitKey(1)
        # Check if current node is the goal node
        if is_goal(current_node, goal_node):
            approx_goal_node = current_node # Approximate goal node (within threshold distance) 
            cost[goal_node] = cost [current_node]   # Cost of goal node
            path = backtrack_path(parent, start_node, approx_goal_node) # Backtrack the path
            if display_animation:
                for node in path:
                    x, y, _ = node
                    cv2.circle(pixels, (int(x), map_height - 1 - int(y)), 1, (0, 0, 255), thickness=-1)
                out.write(pixels)
                cv2.waitKey(0)
            print("Final Cost: ", cost[goal_node])
            out.release()
            cv2.destroyAllWindows()
            return path

        for move_func in [move_m30, move_m60, move_forward, move_p30, move_p60]:    # Iterate through all possible moves
            new_node, move_cost = move_func(current_node, L)
            if is_valid_node(new_node, visited):    # Check if the node is valid
                i, j, k = int(new_node[1] / threshold), int(new_node[0] / threshold), int(new_node[2] / 30) # Get the index of the node in the 3D array
                if not V[i][j][k]:  # Check if the node is in closed list
                    new_cost_to_come = cost_to_come[current_node] + move_cost
                    new_cost_to_go = euclidean_distance(new_node, goal_node)
                    new_cost = new_cost_to_come + new_cost_to_go    # Update cost
                    if new_node not in cost_to_come or new_cost_to_come < cost_to_come[new_node]:
                        cost_to_come[new_node] = new_cost_to_come   # Update cost to come
                        cost_to_go[new_node] = new_cost_to_go    # Update cost to go
                        cost[new_node] = new_cost   # Update cost
                        parent[new_node] = current_node  # Update parent
                        open_list.put((new_cost, new_node)) # Add to open list
                        visited.add(new_node)   # Add to visited list
                        # Draw vector from current_node to new_node
                        cv2.line(pixels, (int(current_node[0]), map_height - 1 - int(current_node[1])), (int(new_node[0]), map_height - 1 - int(new_node[1])), (255, 0, 0), thickness=1)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break

    out.release()
    cv2.destroyAllWindows()
    return None

# Get valid start and goal nodes from user input
while True:
    clearance = int(input("\nEnter the clearance: "))
    radius = int(input("Enter the radius: "))
    pixels = obstacles(clearance, radius)
    start_node = tuple(map(int, input("Enter the start node (in the format 'x y o'). O should be given in degrees and as a multiple of 30 i.e. {.., -60, -30, 0, 30, 60, ..}: ").split()))
    if not is_in_range(start_node):
        print("Error: Start node is in the obstacle space, clearance area, out of bounds or orientation was not given in the required format. Please input a valid node.")
        continue
    goal_node = tuple(map(int, input("Enter the goal node (in the format 'x y o'). O should be given in degrees and as a multiple of 30 i.e. {.., -60, -30, 0, 30, 60, ..}: ").split()))
    if not is_in_range(goal_node):
        print("Error: Goal node is in the obstacle space, clearance area, out of bounds or orientation was not given in the required format. Please input a valid node.")
        continue
    L = int(input("Enter the step size of the robot in the range 1-10: "))
    if L < 1 or L > 10:
        print("Error: Step size should be in the range 1-10. Please input a valid step size.")
        continue
    break

# Run A* algorithm
start_time = time.time()
path = a_star(start_node, goal_node)
if path is None:
    print("\nError: No path found.")
else:
    print("\nGoal Node Reached!\nShortest Path: ", path, "\n")
end_time = time.time()
print("Runtime:", end_time - start_time, "seconds\n")