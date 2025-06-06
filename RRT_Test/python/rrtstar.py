import numpy as np
import cv2
import heapq
import time
from math import dist

WHEEL_RADIUS = 33#/1000 # 33mm
ROBOT_RADIUS = 220#/1000 # 220mm
WHEEL_DISTANCE = 287#/1000 # 287mm
clearance = 5
clearance += ROBOT_RADIUS

width = 6000
height = 2000
scale = 5

clearance_color = (0, 255, 255)
obstacle_color = (0, 0, 0)
robot_radius_color = (254, 105, 180)

# Create a black canvas
canvas = np.zeros((height, width, 3), dtype="uint8")
# Create a white rectangle
canvas = cv2.rectangle(canvas, (clearance, clearance), (width-clearance, height-clearance), (255, 255, 255), -1)

# OBSTACLE 1
x1, x2 = 1500, 1750
y1, y2 = 0, 1000
# Draw the clearance
for i in range(x1-clearance, x2+clearance):
    for j in range(y1+clearance, y2+clearance):
        canvas[j, i] = clearance_color
# Draw the obstacle
for i in range(x1, x2):
    for j in range(y1, y2):
        canvas[j, i] = obstacle_color

# OBSTACLE 2
x1, x2 = 2500, 2750
y1, y2 = height-1000, height
# Draw the clearance
for i in range(x1-clearance, x2+clearance):
    for j in range(y1-clearance, y2-clearance+1):
        canvas[j, i] = clearance_color
# Draw the obstacle
for i in range(x1, x2):
    for j in range(y1, y2):
        canvas[j, i] = obstacle_color

# OBSTACLE 3
# Draw a circle at (4200, 800)
center = (4200, 800)
radius = 600
# Draw the clearance
canvas = cv2.circle(canvas, center, 600+clearance, clearance_color, -1)
# Draw the obstacle
canvas = cv2.circle(canvas, center, 600, obstacle_color, -1)
        
# x_start, y_start = clearance+1, clearance+1
# x_goal, y_goal = width-clearance-1, clearance+1

# Draw a circle at x_start, y_start
# canvas = cv2.circle(canvas, (x_start, y_start), 5, (0, 255, 0), -1)
# Draw a circle at x_goal, y_goal
# canvas = cv2.circle(canvas, (x_goal, y_goal), 5, (0, 0, 255), -1)


# Draw a red line at x = width/2
# canvas = cv2.line(canvas, (int(width/2.5), 0), (int(width/2.5), height), (0, 0, 255), 10)
# Resize the canvas by a factor of scale
width_resized = int(width/scale)
height_resized = int(height/scale)
canvas_resized = cv2.resize(canvas, (width_resized, height_resized))
# cv2.imshow("Canvas", canvas_resized)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

def obstacles():
    width = 6000
    height = 2000
    scale = 5

    clearance_color = (0, 255, 255)
    obstacle_color = (0, 0, 0)

    # Create a black canvas
    canvas = np.zeros((height, width, 3), dtype="uint8")
    # Create a white rectangle
    canvas = cv2.rectangle(canvas, (clearance, clearance), (width-clearance, height-clearance), (255, 255, 255), -1)

    # OBSTACLE 1
    x1, x2 = 1500, 1750
    y1, y2 = 0, 1000
    # Draw the clearance
    for i in range(x1-clearance, x2+clearance):
        for j in range(y1+clearance, y2+clearance):
            canvas[j, i] = clearance_color
    # Draw the obstacle
    for i in range(x1, x2):
        for j in range(y1, y2):
            canvas[j, i] = obstacle_color

    # OBSTACLE 2
    x1, x2 = 2500, 2750
    y1, y2 = height-1000, height
    # Draw the clearance
    for i in range(x1-clearance, x2+clearance):
        for j in range(y1-clearance, y2-clearance+1):
            canvas[j, i] = clearance_color
    # Draw the obstacle
    for i in range(x1, x2):
        for j in range(y1, y2):
            canvas[j, i] = obstacle_color

    # OBSTACLE 3
    # Draw a circle at (4200, 800)
    center = (4200, 800)
    radius = 600
    # Draw the clearance
    canvas = cv2.circle(canvas, center, 600+clearance, clearance_color, -1)
    # Draw the obstacle
    canvas = cv2.circle(canvas, center, 600, obstacle_color, -1)
            
    # x_center, y_center = 4200, 800
    # radius = 600
    # Draw the clearance
    # for i in range(x_center-radius-clearance, x_center+radius+clearance):
    #     for j in range(y_center-radius-clearance, y_center+radius+clearance):
    #         if (i-x_center)**2 + (j-y_center)**2 <= (radius+clearance)**2 and canvas[j, i, 0] != 0:
    #             canvas[j, i] = clearance_color
    # # Draw the obstacle
    # for i in range(x_center-radius, x_center+radius):
    #     for j in range(y_center-radius, y_center+radius):
    #         if (i-x_center)**2 + (j-y_center)**2 <= radius**2:
    #             canvas[j, i] = obstacle_color

    # x_start, y_start = clearance+1, clearance+1
    # x_goal, y_goal = width-clearance-1, clearance+1

    # Draw a circle at x_start, y_start
    # canvas = cv2.circle(canvas, (x_start, y_start), 5, (0, 255, 0), -1)
    # Draw a circle at x_goal, y_goal
    # canvas = cv2.circle(canvas, (x_goal, y_goal), 5, (0, 0, 255), -1)


    # Draw a red line at x = width/2
    # canvas = cv2.line(canvas, (int(width/2.5), 0), (int(width/2.5), height), (0, 0, 255), 10)
    # Resize the canvas by a factor of scale
    width_resized = int(width/scale)
    height_resized = int(height/scale)
    canvas_resized = cv2.resize(canvas, (width_resized, height_resized))
    # cv2.imshow("Canvas", canvas_resized)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return canvas


canvas = obstacles()

x_start, y_start, theta_start = clearance+1, clearance+1, 0
x_goal, y_goal = width-clearance-1, clearance+1

step_size = 100
distance_threshold = 100
search_radius = 300

# Make a lambda function to adjust the value of x to the visited space
adjust = lambda x, threshold: int(int(round(x*2)/2)/threshold)

# Dictionary to store visited nodes
visited = {(adjust(x_start, distance_threshold),
            adjust(y_start, distance_threshold)): 1}

# Dictionary to store the parent of each node
parent = {(x_start, y_start): (x_start, y_start)}

# Dictionary to store the children of each node
children = {(x_start, y_start): []}

# Dictionary to store the cost of each node
cost = {(x_start, y_start): 0}

# Function to check if the node is valid
def valid_node(x, y):
    # Adjust the value for visited nodes
    x_vis = adjust(x, distance_threshold)
    y_vis = adjust(y, distance_threshold)
    # Adjust the values for canvas
    x_cvs = int(round(x*2)/2)
    y_cvs = int(round(y*2)/2)

    # Check if the node is not in the obstacle and not visited
    if canvas[y_cvs, x_cvs, 0] != 0 and (x_vis, y_vis) not in visited:
        return True
    return False

# Function to check if the child node and parent node are not intersecting with the obstacle
def valid_edge(x_parent, y_parent, x_child, y_child):
    # Sample 3 points on the line joining the parent and child nodes, 
    # not including the parent and child nodes
    x_intermediate = np.linspace(x_parent, x_child, 5)[1:-1]
    y_intermediate = np.linspace(y_parent, y_child, 5)[1:-1]

    # Adjust the values for canvas
    x_intermediate = [int(round(x*2)/2) for x in x_intermediate]
    y_intermediate = [int(round(y*2)/2) for y in y_intermediate]

    # Check if any of the intermediate points are in the obstacle
    for x, y in zip(x_intermediate, y_intermediate):
        if canvas[y, x, 0] == 0:
            return False
    return True


reached = False
iterations = 0
c_best = float('inf')
start = time.time()

while iterations < 100:
    
    # Get a new node
    x_node = np.random.randint(clearance, width-clearance)
    y_node = np.random.randint(clearance, height-clearance)    

    # If the node is valid
    if valid_node(x_node, y_node):

        # Find the nearsest node in the tree to get the projected node
        min_dist = float('inf')

        for x, y in parent:
            dist = np.sqrt((x-x_node)**2 + (y-y_node)**2)
            if dist < min_dist:
                min_dist = dist
                x_parent, y_parent = x, y

        # Get the angle of the new node
        theta = np.arctan2(y_node-y_parent, x_node-x_parent)

        # Calculate the new node
        x_new = int(x_parent + step_size*np.cos(theta))
        y_new = int(y_parent + step_size*np.sin(theta))
        
        if reached:
            iterations += 1

        # Check if the new node is valid
        if valid_node(x_new, y_new):

            # Collect all the nodes in the search radius
            neighbours = []
            for x, y in parent:
                if np.sqrt((x-x_new)**2 + (y-y_new)**2) < search_radius:
                    neighbours.append((x, y))

            # Find the node with the minimum cost to get new node
            min_cost = cost[(x_parent, y_parent)] + step_size
            for x, y in neighbours:
                new_cost = cost[(x, y)] + np.sqrt((x-x_new)**2 + (y-y_new)**2)
                if new_cost < min_cost:
                    min_cost = new_cost
                    x_parent, y_parent = x, y

            # Check if the edge between the parent and child nodes is valid
            if not valid_edge(x_parent, y_parent, x_new, y_new):
                continue
            
            # Add the cost of the new node
            cost[(x_new, y_new)] = min_cost

            # Check if rewiring to the newly added node will reduce the cost of the neighbours
            for x, y in neighbours:
                new_cost = cost[(x_new, y_new)] + np.sqrt((x-x_new)**2 + (y-y_new)**2)
                if new_cost < cost[(x, y)]:

                    # Check if the edge between the parent and child nodes is valid
                    if not valid_edge(x, y, x_new, y_new):
                        continue

                    previous_cost = cost[(x, y)]
                    cost[(x, y)] = new_cost
                    
                    # Remove this node from the children of its current parent
                    x_parent_neighbour, y_parent_neighbour = parent[(x, y)]
                    children[(x_parent_neighbour, y_parent_neighbour)].remove((x, y))

                    # Add the new node to the parent dictionary
                    parent[(x, y)] = (x_new, y_new)

                    # Add the new node to the children dictionary
                    children.setdefault((x_new, y_new), []).append((x, y))
                    
                    # If the cost of this node is reduced, we must update the cost of its children,
                    # and their children, and so on
                    cost_reduction = previous_cost - new_cost
                    queue = []
                    heapq.heappush(queue, ((x, y)))
                    while queue:
                        x_, y_ = heapq.heappop(queue)
                        # Check if the node has children
                        if (x_, y_) in children:
                            for child in children[(x_, y_)]:
                                    cost[child] -= cost_reduction
                                    heapq.heappush(queue, child)
                                    
            x_achieved, y_achieved = x_new, y_new

            x_adjusted = adjust(x_new, distance_threshold)
            y_adjusted = adjust(y_new, distance_threshold)
            
            # Add the new node to the visited nodes
            visited[(x_adjusted, y_adjusted)] = 1

            # Add the new node to the parent dictionary
            parent[(x_new, y_new)] = (x_parent, y_parent)

            # Add the new node to the children dictionary
            children.setdefault((x_parent, y_parent), []).append((x_new, y_new))

            if not reached:
                # Check if the new node is close to the goal
                if np.sqrt((x_new-x_goal)**2 + (y_new-y_goal)**2) < distance_threshold:
                # if np.sqrt((x_new-x_goal)**2 + (y_new-y_goal)**2) < 50:
                    end = time.time()
                    print("Goal reached: ", end-start, "seconds")

                    # Add the final node as a child of the current node to complete the path
                    children.setdefault((x_new, y_new), []).append((x_goal, y_goal))

                    # Add the final node to the parent dictionary
                    parent[(x_goal, y_goal)] = (x_new, y_new)

                    # Save the final node
                    x_final, y_final = x_goal, y_goal

                    # Add the cost of the final node
                    cost[(x_goal, y_goal)] = cost[(x_new, y_new)] + np.sqrt((x_goal-x_new)**2 + (y_goal-y_new)**2)

                    print("Current Cost: ", cost[(x_goal, y_goal)])

                    # Set the reached flag to True
                    reached = True
        
print("Final cost: ", cost[(x_final, y_final)]) 

# Get the path from the parent dictionary
path = []
# x, y = x_goal, y_goal   
# x, y = x_achieved, y_achieved
x, y = x_final, y_final
while (x, y) != (x_start, y_start):
    # print(x, y)
    path.append((x, y))
    x, y = parent[(x, y)]
path.append((x, y))
path.reverse()

# Calculate the distance between every point in the path
sum = 0
for i in range(len(path)-1):
    # print(path[i])
    sum += np.sqrt((path[i][0]-path[i+1][0])**2 + (path[i][1]-path[i+1][1])**2)

print("Total distance: ", sum)
print("Difference in cost: ", np.round(cost[(x_final, y_final)] - sum, 2))

canvas = obstacles()

# Plot the start and goal nodes
canvas = cv2.circle(canvas, (x_start, y_start), 5, (0, 0, 254), 10) 
canvas = cv2.circle(canvas, (x_goal, y_goal), 5, (0, 0, 254), 10)

# Plot a line between the parent and child nodes
for x, y in parent:
    x_parent, y_parent = parent[(x, y)]
    canvas = cv2.line(canvas, (x, y), (x_parent, y_parent), (0, 255, 0), 5)
    # Draw a purple circle at the parent node
    canvas = cv2.circle(canvas, (x_parent, y_parent), 5, (255, 0, 255), 3)

# Plot a circle at all the child points 
for x, y in children:
    for child in children[(x, y)]:
        canvas = cv2.circle(canvas, child, 5, (255, 0, 255), 3)

# Plot the path
for i in range(len(path)-1):
    x1, y1 = path[i]
    x2, y2 = path[i+1]
    canvas = cv2.line(canvas, (x1, y1), (x2, y2), (0, 0, 255), 5)

# Resize the canvas by a factor of scale
canvas_resized = cv2.resize(canvas, (width_resized, height_resized))

cv2.imshow("Canvas", canvas_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()