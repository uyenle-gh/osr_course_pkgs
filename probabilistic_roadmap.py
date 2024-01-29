import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('osr_examples/scripts/')
import environment_2d
import heapq
import argparse

def sample_free_points(num_samples: int, size_x: int, size_y: int, env):
  """
  Generates a specified number of random points within a given 2D environment size. 
  Only points that do not collide with obstacles in the environment are included.

  Parameters:
  	num_samples (int): The number of random points to generate.
  	size_x (int): The width of the environment in which to generate points.
  	size_y (int): The height of the environment in which to generate points.
  	env: The environment object with a method check_collision(x, y) 
  				that determines if a point is in a free space.

  Returns:
  	list: A list of tuples, where each tuple represents the coordinates (x, y) 
          of a point that is free of collisions in the environment.
  """
  sample_points = []
  while len(sample_points) < num_samples:
    x = np.random.rand() * size_x
    y = np.random.rand() * size_y
    if not env.check_collision(x,y):
      sample_points.append((x,y))
  return sample_points

def compute_euclid_distance(point1: tuple, point2: tuple):
  """
  Calculates the Euclidean distance between two points in 2D space.

  Parameters:
    point1 (tuple): The first point, represented as a tuple (x1, y1).
    point2 (tuple): The second point, represented as a tuple (x2, y2).

  Returns:
    float: The Euclidean distance between point1 and point2.
  """
  return np.linalg.norm(np.array(point1) - np.array(point2))

def construct_init_roadmap(sample_points: list, neighbor_radius: float, env):
  """
  Constructs an initial roadmap by connecting the sample points that are 
  within a specified neighbor radius from each other, provided there is no 
  collision along the edge as determined by the environment's 
  check_edge_collision method.

  Parameters:
    points (list of tuples): A list of points where each point is represented as a tuple (x, y) 
    neighbor_radius (float): The maximum distance between two points for them to be 
                             considered neighbors and connected by an edge.
    env: The environment object with a method check_edge_collision(edge) that 
         determines if an edge is free of collisions.

  Returns:
    dict: A dictionary representing the constructed roadmap as adjacency lists.
  """
  roadmap = {}

  for i, point1 in enumerate(sample_points):
    roadmap[i] = []
    for j, point2 in enumerate(sample_points):
      if i != j:
        if compute_euclid_distance(point1, point2) <= neighbor_radius:
          edge = (point1, point2)
          if not env.check_edge_collision(edge):
            roadmap[i].append(j)
  return roadmap

def add_start_goal_to_roadmap(sample_points: list, roadmap: dict, neighbor_radius: float,
                              start_point: tuple, goal_point: tuple, env):
  """
  Adds start and goal points to an existing roadmap and connects them to nearby points.

  Parameters:
    points (list of tuples): The list of existing points in the roadmap.
    roadmap (dict): The existing roadmap to be updated, represented as adjacency lists.
    neighbor_radius (float): The maximum distance between points for them to be considered neighbors.
    start_point (tuple): The starting point of the path, represented as a tuple (x, y).
    goal_point (tuple): The goal point of the path, represented as a tuple (x, y).
    env: The environment object with a method check_edge_collision(edge) that determines 
         if an edge is free of collisions.

  Returns:
    dict: The updated roadmap including the start and goal points.
  """
  start_index = len(sample_points)  # index for the start point
  goal_index = start_index + 1  # index for the goal point

  # Add start and goal points to the roadmap
  for i, point in enumerate([start_point, goal_point]):
    current_index = start_index if i == 0 else goal_index
    roadmap[current_index] = []

    for j, other_point in enumerate(sample_points):
      if compute_euclid_distance(point, other_point) <= neighbor_radius and \
        not env.check_edge_collision((point, other_point)):
        roadmap[current_index].append(j)  # Connect to other points
        roadmap[j].append(current_index)

  return roadmap, start_index, goal_index


def dijkstra(roadmap, start_idx, goal_idx):
  """
  Find the shortest path from a start node to a goal node in a graph using Dijkstra's algorithm.

  Parameters:
    roadmap (dict): The graph represented as a dictionary. 
    start_idx (int): Index of the start node in the graph.
    goal_idx (int): Index of the goal node in the graph.

  Returns:
    list: A list of node indices representing the shortest path from the start node to the goal node.
          If no path is found, returns an empty list.
  """
  # Priority queue to hold the nodes to explore
  queue = [(0, start_idx)]
  heapq.heapify(queue)

  # Dictionary to store the cost to reach each node
  distances = {node: float('infinity') for node in roadmap}
  distances[start_idx] = 0

  # Dictionary to store the path
  previous_nodes = {node: None for node in roadmap}

  while queue:
    current_distance, current_node = heapq.heappop(queue)
    # Explore neighbors
    for neighbor in roadmap[current_node]:
      distance = current_distance + compute_euclid_distance(current_node, neighbor)
      if distance < distances[neighbor]:
        distances[neighbor] = distance
        previous_nodes[neighbor] = current_node
        heapq.heappush(queue, (distance, neighbor))
        
  # Reconstruct the path from start to goal
  path = []
  current_node = goal_idx
  while current_node is not None:
    path.append(current_node)
    current_node = previous_nodes[current_node]
  path.reverse()
  return path


def Astar(roadmap, start_idx, goal_idx, sample_points):
  """
  Find the shortest path from a start node to a goal node in a graph using the A* algorithm.

  Parameters:
    roadmap (dict): The graph represented as a dictionary. 
    start_idx (int): Index of the start node in the graph.
    goal_idx (int): Index of the goal node in the graph.

  Returns:
    list: A list of node indices representing the shortest path from the start node to the goal node.
          If no path is found, returns an empty list.
  """
  queue = [(0 + compute_euclid_distance(sample_points[start_idx], sample_points[goal_idx]), 0, start_idx)]
  heapq.heapify(queue)

  distances = {node: float('infinity') for node in roadmap}
  distances[start_idx] = 0

  previous_nodes = {node: None for node in roadmap}

  while queue:
    _, current_distance, current_node = heapq.heappop(queue)

    if current_node == goal_idx:
      break

    for neighbor in roadmap[current_node]:
      tentative_distance = current_distance + compute_euclid_distance(sample_points[current_node], sample_points[neighbor])
      if tentative_distance < distances[neighbor]:
        distances[neighbor] = tentative_distance
        priority = tentative_distance + compute_euclid_distance(sample_points[neighbor], sample_points[goal_idx])
        heapq.heappush(queue, (priority, tentative_distance, neighbor))
        previous_nodes[neighbor] = current_node

  if distances[goal_idx] == float('infinity'):
    return []
  
  # Reconstruct the path from start to goal
  path = []
  current_node = goal_idx
  while current_node is not None:
    path.append(current_node)
    current_node = previous_nodes[current_node]
  path.reverse()
  return path


def probabilistic_roadmap(size_x: float, size_y: float,
                          start_point: tuple, goal_point: tuple,
                          num_samples: int, neighbor_radius: int, 
                          env, path_finder):
  """
  Creates a probabilistic roadmap for pathfinding in a 2D environment and finds a path from start to goal.

  This function first checks for a direct collision-free path between the start and goal points. If such
  a path exists, it is returned. Otherwise, the function samples free points in the environment, constructs
  a navigable roadmap by connecting these points within a specified radius, and then finds the shortest 
  path from start to goal using the A* or Dijkstra's algorithm.

  Parameters:
    size_x (float): The width of the environment.
    size_y (float): The height of the environment.
    start_point (tuple): The starting point (x, y).
    goal_point (tuple): The goal point (x, y).
    num_samples (int): The number of points to sample in the environment.
    neighbor_radius (int): The radius within which to connect points in the roadmap.
    env: The environment object with methods for collision checking and plotting.
    path_finder: The path finding algorithm (A* or Dijkstra's)

  Returns:
    list: A list of tuples representing the coordinates of the points in the found path, 
    			or an empty list if no path is found.
  """
  # Check if there is a direct path from start to goal
  if not env.check_edge_collision((start_point, goal_point)):
    env.plot_edge(start_point, goal_point)
    return [start_point, goal_point]
  
  # Sample free points
  print("Step 1: Free point sampling")
  sample_points = sample_free_points(num_samples, size_x, size_y, env)

  # Create a navigable roadmap by connecting the sample points within neighbor_radius
  print("Step 2: Roadmap construction")
  init_roadmap = construct_init_roadmap(sample_points, neighbor_radius, env)

  # Add start and goal points to the roadmap
  updated_roadmap, start_idx, goal_idx = add_start_goal_to_roadmap(
    sample_points, init_roadmap, neighbor_radius, start_point, goal_point, env)
  sample_points += [start_point, goal_point]

  # Find path based on the roadmap found
  print("Step 3: Path finding")
  path_in_indices = path_finder(updated_roadmap, start_idx, goal_idx, sample_points)
  if len(path_in_indices) <= 1:
    print("No paths exist in current roadmap.")
    return []
  
	# Convert path indices to coordinates
  path_in_coordinates = [sample_points[i] for i in path_in_indices]

  # Plot path found
  for i in range(len(path_in_coordinates)-1):
    env.plot_edge(path_in_coordinates[i], path_in_coordinates[i+1], "black")
  plt.savefig('img/path.png')
  
  return path_in_coordinates

def path_shortcutting(path: list, maxrep: int, env):
  """
  Performs post-processing on a given path to potentially shorten it by removing unnecessary waypoints.

  This function iteratively selects pairs of points along the path and checks if a direct path between 
  these points is collision-free. If it is, the intermediate points between these two points are removed, 
  thus shortening the path. 
  
  Parameters:
    path (list of tuples): The original path represented as a list of points (tuples) in 2D space.
    maxrep (int): The maximum number of iterations for attempting to shorten the path.
    env: The environment object with a method check_edge_collision(edge) that determines 
         if an edge (represented as a tuple of two points) is free of collisions.

  Returns:
    list: The potentially shortened path as a list of tuples representing the points.
  """
  prev_length = len(path)
  for i in range(maxrep):
    t1, t2 = np.random.choice(len(path), size=2, replace=False)
    if t1 > t2:
      t1, t2 = t2, t1
    edge = (path[t1], path[t2])
    if not env.check_edge_collision(edge):
      del path[t1+1:t2]

  if len(path) <= prev_length:
    print("---Shorter path found.")
    for i in range(len(path)-1):
      env.plot_edge(path[i], path[i+1])
  else:
    print("---No potentially shorter paths.")
  return path

def main():
  """
  Main function to execute the Probabilistic Roadmap (PRM) path planning algorithm.

  This function initializes the environment for path planning, sets up the problem parameters, and
  executes the PRM algorithm to find a path from a randomly chosen start point to a goal point.

  Parse Arguments:
    seed (int): Seed for the random number generator to ensure reproducibility.
    size_x (int): Width of the environment.
    size_y (int): Height of the environment.
    num_obs (int): Number of obstacles in the environment.
    num_samples (int): Number of points to sample in the PRM.
    neighbor_radius (int): Radius to connect points in the PRM.
    max_rep (int): Maximum number of iterations for path shortening in post-processing.

  Output:
    list: A list of tuples representing the points taken to reach goal from start.
  """
  # Create the parser
  parser = argparse.ArgumentParser(description="Run Probabilistic Roadmap (PRM) Path Planning")

  # Add arguments
  parser.add_argument("--seed", type=int, default=4, help="Seed for random number generator")
  parser.add_argument("--size_x", type=int, help="Width of the environment")
  parser.add_argument("--size_y", type=int, help="Height of the environment")
  parser.add_argument("--num_obs", type=int, help="Number of obstacles")
  parser.add_argument("--num_samples", type=int, help="Number of samples for PRM")
  parser.add_argument("--neighbor_radius", type=float, help="Neighbor radius for PRM")
  parser.add_argument("--max_rep", type=int, default=100, help="Max iterations for path shortening")
  parser.add_argument("--path_finder", type=str, choices=['Astar', 'dijkstra'], default='Astar',
                        help="Pathfinding algorithm to use")

  # Parse arguments
  args = parser.parse_args()

  # Retrieve the pathfinding function based on the command-line argument
  path_finder_functions = {
    'Astar': Astar,
    'dijkstra': dijkstra
  }
  path_finder = path_finder_functions[args.path_finder]

  # Create environment
  np.random.seed(args.seed)
  env = environment_2d.Environment(args.size_x, args.size_y, args.num_obs)
  plt.clf()
  env.plot()

  # Generate random start and end points
  q = env.random_query()
  if not q:
    print("No query points generated.")
    return
  x_start, y_start, x_goal, y_goal = q
  env.plot_query(x_start, y_start, x_goal, y_goal)
  start_point = (x_start, y_start)
  goal_point = (x_goal, y_goal)
  
  # Execute PRM on environment
  path = probabilistic_roadmap(args.size_x, args.size_y, start_point, goal_point, 
                               args.num_samples, args.neighbor_radius, env, path_finder)
  
  if path:
    print("---Path found.")
    print("Step 4 (Optional): Path shortcutting")
    post_process_path = path_shortcutting(path, args.max_rep, env)

  plt.savefig('img/path.png')
  return post_process_path

if __name__ == "__main__":
  main()