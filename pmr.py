import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('osr_examples/scripts/')
import environment_2d
import heapq

def sample_free_points(num_samples: int, size_x: int, size_y: int):
  """
  """
  points = []
  while len(points) < num_samples:
    x = np.random.rand() * size_x
    y = np.random.rand() * size_y
    if not env.check_collision(x,y):
      points.append((x,y))
  return points

def compute_euclid_distance(point1: tuple, point2: tuple):
  """
  """
  return np.linalg.norm(np.array(point1) - np.array(point2))

def construct_init_roadmap(points: list, neighbor_radius: float):
  """
  """
  roadmap = {}

  for i, point1 in enumerate(points):
    roadmap[i] = []
    for j, point2 in enumerate(points):
      if i != j:
        if compute_euclid_distance(point1, point2) <= neighbor_radius:
          edge = (point1, point2)
          if not env.check_edge_collision(edge):
            roadmap[i].append(j)
  return roadmap

def add_start_goal_to_roadmap(points: list, roadmap: dict, neighbor_radius: float,
                              start_point: tuple, goal_point: tuple):
  """
  """
  start_index = len(points)  # index for the start point
  goal_index = start_index + 1  # index for the goal point

  # Add start and goal points to the roadmap
  for index, point in enumerate([start_point, goal_point]):
    current_index = start_index if index == 0 else goal_index
    roadmap[current_index] = []

    for i, other_point in enumerate(points):
      if compute_euclid_distance(point, other_point) <= neighbor_radius and \
        not env.check_edge_collision((point, other_point)):
        roadmap[current_index].append(i)  # Connect to other points
        roadmap[i].append(current_index)

  return roadmap, start_index, goal_index


def dijkstra(roadmap, start_idx, goal_idx):
  """
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


def Astar(roadmap, start_idx, goal_idx):
  """
  """


def main(start_point: tuple, goal_point: tuple, num_samples: int, neighbor_radius: int):
  if not env.check_edge_collision((start_point, goal_point)):
    env.plot_edge(start_point, goal_point)
    return [start_point, goal_point]
  
  # Sample free points
  sample_points = sample_free_points(num_samples, 10, 6)
  for p in sample_points:
    env.plot_point([p[0]], [p[1]])

  # Create a navigateable roadmap by connecting the sample points within neighbor_radius
  init_roadmap = construct_init_roadmap(sample_points, neighbor_radius)

  # Add start and goal points to the roadmap
  updated_roadmap, start_idx, goal_idx = add_start_goal_to_roadmap(sample_points, init_roadmap, neighbor_radius, start_point, goal_point)

  # Find path based on the roadmap found
  path_in_indices = dijkstra(updated_roadmap, start_idx, goal_idx)
  if len(path_in_indices) == 1:
    print("No paths exist in current roadmap.")
    return None
  path_in_coordinates = [start_point] + [sample_points[i] for i in path_in_indices[1:len(path_in_indices)-1]] + [goal_point]

  # Plot path found
  for i in range(len(path_in_coordinates)-1):
    point1 = path_in_coordinates[i]
    point2 = path_in_coordinates[i+1]
    env.plot_edge(point1, point2, color="black")
  plt.savefig('img/path.png')
  return path_in_coordinates

if __name__ == "__main__":
  plt.ion()
  # np.random.seed(4)
  # env = environment_2d.Environment(10, 6, 5)
  np.random.seed(5)
  env = environment_2d.Environment(10, 6, 3)
  plt.clf()
  env.plot()
  q = env.random_query()
  if q is not None:
    x_start, y_start, x_goal, y_goal = q
    env.plot_query(x_start, y_start, x_goal, y_goal)
  plt.savefig('img/setup.png')

  start_point = (x_start, y_start)
  goal_point = (x_goal, y_goal)

  num_samples = 100
  neighbor_radius = 10
  
  path = main(start_point, goal_point, num_samples, neighbor_radius)
  if path:
    print("Path length: {}\nPath: {}".format(len(path)-1, path))