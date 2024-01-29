#!/usr/bin/env python
import numpy as np
import pylab as pl
pl.ion()


class TriangularObstacle(object):
  def __init__(self, x0, y0, x1, y1, x2, y2):
    self.x0 = x0
    self.y0 = y0
    self.x1 = x1
    self.y1 = y1
    self.x2 = x2
    self.y2 = y2

    self.A = np.zeros((3,2))
    self.C = np.zeros(3)

    a = x1 - x0
    b = y1 - y0
    c = x2 - x0
    d = y2 - y0
    if -b*c + a*d > 0:
      self.A[0, :] = -b, a
    else:
      self.A[0, :] = b, -a
    self.C[0] = np.dot(self.A[0, :], np.array([x0,y0]))

    a = x2 - x1
    b = y2 - y1
    c = x0 - x1
    d = y0 - y1
    if -b*c + a*d > 0:
      self.A[1, :] = -b, a
    else:
      self.A[1, :] = b, -a
    self.C[1] = np.dot(self.A[1, :], np.array([x1,y1]))

    a = x0 - x2
    b = y0 - y2
    c = x1 - x2
    d = y1 - y2
    if -b*c + a*d > 0:
      self.A[2, :] = -b, a
    else:
      self.A[2, :] = b, -a
    self.C[2] = np.dot(self.A[2, :], np.array([x2,y2]))

  def get_vertices(self):
    return ((self.x0, self.y0), (self.x1, self.y1), (self.x2, self.y2))

  def contains(self, x, y):
    r = np.dot(self.A, np.array([x,y])) - self.C
    return all([i>0 for i in r])

  def plot(self):
    pl.plot([self.x0,self.x1], [self.y0,self.y1], "r" , linewidth = 2)
    pl.plot([self.x1,self.x2], [self.y1,self.y2], "r" , linewidth = 2)
    pl.plot([self.x2,self.x0], [self.y2,self.y0], "r" , linewidth = 2)        


class Environment(object):
  def __init__(self, size_x, size_y, n_obs):
    self.size_x = size_x
    self.size_y = size_y
    self.obs = []
    for i in range(n_obs):
      x0 = np.random.rand()*size_x
      y0 = np.random.rand()*size_y
      x1 = np.random.rand()*size_x
      y1 = np.random.rand()*size_y
      x2 = np.random.rand()*size_x
      y2 = np.random.rand()*size_y
      self.obs.append(TriangularObstacle(x0, y0, x1, y1, x2, y2))

  def check_collision(self, x, y):
    for ob in self.obs:
      if ob.contains(x, y):
        return True
    return False

  def random_query(self):
    max_attempts = 100
    found_start = False
    found_goal = False
    for i in range(max_attempts):
      x_start = np.random.rand()*self.size_x
      y_start = np.random.rand()*self.size_y
      if not self.check_collision(x_start, y_start):
        found_start = True
        break
    for i in range(max_attempts):
      x_goal = np.random.rand()*self.size_x
      y_goal = np.random.rand()*self.size_y
      if not self.check_collision(x_goal, y_goal):
        found_goal = True
        break
    if found_start and found_goal:
      return x_start, y_start, x_goal, y_goal
    else:
      return None

  def plot(self):
    pl.plot([0, self.size_x, self.size_x, 0, 0], [0, 0, self.size_y, self.size_y, 0], "k", linewidth = 2)
    for ob in self.obs:
      ob.plot()

  def plot_query(self, x_start, y_start, x_goal, y_goal):
    pl.plot([x_start], [y_start], "bs", markersize = 8)
    pl.plot([x_goal], [y_goal], "y*", markersize = 12)

  def plot_point(self, x, y):
    pl.plot([x], [y], "p", markersize = 12, color = "grey")

  def plot_edge(self, point1, point2, color="b"):
    pl.plot([point1[0], point2[0]], [point1[1], point2[1]], color=color , linewidth = 2)

  def line_intersects_triangle(self, line: tuple, triangle: tuple):
    """
    Check if a line segment intersects with a triangular obstacle.

    Parameters:
    line (tuple): The line segment represented by two points ((x0, y0), (x1, y1)).
    triangle (tuple): The triangle represented by three points ((x0, y0), (x1, y1), (x2, y2)).

    Returns:
    bool: True if the line segment intersects the triangle, False otherwise.
    """
    def on_segment(p, q, r):
        """
        Check if point q lies on line segment connected by p and r.
        """
        if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
            return True
        return False

    def orientation(p, q, r):
        """
        Find orientation of ordered triplet (p, q, r).
        Returns 0 if p, q and r are collinear, 1 if Clockwise, 2 if Counterclockwise.
        """
        val = ((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1]))
        if val == 0:
            return 0  # Collinear
        elif val > 0:
            return 1  # Clockwise
        else:
            return 2  # Counterclockwise

    def do_intersect(p1, q1, p2, q2):
        """
        Check if line segments p1q1 and p2q2 intersect.
        """
        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        # General case
        if o1 != o2 and o3 != o4:
            return True

        # Special Cases: Collinearity
        if o1 == 0 and on_segment(p1, p2, q1): return True
        if o2 == 0 and on_segment(p1, q2, q1): return True
        if o3 == 0 and on_segment(p2, p1, q2): return True
        if o4 == 0 and on_segment(p2, q1, q2): return True

        return False

    # Check for intersection with each side of the triangle
    return (do_intersect(line[0], line[1], triangle[0], triangle[1]) or
            do_intersect(line[0], line[1], triangle[1], triangle[2]) or
            do_intersect(line[0], line[1], triangle[2], triangle[0]))

  def check_edge_collision(self, edge: tuple):
    """
    Check if edge collides with any obstacles in environment.

    Parameters:
      edge (tuple): The line segment represented by two points ((x0, y0), (x1, y1)).
    
    Returns:
      bool: True if edge collides with any obstacles, False otherwise.
    """
    for ob in self.obs:
      triangle = ob.get_vertices()
      if self.line_intersects_triangle(edge, triangle):
        return True
    return False