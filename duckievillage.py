# MAC0318 - Intro to Mobile Robots
# ---
# A slightly modified version of Duckietown's DuckietownEnv OpenAI Gym environment.
#
# Also contains many utility classes and functions that should prove useful for assignments.
#
# See https://github.com/duckietown/gym-duckietown for more information on Duckietown Gym.

import logging
import math
import random
from ctypes import POINTER

import numpy as np
import numpy.linalg
import cv2
import gym.wrappers.monitoring.video_recorder
import gym_duckietown.objmesh
import gym_duckietown.objects
import gym_duckietown.simulator
import gym_duckietown.envs
import gym_duckietown.wrappers
import pyglet
from pyglet import gl, window, image

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600

# Let's define a class for waypoints.
# We'll basically have two execution modes: normal and mark mode.
# When we're on mark mode, we'll be able to point and click on a position on the map and save said
# waypoints to a file. When we're not in mark mode, we'll read waypoints from a file and execute
# the route.
class Waypoints:
  def __init__(self, env, read = True, filepath = 'waypoints.txt'):
    self._i = 0
    self._env = env
    if not read:
      self._mark = True
      self._waypoints = []
    else:
      self._mark = False
      self._waypoints = None
      self.load(filepath, env)

  # This method will mark a position on the map. Coordinates (px, py) are the actual map
  # coordinates (using Duckietown's coordinate system). Meanwhile (x, y) refer to the screen
  # positions. The latter will be used for rendering.
  def mark(self, px, py, x, y):
    if self._mark:
      self._waypoints.append((px, py, x, y))
      self._env.add_cone(px, py)

  # Let's render waypoints.
  def render(self):
    gl.glPushAttrib(gl.GL_CURRENT_BIT)
    gl.glColor3f(1.0, 0.0, 0.0)
    # Here we'll render each waypoint as a red square.
    for (_, _, x, y) in self._waypoints:
      gl.glBegin(gl.GL_QUADS)
      gl.glVertex2f(x-2, y-4)
      gl.glVertex2f(x+2, y-4)
      gl.glVertex2f(x+2, y+4)
      gl.glVertex2f(x-2, y+4)
      gl.glEnd()
    gl.glLineWidth(3)
    # And let's draw a line between each waypoint, so we know where we're going next.
    gl.glBegin(gl.GL_LINE_STRIP)
    for (_, _, x, y) in self._waypoints:
      gl.glVertex2f(x, y)
    gl.glEnd()
    gl.glPopAttrib(gl.GL_CURRENT_BIT)

  # Let's also implement an iterator to make things more intuitive for the user (you).
  def __iter__(self):
    return self

  # Whenever you iterate Waypoints, it'll return ONLY the Duckietown map coordinates.
  def __next__(self):
    if self._i >= len(self._waypoints):
      self._i = 0
      raise StopIteration
    x, y, _, _ = self._waypoints[self._i]
    self._i += 1
    return x, y

  # Alternatively, we can call next to return the next waypoint. If it returns None, we're done.
  def next(self):
    if self._i >= len(self._waypoints):
      self._i = 0
      return None
    x, y, _, _ = self._waypoints[self._i]
    self._i += 1
    return x, y

  # Write to disk.
  def write(self, filepath):
    np.savetxt(filepath, self._waypoints)

  # Load from disk.
  def load(self, filepath, env):
    self._waypoints = np.loadtxt(filepath)
    for (x, y, _, _) in self._waypoints:
      env.add_cone(x, y)

# Manhattan distance
def _manhattan_dist(p, q):
  return abs(p[0]-q[0] + p[1]-q[1])

# Euclidean distance
def _euclidean_dist(p, q):
  dx, dy = p[0]-q[0], p[1]-q[1]
  return dx*dx+dy*dy

# A topological graph of a Duckietown map.
# Nodes are road tiles, edges indicate whether a road tile should be connected to another.
class TopoGraph:
  def __init__(self, r):
    self._L = {}
    self._r = r

  # Returns nodes.
  def nodes(self):
    return list(self._L.keys())

  # Returns whether there exists a directed edge p->q.
  def edge(self, p, q):
    if self.invalid_tile(p, q):
      return False
    return q in self._L[p]

  # Remove undirected edge p<->q by removing both edges p->q and q->p.
  def remove_edge(self, p, q):
    if self.invalid_tile(p, q): return
    self._L[p].pop(q, None)
    self._L[q].pop(p, None)

  # Removes directed edge p to q.
  def remove_dir_edge(self, p, q):
    if self.invalid_tile(p, q): return
    self._L[p].pop(q, None)

  # Remove node p and all edges coming from and to p.
  def remove_node(self, p):
    if p not in self._L: return
    self._L.pop(p, None)
    for n in self._L:
      self._L[n].pop(p, None)

  # Adds a node to the graph. Node must be a tuple! Convert your numpy.ndarray to tuple first.
  def add_node(self, p):
    if p not in self._L:
      self._L[p] = {}

  # Adds center position of tile to graph.
  def add_node_center(self, p):
    if p not in self._L:
      self._L[self.center_pos(p)] = {}

  # Adds an undirected edge (effectively p->q and q->p) to the graph connecting nodes p and q.
  def add_edge(self, p, q):
    self._L[p][q] = True
    self._L[q][p] = True

  # Adds a directed edge p->q.
  def add_dir_edge(self, p, q):
    self._L[p][q] = True

  # Returns whether p or q is not a road tile.
  def invalid_tile(self, p, q):
    return p not in self._L or q not in self._L

  # Returns the closest node to position p.
  def closest_node(self, p):
    m, mp = math.inf, None
    for q in self._L:
      d = _euclidean_dist(p, q)
      if m > d:
        m, mp = d, q
    return mp

  # Returns the center position of a tile (i.e. the center position of the road tile).
  def center_pos(self, p):
    return tuple((np.array(p)+0.5)*self._r)

  def uncenter_pos(self, p):
    return p[0]/self._r-0.5, p[1]/self._r-0.5

  # Breadth-first search.
  def bfs(self, p, q):
    # Get closest nodes to positions.
    p = self.closest_node(p)
    q = self.closest_node(q)
    # BFS Queue.
    Q = [p]
    # Maps visited nodes.
    V = {}
    # Keep track of parents.
    Pa = {}
    while len(Q) != 0:
      # Get first element from queue.
      n = Q.pop(0)
      if n == q:
        # Backtrack.
        P = [q]
        while P[-1] != p:
          P.append(Pa[P[-1]])
        for i, u in enumerate(P):
          P[i] = u
        return P
      for c in self._L[n]:
        if c not in V:
          V[c] = True
          Pa[c] = n
          Q.append(c)
    # Could not find a path between the two vertices: graph is disconnected.
    return None

  def path(self, p, q): return self.bfs(p, q)

  # A-star path finding.
  def astar(self, p, q):
    # Get closest nodes to positions.
    p = self.closest_node(p)
    q = self.closest_node(q)
    # We're assuming the graph is connected.
    import heapq
    # Priority queue.
    P = []
    # Nodes for backtracking.
    K = {}
    # Maps visited nodes.
    V = {}
    # Cost function.
    g = {}
    # Choose any node as root.
    V[p] = True
    g[p] = 0
    # We use Manhattan's as heuristics.
    heapq.heappush(P, (0+_manhattan_dist(p, q), p))
    while len(P) != 0:
      f, n = heapq.heappop(P)
      if n == q:
        # Backtrack.
        R = [q]
        m, mv, = None, math.inf
        u = q
        l = None
        # Greedily search for minimal tiles.
        while u != p:
          for c in self._L[u]:
            if c == p:
              m = p
              break
            # Don't let it infinitely loop! This only works in our domain, not general.
            if c != l and c in V:
              v = K[c]
              if v < mv:
                m, mv = c, mv
          l = u
          u = m
          R.append(u)
        # Returns a stack with the coordinates in reverse order.
        return R
      for c in self._L[n]:
        if c not in V:
          # Update unseen children.
          g[c] = g[n] + 1
          V[c] = True
          cost = g[c] + _manhattan_dist(c, q)
          heapq.heappush(P, (cost, c))
          K[c] = cost
        v = g[c]+_manhattan_dist(c, q)
    # The graph is not connected.
    return None

def _create_topo_graph(w: int, h: int, tiles: dict, r: float) -> TopoGraph:
  G = TopoGraph(r)
  M = [[None] * h for i in range(w)]
  for t in tiles:
    i, j = t['coords']
    M[i][j] = t
    G.add_node_center(t['coords'])
  # We're assuming adjacent drivable tiles are always connected. That's not true for the general
  # case, but let's not worry about this for now.
  for t in tiles:
    i, j = t['coords']
    if i-1 >= 0:
      u = M[i-1][j]
      if u is not None:
        G.add_edge(G.center_pos(t['coords']), G.center_pos(u['coords']))
    if j-1 >= 0:
      u = M[i][j-1]
      if u is not None:
        G.add_edge(G.center_pos(t['coords']), G.center_pos(u['coords']))
    if i+1 < w:
      u = M[i+1][j]
      if u is not None:
        G.add_edge(G.center_pos(t['coords']), G.center_pos(u['coords']))
    if j+1 < h:
      u = M[i][j+1]
      if u is not None:
        G.add_edge(G.center_pos(t['coords']), G.center_pos(u['coords']))
  return G

class JunctionGraph(TopoGraph):
  def __init__(self, r: float, M: list, env):
    super().__init__(r)
    self._W = {}
    self._M = M
    self._env = env

  def add_node(self, p: tuple):
    if p not in self._L:
      self._L[p] = {}
      self._W[p] = {}

  def add_node_center(self, p: tuple) -> tuple:
    u = self.center_pos(p)
    if u not in self._L:
      self._L[u] = {}
      self._W[u] = {}
      return u
    return u

  def add_edge(self, p: tuple, dp: str, q: tuple, dq: str, w: float):
    self._L[p][dp] = q
    self._L[q][dq] = p
    self._W[p][dp] = w
    self._W[q][dq] = w

  def wedge(self, p: tuple, d: str) -> float:
    """
    Returns the weight of the edge coming out of p and going to direction d ('n', 'e', 's', 'w').
    Returns False if node p does not exist or direction d is invalid.
    """
    if (p not in self._L) or (d not in self._L[p]): return False
    return self._L[p][d]

  def _nearest_tile(self, p: tuple) -> tuple:
    x, y = self._env.nearest_drivable(p)
    x, y = self.uncenter_pos((x, y))

    return int(x), int(y)

  def path(self, p: tuple, q: tuple) -> list:
    i, j = self._nearest_tile(p)
    x, y = self._env.pointing_direction()
    v, _, _ = _trace_up_to_junction(i, j, i-x, j-y, self._M, 0)

    gx, gy = self._nearest_tile(q)
    if "way" not in self._M[gy][gx]["kind"]:
      q, d, _ = _trace_junction_from(gx, gy, self._M)
      P = self.dijkstra(self.center_pos(v), q)
      P.append((self.center_pos(q), d))
      return P
    return self.dijkstra(self.center_pos(v), q)

  def dijkstra(self, p: tuple, q: tuple) -> list:
    p = self.closest_node(p)
    q = self.closest_node(q)
    V = set()
    D = {u: math.inf for u in self._L}
    B = {u: None for u in self._L}
    C = {u: None for u in self._L}
    D[p] = 0.0
    Q = [p]
    while len(Q) > 0:
      u = Q.pop(0)
      min_d, min_v = math.inf, None
      for di in self._L[u]:
        v = self._L[u][di]
        if v in V: continue
        w = self._W[u][di]
        d = D[u]+w
        if min_d > d: min_d, min_v = d, v
        if d < D[v]: D[v], B[v], C[v] = d, u, di
      V.add(u)
      if (q in V) or math.isinf(min_d): break
      Q.append(min_v)
    # Backtrack
    u, P = q, []
    while u != p:
      P.append((B[u], C[u]))
      u = B[u]
    P.reverse()
    return P

def _trace_junction_from(i: int, j: int, M: list) -> (tuple, str, int):
  """ Traces closest intersection from tile. """
  t = M[j][i]
  k, a = t["kind"], t["angle"]
  X, Y = None, None
  if (k == "straight"):
    if a % 2 == 0:
      # North or South
      X = _trace_up_to_junction(i, j-1, i, j, M, 0)
      Y = _trace_up_to_junction(i, j+1, i, j, M, 0)
    else: # East or West
      X = _trace_up_to_junction(i-1, j, i, j, M, 0)
      Y = _trace_up_to_junction(i+1, j, i, j, M, 0)
  elif (k == "curve_left" and a == 3) or (k == "curve_right" and a == 2):
    X = _trace_up_to_junction(i+1, j, i, j, M, 0)
    Y = _trace_up_to_junction(i, j+1, i, j, M, 0)
  elif (k == "curve_left" and a == 2) or (k == "curve_right" and a == 1):
    X = _trace_up_to_junction(i-1, j, i, j, M, 0)
    Y = _trace_up_to_junction(i, j+1, i, j, M, 0)
  elif (k == "curve_left" and a == 1) or (k == "curve_right" and a == 0):
    X = _trace_up_to_junction(i, j-1, i, j, M, 0)
    Y = _trace_up_to_junction(i-1, j, i, j, M, 0)
  else:
    X = _trace_up_to_junction(i+1, j, i, j, M, 0)
    Y = _trace_up_to_junction(i, j-1, i, j, M, 0)
  v, d, w = Y if X[2] > Y[2] else X
  return v, d, w

def _trace_up_to_junction(i: int, j: int, x: int, y: int, M: list, steps: int) -> (tuple, str, int):
  t = M[j][i]
  k = t["kind"]
  if "way" in k:
    dx, dy = i-x, j-y
    d = None
    if dy != 0: d = 's' if dy < 0 else 'n'
    if dx != 0: d = 'e' if dx < 0 else 'w'
    # Switch x and y axis for Duckietown's simulator coordinate system.
    return (i, j), d, steps
  a = t["angle"]
  if (k == "straight"):
    if a % 2 == 0:
      # North or South
      # Check if not going back to start point.
      if (i != x) or (j-1 != y): return _trace_up_to_junction(i, j-1, i, j, M, steps+1)
      return _trace_up_to_junction(i, j+1, i, j, M, steps+1)
    # Else, East or West
    if (i-1 != x) or (j != y): return _trace_up_to_junction(i-1, j, i, j, M, steps+1)
    return _trace_up_to_junction(i+1, j, i, j, M, steps+1)
  # Else, curve.
  if (k == "curve_left" and a == 3) or (k == "curve_right" and a == 2):
    if (i+1 != x) or (j != y): return _trace_up_to_junction(i+1, j, i, j, M, steps+1)
    return _trace_up_to_junction(i, j+1, i, j, M, steps+1)
  elif (k == "curve_left" and a == 2) or (k == "curve_right" and a == 1):
    if (i-1 != x) or (j != y): return _trace_up_to_junction(i-1, j, i, j, M, steps+1)
    return _trace_up_to_junction(i, j+1, i, j, M, steps+1)
  elif (k == "curve_left" and a == 1) or (k == "curve_right" and a == 0):
    if (i != x) or (j-1 != y): return _trace_up_to_junction(i, j-1, i, j, M, steps+1)
    return _trace_up_to_junction(i-1, j, i, j, M, steps+1)
  #if (k == "curve_left/S") or (k == "curve_right/W"):
  if (i+1 != x) or (j != y): return _trace_up_to_junction(i+1, j, i, j, M, steps+1)
  return _trace_up_to_junction(i, j-1, i, j, M, steps+1)

def _create_junction_graph(width: int, height: int, tiles: dict, r: float, env) -> TopoGraph:
  M = [[None] * width for i in range(height)]
  G = JunctionGraph(r, M, env)
  # Transpose due to wacky Duckietown map coordinate system.
  for t in tiles:
    i, j = t["coords"]
    M[j][i] = t

  def expand_east(u: tuple, i: int, j: int):
    v, dv, w = _trace_up_to_junction(i+1, j, i, j, M, 0)
    G.add_edge(u, 'e', G.add_node_center(v), dv, w)

  def expand_south(u: tuple, i: int, j: int):
    v, dv, w = _trace_up_to_junction(i, j+1, i, j, M, 0)
    G.add_edge(u, 's', G.add_node_center(v), dv, w)

  def expand_west(u: tuple, i: int, j: int):
    v, dv, w = _trace_up_to_junction(i-1, j, i, j, M, 0)
    G.add_edge(u, 'w', G.add_node_center(v), dv, w)

  def expand_north(u: tuple, i: int, j: int):
    v, dv, w = _trace_up_to_junction(i, j-1, i, j, M, 0)
    G.add_edge(u, 'n', G.add_node_center(v), dv, w)

  for t in tiles:
    k = t["kind"]
    # Intersection
    if "way" in k:
      u = G.add_node_center(t["coords"])
      i, j = t["coords"]
      if "4way" in k:
        expand_north(u, i, j)
        expand_south(u, i, j)
        expand_east(u, i, j)
        expand_west(u, i, j)
      else: # 3way
        a = t["angle"]
        if (k == "3way_left" and a == 2) or (k == "3way_right" and a == 0):
          expand_west(u, i, j)
          expand_north(u, i, j)
          expand_south(u, i, j)
        elif (k == "3way_left" and a == 1) or (k == "3way_right" and a == 3):
          expand_north(u, i, j)
          expand_west(u, i, j)
          expand_east(u, i, j)
        elif (k == "3way_left" and a == 0) or (k == "3way_right" and a == 2):
          expand_north(u, i, j)
          expand_east(u, i, j)
          expand_south(u, i, j)
        else: # (k == "3way_left/W") or (k == "3way_right/E")
          expand_east(u, i, j)
          expand_south(u, i, j)
          expand_west(u, i, j)

  return G

# Map of object polygons.
class PolygonMap:
  def __init__(self, env):
    # List of polygons relative to the actual Duckietown coordinate system.
    self._polys = np.empty((0, 4, 2), float)
    # List of polygons relative to the window coordinate system.
    self._polys_top = np.empty((0, 4, 2), float)
    self._env = env

  def add(self, obj):
    # Transform coordinates back to Duckietown.
    p = obj.obj_corners/(2,1)
    # Append Duckietown coordinates.
    self._polys = np.append(self._polys, [obj.obj_corners], axis=0)
    # Convert to window coordinates.
    for i, q in enumerate(p):
      p[i] = self._env.unconvert_coords(q)
    # Append window coordinates.
    self._polys_top = np.append(self._polys_top, [p], axis=0)

  @staticmethod
  def _dilate_each(polys, dilation = 0.25, abs = False):
    if not abs:
      d = dilation / 2
    for i, p in enumerate(polys):
      # Take the centroid of the polygon.
      c = np.mean(p, axis=0)
      # For each vertex, add dilated vector.
      for j, q in enumerate(p):
        if abs:
          from numpy.linalg import norm
          u = (q-c)
          u /= norm(u)
          p[j] = q+u*dilation
        else:
          p[j] = q+(q-c)*d

  # Dilates polygons by some percentage rate.
  def dilate(self, dilation = 0.25, abs = False):
    PolygonMap._dilate_each(self._polys, dilation, abs)
    PolygonMap._dilate_each(self._polys_top, dilation, abs)

  def debug(self):
    for p in self._polys:
      for q in p:
        self._env.add_cone(q)

  def render(self):
    from pyglet import gl
    gl.glPushAttrib(gl.GL_CURRENT_BIT)
    gl.glColor3f(1.0, 0.0, 0.0)
    for p in self._polys_top:
      gl.glBegin(gl.GL_LINE_STRIP)
      for i, q in enumerate(p):
        gl.glVertex2f(q[0], q[1])
      gl.glVertex2f(p[0][0], p[0][1])
      gl.glEnd()
    gl.glPopAttrib(gl.GL_CURRENT_BIT)

  def polygons(self):
    return self._polys

FULL_VIEW_MODE = 0
TOP_DOWN_VIEW_MODE = 1
FRONT_VIEW_MODE = 2
N_VIEW_MODES = 3

# A virtual odometer with a certain error rate.
class Odometer:
  def __init__(self, sigma_theta = 0.1, sigma_dist = 0.01):
    self._dist = 0
    self._theta = 0
    self._sigma_theta = sigma_theta
    self._sigma_dist = sigma_dist

  # Update odometer and applies some gaussian error to values.
  def update(self, wl, wr, r):
    self._theta += wl - wr + np.random.normal(0, self._sigma_theta)
    self._dist += ((wl + wr)*r)/2 + np.random.normal(0, self._sigma_dist)

  def measure(self):
    from math import fmod, pi
    t, d = fmod(self._theta, 360), self._dist
    if t < 0:
      t += 360
    if d < 0:
      d = 0
    self.reset()
    return d, t

  # Resets the odometer.
  def reset(self):
    self._theta = 0
    self._dist = 0

# This is a mock-up of a road tile sensor. It detects the road type of the current tile (straight,
# curve or intersection) and applies some error to make it more realistic.
class RoadSensor:
  KINDS = {'straight': 0, 'curve_left': 1, 'curve_right': 1, '4way': 2, '3way_left': 2,
           '3way_right': 2}

  def __init__(self, env, error_mu = 0.7, error_sigma = 0.4):
    self._env = env
    self._error_mu = error_mu
    self._error_sigma = error_sigma

    self._cm = ((0.7, 0.1, 0.2),
                (0.1, 0.8, 0.1),
                (0.2, 0.1, 0.7))

  # Predicts the current tile and applies an error to it. Returns None if could not
  # recognize terrain (i.e. agent is not standing on a road tile). Returns 0 if straight, 1 if
  # curve and 2 if intersection.
  def predict(self):
    for t in self._env.drivable_tiles:
      if t['coords'] == self._env.current_tile():
        from numpy.random import choice
        c = choice(3, p=self._cm[RoadSensor.KINDS[t['kind']]])
        # Returns
        return c
    return None

# Light sensor.
class LightSensor:
  DUCKIE_LOWER_HSV = np.array([0, 50, 70])
  DUCKIE_UPPER_HSV = np.array([40, 255, 255])

  @staticmethod
  def _rescale(a: float, L: float, U: float):
    if np.allclose(L, U): return 0.0
    return (a-L)/(U-L)

  def __init__(self, env):
    self._env = env
    self.l_max = -math.inf
    self.r_max = -math.inf
    self.l_min = math.inf
    self.r_min = math.inf

  # Measures "light" intensity (it really measures Duckie intensity) and returns how much power to
  # give to left and right motors.
  def measure(self, left_motor_matrix, right_motor_matrix):
    I = cv2.inRange(cv2.cvtColor(self._env.front(), cv2.COLOR_RGB2HSV),
                    LightSensor.DUCKIE_LOWER_HSV, LightSensor.DUCKIE_UPPER_HSV)
    x, y = I.shape[0], I.shape[1]
    L, R = left_motor_matrix(x, y), right_motor_matrix(x, y)
    l, r = float(np.sum(I * L)), float(np.sum(I * R))
    self.l_max = max(l, self.l_max)
    self.r_max = max(r, self.r_max)
    self.l_min = min(l, self.l_min)
    self.r_min = min(r, self.r_min)
    ls = LightSensor._rescale(l, self.l_min, self.l_max)
    rs = LightSensor._rescale(r, self.r_min, self.r_max)
    return ls, rs

class Histogram:
  def __init__(self, bel: np.ndarray, cells: list, x: float = 50, y: float = 450, dw: int = 30, h:
               float = 100, spacing: float = 5):
    self.bel = bel
    n = len(bel)
    self.w, self.h = n*(dw+spacing)-spacing, h
    self.x, self.y = x, y
    self.spacing = spacing
    self.dw = dw
    self.gap = self.dw+spacing
    self.X = [self.x+i*self.gap for i in range(n)]
    self.centers = [(c[0]+c[1])/2 for c in cells]
    self.labels = [pyglet.text.Label(f"{self.centers[i]:.2f}", x = self.X[i], y = self.y-10,
                                     font_size = 8, color = (0, 0, 0, 255)) for i in range(n)]
    self.predictions = pyglet.text.Label("x", x = x+50, y = y+h+10, font_size = 12,
                                         color = (0, 0, 0, 255))

  def render(self, estimate: float):
    gl.glPushAttrib(gl.GL_CURRENT_BIT)

    for i, p in enumerate(self.bel):
      x = self.X[i]
      y = self.y+self.h*p
      gl.glColor3f(0.0, 0.0, 0.0)
      gl.glBegin(gl.GL_QUADS)
      gl.glVertex2f(x, self.y)
      gl.glVertex2f(x+self.dw, self.y)
      gl.glVertex2f(x+self.dw, self.y+self.h)
      gl.glVertex2f(x, self.y+self.h)
      gl.glEnd()

      gl.glColor3f(0.0, 1.0, 0.0)
      gl.glBegin(gl.GL_QUADS)
      gl.glVertex2f(x, self.y)
      gl.glVertex2f(x+self.dw, self.y)
      gl.glVertex2f(x+self.dw, y)
      gl.glVertex2f(x, y)
      gl.glEnd()

    gl.glPopAttrib(gl.GL_CURRENT_BIT)
    self.predictions.text = f"Estimate: {estimate:.3f}\tMAP: {self.centers[np.argmax(self.bel)]:.3f}"
    self.predictions.draw()

    for l in self.labels: l.draw()

class Evaluator:
  def __init__(self, env):
    env.max_steps = math.inf
    self._env = env
    self._log = {}

  def infraction(self, t: str, penalty: float, warning: str):
    print(warning)
    if t not in self._log: self._log[t] = []
    self._log[t].append((penalty, warning))

  def track(self):
    r = self._env.penalization(self._env.cur_pos, self._env.cur_angle)
    if r is None: return
    if r == "out":
      self.infraction(r, -1, "Mailduck has gone off-road!")
    if r == "crash":
      self.infraction(r, -1, "Mailduck has crashed into something!")

def _draw_circle(x, y, color, radius=0.1, sides=8, infill=False):
  gl.glPushAttrib(gl.GL_CURRENT_BIT)
  gl.glColor3f(*color)
  if infill:
    gl.glBegin(gl.GL_POLYGON)
  else:
    gl.glBegin(gl.GL_LINE_LOOP)
  pi2 = np.pi * 2
  for i in range(sides):
    dx = radius*np.cos(i*pi2/sides)
    dy = radius*np.sin(i*pi2/sides)
    gl.glVertex3f(x+dx, 0.01, y+dy)
  gl.glEnd()
  gl.glPopAttrib(gl.GL_CURRENT_BIT)

class Mailbox:
  """
    Mailbox tracks all packages (to be) delivered.

    Function mail returns a list where each element is a triple representing a package:

      (d, x, y)

    d - whether this package has been delivered
    x - (x-axis) position of where this package should be delivered
    y - (y-axis) position of where this package should be delivered

    The id of each package is its list index.

    Function deliver causes the robot to attempt a delivery from the current position. If the robot
    is close enough to the target, then the attempt is successful and the item is set as delivered.
  """

  def parse(self, f:str) -> list:
    L = []
    with open(f) as file:
      file.readline() # Skip map name
      for line in file:
        t = line.strip().split()
        x, y = int(t[0]), int(t[1])
        L.append([False, (x+0.5)*self._env.road_tile_size, (y+0.5)*self._env.road_tile_size])
    return L

  def __init__(self, env, in_file: str):
    self._env = env
    self._mailing_list = self.parse(in_file)
    self._delivered_color = (0, 1, 0)
    self._todeliver_color = (0, 0, 1)

  def render(self):
    for (d, x, y) in self._mailing_list:
      color = self._delivered_color if d else self._todeliver_color
      gl.glDisable(gl.GL_TEXTURE_2D)
      _draw_circle(x, y, color, infill=True)

  def mail(self):
    return self._mailing_list

  def deliver(self, i: int) -> bool:
    x, y = self._env.get_position()
    d, a, b = self._mailing_list[i]
    if d:
      self._env.eval.infraction("mail-deliver", -0.5, "Attempted to deliver already delivered package!")
      return False
    dist = (x-a)*(x-a)+(y-b)*(y-b)
    if dist > 0.05:
      self._env.eval.infraction("mail-dist", -0.5, "Attempted to deliver package from out of the receiver's influence radius!")
      return False
    self._mailing_list[i][0] = True
    return True

class GPS:
  def __init__(self, env, sigma: float = -1):
    """ Constructs a GPS which has a(n) (isotropic) gaussian error centered at the true position
    and fixed standard deviation given by sigma. """
    self._env = env
    if sigma <= 0: sigma = 0.01
    self._sigma = (sigma, sigma)

  def track(self) -> np.ndarray:
    """ Returns the position of the agent with an error. """
    return np.random.normal(loc = self._env.get_position(), scale = self._sigma)

class Mileage:
  def __init__(self): self._mileage = 0
  def update(self, dt: float): self._mileage += dt
  def mileage(self): return self._mileage

def create_env(raw_motor_input: bool = True, noisy: bool = False, **kwargs):
  class DuckievillageEnv(gym_duckietown.envs.DuckietownNoisyEnv if noisy else
                         gym_duckietown.simulator.Simulator if raw_motor_input else gym_duckietown.envs.DuckietownEnv):
    top_down = False

    def __init__(self, top_down = False, cam_height = 5, enable_topomap: bool = False,
                 enable_polymap: bool = False, enable_roadsensor: bool = False,
                 enable_odometer: bool = False, enable_lightsensor: bool = False,
                 enable_junction: bool = False, enable_gps: bool = False,
                 enable_eval: bool = False, enable_mailbox: bool = False,
                 mailbox_file: str = None, enable_mileage: bool = False,
                 video_path: str = None, **kwargs):
      super().__init__(**kwargs)
      self.horizon_color = self._perturb(self.color_sky)
      self.cam_fov_y = gym_duckietown.simulator.CAMERA_FOV_Y
      self.top_down = top_down
      self.topo_graph = _create_topo_graph(self.grid_width, self.grid_height, self.drivable_tiles,
                                           self.road_tile_size) if enable_topomap else None
      self.junction_graph = _create_junction_graph(self.grid_width, self.grid_height,
                                                   self.drivable_tiles, self.road_tile_size, self) if enable_junction else None
      self.gps = GPS(self) if enable_gps else None

      self.mailbox = Mailbox(self, mailbox_file) if enable_mailbox else None
      self.eval = Evaluator(self) if enable_eval else None
      self.mileage = Mileage() if enable_mileage else None

      if enable_polymap:
        self.poly_map = PolygonMap(self)
        for o in self.objects:
          self.poly_map.add(o)
      else: self.poly_map = None

      self._view_mode = 0
      self.top_cam_height = cam_height

      self.odometer = Odometer() if enable_odometer else None

      if enable_roadsensor:
        self.road_sensor = RoadSensor(self)

        self._roads = []
        for t in self.drivable_tiles:
          k = t['kind'].split('_', 1)[0]
          if k == '3way' or k == '4way':
            k = 'inter'
          self._roads.append((t['coords'], k))
      else: self.road_sensor = None

      self.lightsensor = LightSensor(self) if enable_lightsensor else None
      self.actions = [0, 0]

      if video_path is not None:
        self.rec = gym.wrappers.monitoring.video_recorder.VideoRecorder(self, path=video_path, enabled = True)
      else: self.rec = None

      self.renderables = None

    def add_renderable(self, r):
      if self.renderables is None: self.renderables = [r]
      else: self.renderables.append(r)
      return self.renderables

    def next_view(self):
      self._view_mode = (self._view_mode + 1) % N_VIEW_MODES

    def set_view(self, view):
      self._view_mode = view % N_VIEW_MODES

    def toggle_single_view(self):
      if self._view_mode == TOP_DOWN_VIEW_MODE:
        self._view_mode = FRONT_VIEW_MODE
      else:
        self._view_mode = TOP_DOWN_VIEW_MODE

    # Returns a list with all road tiles in the current map. Each item of the list contains the
    # tile indices (not positional coordinates), and the tile type (curve, straight, intersection).
    def roads(self):
      return self._roads

    def current_tile(self):
      return self.get_grid_coords(self.cur_pos)

    def tile_center(self, i, j=None):
      if j is None:
        i, j = i[0], i[1]
      return (np.array([i, j])+0.5)*self.road_tile_size

    def nearest_drivable(self, x: float, y: float = None) -> (float, float):
      if y is None: x, y = x[0], x[1]
      min_d, min_p = math.inf, None
      for t in self.drivable_tiles:
        a, b = t["coords"]
        cx, cy = (a+0.5)*self.road_tile_size, (b+0.5)*self.road_tile_size
        d = (cx-x)*(cx-x)+(cy-y)*(cy-y)
        if min_d > d: min_d, min_p = d, (cx, cy)
      return min_p

    def get_position(self):
      return np.delete(self.cur_pos, 1)

    def top_down_obs(self, segment = False):
      return self._render_img(
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        self.multi_fbo_human,
        self.final_fbo_human,
        self.img_array_human,
        top_down = True,
        segment=segment,
        callback = (lambda: self.mailbox.render()) if self.mailbox is not None else None,
      )

    def front(self, segment = False):
      return self._render_img(
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        self.multi_fbo_human,
        self.final_fbo_human,
        self.img_array_human,
        top_down = False,
        segment=segment,
        callback = (lambda: self.mailbox.render()) if self.mailbox is not None else None,
      )

    def render(self, mode: str = "human", close: bool = False, segment: bool = False, text: str = ""):
      """
      Render the environment for human viewing

      mode: "human", "top_down", "free_cam", "rgb_array"

      """
      assert mode in ["human", "top_down", "free_cam", "rgb_array"]

      if close:
        if self.window:
          self.window.close()
        return

      top_down = mode == 'top_down'
      # Render the image
      top = self.top_down_obs(segment)
      bot = self.front(segment)

      if self.distortion and not self.undistort and mode != "free_cam":
        bot = self.camera_model.distort(bot)

      win_width = WINDOW_WIDTH
      if self._view_mode == FULL_VIEW_MODE:
        img = np.concatenate((top, bot), axis=1)
        win_width = 2*WINDOW_WIDTH
      elif self._view_mode == TOP_DOWN_VIEW_MODE:
        img = top
      else:
        img = bot


      if self.window is not None:
        self.window.set_size(win_width, WINDOW_HEIGHT)

      if mode == 'rgb_array':
        return img

      if self.window is None:
        config = gl.Config(double_buffer=False)
        self.window = window.Window(
          width=win_width,
          height=WINDOW_HEIGHT,
          resizable=False,
          config=config
        )

      self.window.clear()
      self.window.switch_to()
      self.window.dispatch_events()

      # Bind the default frame buffer
      gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)

      # Setup orghogonal projection
      gl.glMatrixMode(gl.GL_PROJECTION)
      gl.glLoadIdentity()
      gl.glMatrixMode(gl.GL_MODELVIEW)
      gl.glLoadIdentity()
      gl.glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, 0, 10)

      # Draw the image to the rendering window
      width = img.shape[1]
      height = img.shape[0]
      img = np.ascontiguousarray(np.flip(img, axis=0))
      img_data = image.ImageData(
        width,
        height,
        'RGB',
        img.ctypes.data_as(POINTER(gl.GLubyte)),
        pitch=width * 3,
      )
      img_data.blit(
        0,
        0,
        0,
        width=WINDOW_WIDTH,
        height=WINDOW_HEIGHT
      )

      # Display position/state information
      if mode != "free_cam":
        x, y, z = self.cur_pos
        self.text_label.text = (
          f"pos: ({x:.2f}, {y:.2f}, {z:.2f}), angle: "
          f"{np.rad2deg(self.cur_angle):.1f} deg, steps: {self.step_count}, "
          f"speed: {self.speed:.2f} m/s"
        )
        if len(text) > 0: self.text_label.text += text
        self.text_label.draw()

      # Force execution of queued commands
      gl.glFlush()

      if self.rec is not None:
        self.rec.capture_frame()

      if self.renderables is not None:
        for r in self.renderables: r.render()

      return img

    def close(self):
      if self.rec is not None:
        self.rec.close()
      super().close()

    def reset(self, segment: bool = False, force = True):
      if force:
        self.force_reset(segment)

    def step(self, pwm_left: float, pwm_right: float):
      self.actions[0], self.actions[1] = pwm_left, pwm_right
      obs, reward, done, info = super().step(self.actions)
      if self.odometer is not None:
        metrics = info['DuckietownEnv']
        self.odometer.update(metrics['omega_l'], metrics['omega_r'], metrics['radius'])
      return obs, reward, done, info

    def pointing_direction(self) -> tuple:
      q = math.pi*0.25
      theta = self.cur_angle
      # East
      if q >= theta > -q:
        return 1, 0
      t = math.pi*0.75
      # North
      if t >= theta > q:
        return 0, -1
      # South
      if -q >= theta > -t:
        return 0, 1
      # West
      return -1, 0

    def get_dir_vec(self):
      return gym_duckietown.simulator.get_dir_vec(self.cur_angle)

    def force_reset(self, segment: bool = False):
      super().reset(segment)

    # We have to convert from window positions to actual Duckietown coordinates.
    def convert_coords(self, x: int, y: int) -> (float, float):
      w = self.grid_width+3
      h = self.grid_height+2
      dw = WINDOW_WIDTH/w
      dh = WINDOW_HEIGHT/h
      x -= 1.5*dw
      y += dh
      x, y = x/dw, (WINDOW_HEIGHT-y) / dh
      return (x+0.5)*self.road_tile_size, (y+0.5)*self.road_tile_size

    # The inverse transformation of the above.
    def unconvert_coords(self, x: float, y: float = None) -> (int, int):
      if y is None:
        x, y = x[0], x[1]
      x, y = x/self.road_tile_size-0.5, y/self.road_tile_size-0.5
      w = self.grid_width+3
      h = self.grid_height+2
      dw = WINDOW_WIDTH/w
      dh = WINDOW_HEIGHT/h
      x, y = dw*x, dh*y-WINDOW_HEIGHT
      return int(x + 1.5*dw), int(y - dh)

    def add_duckie(self, x, y = None, static = True, optional = False):
      if y is None:
        x, y = x[0], x[1]
      obj = _get_obj_props('duckie', x, y, static, optional = optional)
      self.objects.append(gym_duckietown.objects.DuckieObj(obj, False,
                                                           gym_duckietown.simulator.SAFETY_RAD_MULT,
                                                           self.road_tile_size))

    def add_big_duckie(self, x, y = None, static = True, optional = False):
      if y is None:
        x, y = x[0], x[1]
      obj = _get_obj_props('duckie', x, y, static, rescale = 3.0, optional = optional)
      self.objects.append(gym_duckietown.objects.DuckieObj(obj, False,
                                                           gym_duckietown.simulator.SAFETY_RAD_MULT,
                                                           self.road_tile_size))

    def add_static_duckie(self, x, y = None, angle = None):
      if y is None: x, y = x[0], x[1]
      if angle is None: angle = self.np_random.random()*math.pi
      obj = _get_obj_props('duckie', x, y, True, rescale = 3.0, angle = angle)
      self.objects.append(gym_duckietown.objects.WorldObj(obj, False, gym_duckietown.simulator.SAFETY_RAD_MULT))

    def add_static_big_duckie(self, x, y = None, angle = None):
      if y is None: x, y = x[0], x[1]
      if angle is None: angle = self.np_random.random()*math.pi
      obj = _get_obj_props('duckie', x, y, True, angle = angle)
      self.objects.append(gym_duckietown.objects.WorldObj(obj, False, gym_duckietown.simulator.SAFETY_RAD_MULT))

    def add_static_duckiebot(self, x, y = None, angle = None):
      if y is None: x, y = x[0], x[1]
      if angle is None: angle = self.np_random.random()*math.pi
      obj = _get_obj_props('duckiebot', x, y , True, rescale = 2.0, angle = angle)
      self.objects.append(gym_duckietown.objects.WorldObj(obj, False, gym_duckietown.simulator.SAFETY_RAD_MULT))

    def add_cone(self, x, y = None, scale = 1.0):
      if y is None:
        x, y = x[0], x[1]
      obj = _get_obj_props('cone', x, y, rescale = scale)
      c = gym_duckietown.objects.WorldObj(obj, False, gym_duckietown.simulator.SAFETY_RAD_MULT)
      self.objects.append(c)
      return c

    def add_walking_duckie(self, x, y = None):
      if y is None:
        x, y = x[0], x[1]
      obj = _get_obj_props('duckie', x, y, False)
      obj['kind'] = 'duckiebot'
      d = gym_duckietown.objects.DuckiebotObj(obj, False, gym_duckietown.simulator.SAFETY_RAD_MULT,
                                              gym_duckietown.simulator.WHEEL_DIST,
                                              gym_duckietown.simulator.ROBOT_WIDTH,
                                              gym_duckietown.simulator.ROBOT_LENGTH)
      self.objects.append(d)
      return d

    def add_light(self, x, y):
      li = gl.GL_LIGHT0 + 1

      li_pos = [x, 0.5, y, 1.0]
      diffuse = [0.5, 0.5, 0.5, 0.5]
      ambient = [0.5, 0.5, 0.5, 0.5]
      specular = [0.5, 0.5, 0.5, 1.0]
      spot_direction = [0.5, -0.5, 0.5]
      gl.glLightfv(li, gl.GL_POSITION, (gl.GLfloat * 4)(*li_pos))
      gl.glLightfv(li, gl.GL_AMBIENT, (gl.GLfloat * 4)(*ambient))
      gl.glLightfv(li, gl.GL_DIFFUSE, (gl.GLfloat * 4)(*diffuse))
      gl.glLightfv(li, gl.GL_SPECULAR, (gl.GLfloat * 4)(*specular))
      gl.glLightfv(li, gl.GL_SPOT_DIRECTION, (gl.GLfloat * 3)(*spot_direction))
      # gl.glLightfv(li, gl.GL_SPOT_EXPONENT, (gl.GLfloat * 1)(64.0))
      gl.glLightf(li, gl.GL_SPOT_CUTOFF, 60)

      gl.glLightfv(li, gl.GL_CONSTANT_ATTENUATION, (gl.GLfloat * 1)(1.0))
      # gl.glLightfv(li, gl.GL_LINEAR_ATTENUATION, (gl.GLfloat * 1)(0.1))
      gl.glLightfv(li, gl.GL_QUADRATIC_ATTENUATION, (gl.GLfloat * 1)(0.2))
      gl.glEnable(li)

    def sine_target(self, t: np.ndarray, s: np.ndarray = None):
      if s is None:
        s = np.delete(self.get_dir_vec(), 1)
      u, v = s/np.linalg.norm(s), t/np.linalg.norm(t)
      return np.cross(u, v)/(np.linalg.norm(u, ord=1)*np.linalg.norm(v, ord=1))

    def lf_target(self):
      _, ct, c0 = self.closest_curve_point(self.cur_pos, self.cur_angle, delta = 0.2)
      u, v = np.delete(c0, 1), np.delete(ct, 1)
      p = u-self.get_position()
      t = math.asin(self.sine_target(v))
      d = np.linalg.norm(p)*np.sign(self.sine_target(p, v))
      return d, t

    def tile_position(self, x, y, centered: bool = False) -> (float, float):
      s = self.road_tile_size
      if centered: return x*s+s/2, y*s+s/2
      return x*s, y*s

    road_tiles = set(["curve_left", "curve_right", "straight", "4way", "3way_left", "3way_right"])

    def random_road_pose(self) -> (np.ndarray, float):
      R = [np.array(t["coords"])*self.road_tile_size+self.road_tile_size/2 for t in self.grid if t["kind"] in DuckievillageEnv.road_tiles]
      return np.insert(random.choice(R), 1, 0), math.pi*self.np_random.random()

  return DuckievillageEnv(**kwargs)

def _get_obj_props(kind, x, y, static = True, rescale = 1.0, angle = 0.0, optional = True):
  mesh = gym_duckietown.objmesh.get_mesh(kind)
  return {
    'kind': kind,
    'mesh': mesh,
    'angle': angle,
    'pos': np.array([x, 0, y]),
    'scale': (0.06 / mesh.max_coords[1])*rescale,
    'y_rot': 0,
    'optional': optional,
    'static': static
  }
