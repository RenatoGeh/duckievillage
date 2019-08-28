# MAC0318 - Intro to Mobile Robots
# ---
# A slightly modified version of Duckietown's DuckietownEnv OpenAI Gym environment.
#
# Also contains many utility classes and functions that should prove useful for assignments.
#
# See https://github.com/duckietown/gym-duckietown for more information on Duckietown Gym.

import logging
import math
from ctypes import POINTER

import numpy as np
import gym_duckietown.objmesh
import gym_duckietown.objects
import gym_duckietown.simulator
import gym_duckietown.envs
import gym_duckietown.wrappers

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
    from pyglet import gl
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
  return np.linalg.norm(np.array(q)-np.array(p))

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

  # A-star path finding.
  def path(self, p, q):
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

class DuckievillageEnv(gym_duckietown.envs.DuckietownEnv):
  top_down = False

  def __init__(self, top_down = False, cam_height = 5, **kwargs):
    gym_duckietown.envs.DuckietownEnv.__init__(self, **kwargs)
    self.top_down = top_down
    logger = logging.getLogger('gym-duckietown')
    logger.propagate = False
    self.force_reset()
    self.topo_graph = _create_topo_graph(self.grid_width, self.grid_height, self.drivable_tiles,
                                         self.road_tile_size)
    self.poly_map = PolygonMap(self)
    for o in self.objects:
      self.poly_map.add(o)

    self._view_mode = 0
    self.top_cam_height = cam_height

    self.odometer = Odometer()
    self.road_sensor = RoadSensor(self)

    self._roads = []
    for t in self.drivable_tiles:
      k = t['kind'].split('_', 1)[0]
      if k == '3way' or k == '4way':
        k = 'inter'
      self._roads.append((t['coords'], k))

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

  def render_obs(self):
    obs = self._render_img(
      self.camera_width,
      self.camera_height,
      self.multi_fbo,
      self.final_fbo,
      self.img_array,
      top_down = self.top_down
    )
    if self.distortion and not self.undistort:
      obs = self.camera_model.distort(obs)
    return obs

  def current_tile(self):
    return self.get_grid_coords(self.cur_pos)

  def tile_center(self, i, j=None):
    if j is None:
      i, j = i[0], i[1]
    return (np.array([i, j])+0.5)*self.road_tile_size

  def get_position(self):
    return np.delete(self.cur_pos, 1)

  def top_down(self):
    return self._render_img(
      WINDOW_WIDTH,
      WINDOW_HEIGHT,
      self.multi_fbo,
      self.final_fbo,
      self.img_array,
      top_down = True
    )

  def front(self):
    return self._render_img(
      WINDOW_WIDTH,
      WINDOW_HEIGHT,
      self.multi_fbo,
      self.final_fbo,
      self.img_array,
      top_down = True
    )

  def _render_img(self, width, height, multi_fbo, final_fbo, img_array, top_down=True):
    """
    Render an image of the environment into a frame buffer
    Produce a numpy RGB array image as output
    """

    if not self.graphics:
      return

    # Switch to the default context
    # This is necessary on Linux nvidia drivers
    # pyglet.gl._shadow_window.switch_to()
    self.shadow_window.switch_to()

    from pyglet import gl
    # Bind the multisampled frame buffer
    gl.glEnable(gl.GL_MULTISAMPLE)
    gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, multi_fbo)
    gl.glViewport(0, 0, width, height)

    # Clear the color and depth buffers

    c0, c1, c2 = self.horizon_color
    gl.glClearColor(c0, c1, c2, 1.0)
    gl.glClearDepth(1.0)
    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

    # Set the projection matrix
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    gl.gluPerspective(
      self.cam_fov_y,
      width / float(height),
      0.04,
      100.0
    )

    # Set modelview matrix
    # Note: we add a bit of noise to the camera position for data augmentation
    pos = self.cur_pos
    angle = self.cur_angle
    if self.domain_rand:
      pos = pos + self.randomization_settings['camera_noise']

    x, y, z = pos + self.cam_offset
    dx, dy, dz = self.get_dir_vec(angle)
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()

    if self.draw_bbox:
      y += 0.8
      gl.glRotatef(90, 1, 0, 0)
    elif not top_down:
      y += self.cam_height
      gl.glRotatef(self.cam_angle[0], 1, 0, 0)
      gl.glRotatef(self.cam_angle[1], 0, 1, 0)
      gl.glRotatef(self.cam_angle[2], 0, 0, 1)
      gl.glTranslatef(0, 0, self._perturb(gym_duckietown.simulator.CAMERA_FORWARD_DIST))

    if top_down:
      gl.gluLookAt(
        # Eye position
        (self.grid_width * self.road_tile_size) / 2,
        self.top_cam_height,
        (self.grid_height * self.road_tile_size) / 2,
        # Target
        (self.grid_width * self.road_tile_size) / 2,
        0,
        (self.grid_height * self.road_tile_size) / 2,
        # Up vector
        0, 0, -1.0
      )
    else:
      gl.gluLookAt(
        # Eye position
        x,
        y,
        z,
        # Target
        x + dx,
        y + dy,
        z + dz,
        # Up vector
        0, 1.0, 0.0
      )

    # Draw the ground quad
    gl.glDisable(gl.GL_TEXTURE_2D)
    gl.glColor3f(*self.ground_color)
    gl.glPushMatrix()
    gl.glScalef(50, 1, 50)
    self.ground_vlist.draw(gl.GL_QUADS)
    gl.glPopMatrix()

    # Draw the ground/noise triangles
    self.tri_vlist.draw(gl.GL_TRIANGLES)

    # Draw the road quads
    gl.glEnable(gl.GL_TEXTURE_2D)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

    # For each grid tile
    for j in range(self.grid_height):
      for i in range(self.grid_width):
        # Get the tile type and angle
        tile = self._get_tile(i, j)

        if tile is None:
          continue

        # kind = tile['kind']
        angle = tile['angle']
        color = tile['color']
        texture = tile['texture']

        gl.glColor3f(*color)

        gl.glPushMatrix()
        gl.glTranslatef((i + 0.5) * self.road_tile_size, 0, (j + 0.5) * self.road_tile_size)
        gl.glRotatef(angle * 90, 0, 1, 0)

        # Bind the appropriate texture
        texture.bind()

        self.road_vlist.draw(gl.GL_QUADS)
        gl.glPopMatrix()

        if self.draw_curve and tile['drivable']:
          # Find curve with largest dotproduct with heading
          curves = self._get_tile(i, j)['curves']
          curve_headings = curves[:, -1, :] - curves[:, 0, :]
          curve_headings = curve_headings / np.linalg.norm(curve_headings).reshape(1, -1)
          dirVec = get_dir_vec(angle)
          dot_prods = np.dot(curve_headings, dirVec)

          # Current ("closest") curve drawn in Red
          pts = curves[np.argmax(dot_prods)]
          bezier_draw(pts, n=20, red=True)

          pts = self._get_curve(i, j)
          for idx, pt in enumerate(pts):
            # Don't draw current curve in blue
            if idx == np.argmax(dot_prods):
                continue
            bezier_draw(pt, n=20)

    # For each object
    for idx, obj in enumerate(self.objects):
      obj.render(self.draw_bbox)

    # Draw the agent's own bounding box
    if self.draw_bbox:
      corners = get_agent_corners(pos, angle)
      gl.glColor3f(1, 0, 0)
      gl.glBegin(gl.GL_LINE_LOOP)
      gl.glVertex3f(corners[0, 0], 0.01, corners[0, 1])
      gl.glVertex3f(corners[1, 0], 0.01, corners[1, 1])
      gl.glVertex3f(corners[2, 0], 0.01, corners[2, 1])
      gl.glVertex3f(corners[3, 0], 0.01, corners[3, 1])
      gl.glEnd()

    if top_down:
      gl.glPushMatrix()
      gl.glTranslatef(*self.cur_pos)
      gl.glScalef(1, 1, 1)
      gl.glRotatef(self.cur_angle * 180 / np.pi, 0, 1, 0)
      # glColor3f(*self.color)
      self.mesh.render()
      gl.glPopMatrix()

    # Resolve the multisampled frame buffer into the final frame buffer
    gl.glBindFramebuffer(gl.GL_READ_FRAMEBUFFER, multi_fbo)
    gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, final_fbo)
    gl.glBlitFramebuffer(
      0, 0,
      width, height,
      0, 0,
      width, height,
      gl.GL_COLOR_BUFFER_BIT,
      gl.GL_LINEAR
    )

    # Copy the frame buffer contents into a numpy array
    # Note: glReadPixels reads starting from the lower left corner
    gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, final_fbo)
    gl.glReadPixels(
      0,
      0,
      width,
      height,
      gl.GL_RGB,
      gl.GL_UNSIGNED_BYTE,
      img_array.ctypes.data_as(POINTER(gl.GLubyte))
    )

    # Unbind the frame buffer
    gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)

    # Flip the image because OpenGL maps (0,0) to the lower-left corner
    # Note: this is necessary for gym.wrappers.Monitor to record videos
    # properly, otherwise they are vertically inverted.
    img_array = np.ascontiguousarray(np.flip(img_array, axis=0))

    return img_array

    def render_obs(self):
        """
        Render an observation from the point of view of the agent
        """

        observation = self._render_img(
                self.camera_width,
                self.camera_height,
                self.multi_fbo,
                self.final_fbo,
                self.img_array,
                top_down=True
        )

        # self.undistort - for UndistortWrapper
        if self.distortion and not self.undistort:
            observation = self.camera_model.distort(observation)

        return observation



  def render(self, mode='human', close=False, text=False):
    """
    Render the environment for human viewing
    """

    if close:
      if self.window:
        self.window.close()
      return

    top_down = mode == 'top_down'
    # Render the image
    top = self._render_img(
      WINDOW_WIDTH,
      WINDOW_HEIGHT,
      self.multi_fbo_human,
      self.final_fbo_human,
      self.img_array_human,
      top_down = True
    )
    bot = self._render_img(
      WINDOW_WIDTH,
      WINDOW_HEIGHT,
      self.multi_fbo_human,
      self.final_fbo_human,
      self.img_array_human,
      top_down = False
    )

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

    # self.undistort - for UndistortWrapper
    if self.distortion and not self.undistort and mode != "free_cam":
      img = self.camera_model.distort(img)

    if mode == 'rgb_array':
      return img

    from pyglet import gl, window, image

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
    if text and mode != "free_cam":
      x, y, z = self.cur_pos
      self.text_label.text = "pos: (%.2f, %.2f, %.2f), angle: %d, steps: %d, speed: %.2f m/s" % (
        x, y, z,
        int(self.cur_angle * 180 / math.pi),
        self.step_count,
        self.speed
      )
      self.text_label.draw()

    # Force execution of queued commands
    gl.glFlush()

  def reset(self, force=False):
    if force:
      self.force_reset()

  def step(self, action):
    obs, reward, done, info = gym_duckietown.envs.DuckietownEnv.step(self, action)
    metrics = info['DuckietownEnv']
    self.odometer.update(metrics['omega_l'], metrics['omega_r'], metrics['radius'])
    return obs, reward, done, info

  def force_reset(self):
    gym_duckietown.envs.DuckietownEnv.reset(self)

  # We have to convert from window positions to actual Duckietown coordinates.
  def convert_coords(self, x: int, y: int) -> (float, float):
    # Maps are up to grid_width tiles high. Assuming square tiles and invariant tile dimensions:
    hx = self.grid_width * self.road_tile_size
    hy = self.grid_height * self.road_tile_size
    rx =  hx / WINDOW_WIDTH
    ry =  hy / WINDOW_HEIGHT
    # Do some mathemagics.
    return x*rx, hy - y*ry

  # The inverse transformation of the above.
  def unconvert_coords(self, x: float, y: float = None) -> (int, int):
    if y is None:
      x, y = x[0], x[1]
    hx = self.grid_width * self.road_tile_size
    hy = self.grid_height * self.road_tile_size
    rx = WINDOW_WIDTH / hx
    ry = WINDOW_HEIGHT / hy
    return round(x*rx), round(WINDOW_HEIGHT - y*ry)

  def add_duckie(self, x, y = None, static = True):
    if y is None:
      x, y = x[0], x[1]
    obj = _get_obj_props('duckie', x, y, static)
    self.objects.append(gym_duckietown.objects.DuckieObj(obj, False,
                                                         gym_duckietown.simulator.SAFETY_RAD_MULT,
                                                         self.road_tile_size))

  def add_cone(self, x, y = None):
    if y is None:
      x, y = x[0], x[1]
    obj = _get_obj_props('cone', x, y, True)
    self.objects.append(gym_duckietown.objects.WorldObj(obj, False,
                                                        gym_duckietown.simulator.SAFETY_RAD_MULT))

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

def _get_obj_props(kind, x, y, static = True):
  mesh = gym_duckietown.objmesh.ObjMesh.get(kind)
  return {
    'kind': kind,
    'mesh': mesh,
    'pos': np.array([x, 0, y]),
    'scale': 0.06 / mesh.max_coords[1],
    'y_rot': 0,
    'optional': None,
    'static': static
  }
