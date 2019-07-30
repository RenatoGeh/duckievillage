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

# A topological graph of a Duckietown map.
class TopoGraph:
  def __init__(self):
    self._L = {}

  def add_node(self, p):
    if p not in self._L:
      self._L[p] = {}

  def add_edge(self, p, q):
    self._L[p][q] = True
    self._L[q][p] = True

  # A-star
  def path(self, p, q):
    if p not in self._L or q not in self._L:
      print("Can't get there! Source or target are not drivable.")
      return None
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
    print("Start: {}, End: {}".format(p, q))
    # We use Manhattan's as heuristics.
    heapq.heappush(P, (0+_manhattan_dist(p, q), p))
    while len(P) != 0:
      f, n = heapq.heappop(P)
      print("  Node: {}, f(n) = {}".format(n, f))
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
          R.insert(0, u)
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

def _create_topo_graph(w: int, h: int, tiles: dict) -> TopoGraph:
  G = TopoGraph()
  M = [[None] * h for i in range(w)]
  for t in tiles:
    i, j = t['coords']
    M[i][j] = t
    G.add_node(t['coords'])
  # We're assuming adjacent drivable tiles are always connected. That's not true for the general
  # case, but let's not worry about this for now.
  for t in tiles:
    i, j = t['coords']
    if i-1 >= 0:
      u = M[i-1][j]
      if u is not None:
        G.add_edge(t['coords'], u['coords'])
    if j-1 >= 0:
      u = M[i][j-1]
      if u is not None:
        G.add_edge(t['coords'], u['coords'])
    if i+1 < w:
      u = M[i+1][j]
      if u is not None:
        G.add_edge(t['coords'], u['coords'])
    if j+1 < h:
      u = M[i][j+1]
      if u is not None:
        G.add_edge(t['coords'], u['coords'])
  return G

class DuckievillageEnv(gym_duckietown.envs.DuckietownEnv):
  top_down = False

  def __init__(self, top_down = False, **kwargs):
    gym_duckietown.envs.DuckietownEnv.__init__(self, **kwargs)
    self.top_down = top_down
    logger = logging.getLogger('gym-duckietown')
    logger.propagate = False
    self.force_reset()
    self.topo_graph = _create_topo_graph(self.grid_width, self.grid_height, self.drivable_tiles)

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
    img = np.concatenate((top, bot), axis=1)

    # self.undistort - for UndistortWrapper
    if self.distortion and not self.undistort and mode != "free_cam":
      img = self.camera_model.distort(img)

    if mode == 'rgb_array':
      return img

    from pyglet import gl, window, image

    if self.window is None:
      config = gl.Config(double_buffer=False)
      self.window = window.Window(
        width=int(2.0*WINDOW_WIDTH),
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

  def force_reset(self):
    gym_duckietown.envs.DuckietownEnv.reset(self)

  # We have to convert from window positions to actual Duckietown coordinates.
  def convert_coords(self, x: int, y: int) -> (float, float):
    # Maps are up to 7 tiles high. Assuming square tiles and invariant tile dimensions:
    h = 7 * self.road_tile_size
    r =  h / WINDOW_HEIGHT
    # Do some mathemagics.
    return x*r - 2*self.road_tile_size/3, h - y*r

  def add_duckie(self, x, y, static = True):
    obj = _get_obj_props('duckie', x, y, static)
    self.objects.append(gym_duckietown.objects.DuckieObj(obj, False,
                                                         gym_duckietown.simulator.SAFETY_RAD_MULT,
                                                         self.road_tile_size))

  def add_cone(self, x, y):
    obj = _get_obj_props('cone', x, y, True)
    self.objects.append(gym_duckietown.objects.WorldObj(obj, False,
                                                        gym_duckietown.simulator.SAFETY_RAD_MULT))


  # This function checks whether we've reached a waypoint (up to some error).
  def arrived(self, x, y = None):
    from numpy.linalg import norm
    v = np.array([x, y]) if y is not None else x
    return norm(v-self.get_position()) < 0.1

  # Returns the angle relative to the Duckiebot's position and a target position.
  def angle_target(self, t):
    # This is the Duckiebot's pose.
    s = np.delete(self.get_dir_vec(), 1)
    # Get the vector representing the "distance" between Duckie and its target.
    t -= self.get_position()
    from numpy.linalg import norm
    # Normalize vectors to unit size.
    u, v = s/norm(s), t/norm(t)
    # Do some basic trig and clip to [-1, 1] range.
    return np.arccos(np.clip(np.dot(u, v), -1, 1))

  # Returns the sine between the Duckiebot's vector direction + position and the target's position.
  # This effectively gives a finer measure on "how good" is our heading towards the target. The
  # sibling function angle_target returns the angle extracted from the cosine of two vectors.
  # Because the cosine is zero from both limits (x -> 0^+ and x -> 0^-), we end up unable to
  # differenciate between coming from "below" or "above". This function takes care of this by
  # returning the sine (which is "zero-sensitive", i.e. limits on zero are not equal) instead of
  # the arccosine.
  def sine_target(self, t):
    s = np.delete(self.get_dir_vec(), 1)
    t -= self.get_position()
    from numpy.linalg import norm
    u, v = s/norm(s), t/norm(t)
    return np.cross(u, v)/(norm(u, ord=1)*norm(v, ord=1))

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
