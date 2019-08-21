# Intro to Robotics - MAC0318
#
# Name:
# NUSP:
#
# ---
#
# Assignment 4 - "Catch"
# Carefully read this header and follow submission instructions!
# Failure to follow instructions may result in a zero!
#
# Task:
#  - Implement the potential field algorithm to play catch with Mr. Duckie.
#
# Don't forget that you can (and should!) read the Duckievillage code in search of anything that
# can be useful for your work.
#
# Don't forget to run this from the Duckievillage root directory!
# From within the root directory, run python with the -m flag.
#   python3 -m assignments.potential
#
# Submission instructions:
#  0. Add your name and USP number to the header's header.
#  1. Make sure everything is running fine and there are no errors during startup. If the code does
#     not even start the environment, you will receive a zero.
#  2. Test your code and make sure it's doing what it's supposed to do.
#  3. Append your NUSP to this file name.
#  4. Submit your work to PACA.

import math
import pyglet
from pyglet.window import key
from pyglet.window import mouse
import numpy as np
import gym
import gym_duckietown
import duckievillage
from duckievillage import DuckievillageEnv

env = DuckievillageEnv(
  seed = 101,
  map_name = './maps/catch.yaml',
  draw_curve = False,
  draw_bbox = False,
  domain_rand = False,
  distortion = False,
  top_down = False
)
# Mr. Duckie ready for duty.
def go_mr_duckie():
  import random
  possible_starts = [[0.88, 0.86], [3.85, 0.89], [3.90, 2.68], [0.80, 3.26]]
  p = possible_starts[random.randint(0, len(possible_starts)-1)]
  return env.add_walking_duckie(p)
mr_duckie = go_mr_duckie()

# This returns Mr. Duckie's position.
def mr_duckie_pos():
  return np.delete(mr_duckie.pos, 1)

# Dilate polygon map by 0.1.
env.poly_map.dilate(0.1, True)

env.reset()
env.render()

@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
  if symbol == key.ESCAPE:
    env.close()
    sys.exit(0)
  env.render()

key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

# Parameters. Tweak at will.
# Constant for attraction.
k_att = 10
# Constant for repulsion.
k_rep = 100
# Distance for computation.
rho = 4

# Returns distance between two points p and q.
def dist(p, q):
  raise NotImplemented

# Returns value of a line equation given parameters a and b. Parameter x is the line equation's
# parameter.
def line(a, b, x):
  raise NotImplemented

# Returns distance between a point p and a polygon object o. The polygon object o is a list of
# points making up each vertex. For instance, in a rectangle there would be four points
# corresponding to its four corners.
def dist_obj(p, o):
  raise NotImplemented

# Returns attraction force given a current position p and a goal position g.
def F_att(p, g):
  raise NotImplemented

# Returns repulsion force given a current position p and a polygon object o.
def F_rep(p, o):
  raise NotImplemented

# Returns the sine between two vectors p and q.
def sine_vec(p, q):
  raise NotImplemented

# Returns an action corresponding to the power given to the engines and steering angle.
def force():
  from numpy.linalg import norm
  # Current bot position.
  p = env.get_position()
  # Mr. Duckie's position.
  g = mr_duckie_pos()
  # Repulsion force.
  f_rep = np.array((0.0, 0.0))
  # For each polygon object mapped by PolygonMap, add some repulsion force.
  for o in env.poly_map.polygons():
    f_rep += F_rep(p, o)
  # Attraction force.
  f_att = F_att(p, g)
  # This is the resulting force.
  f = f_att + f_rep
  # Clips the norm-1 of f between -0.2 and 0.2. The steering angle is given by the opposite
  # direction of the sine of the angle between f and the current direction of the bot.
  return (np.clip(norm(f, ord=1), -0.2, 0.2), -sine_vec(np.delete(env.get_dir_vec(), 1), f))

def update(dt):
  action = np.array([0.0, 0.0])

  # This is where you'll write the Duckie's logic.
  # You can fetch your duckiebot's position with env.get_position().
  action = force()

  obs, reward, done, info = env.step(action)

  env.render()
  # Render boundaries for better visualization.
  env.poly_map.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

pyglet.app.run()

env.close()
