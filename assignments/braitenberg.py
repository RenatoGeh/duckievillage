# Intro to Robotics - MAC0318
#
# Name:
# NUSP:
#
# ---
#
# Assignment 2 - Braitenberg vehicles
# Carefully read this header and follow submission instructions!
# Failure to follow instructions may result in a zero!
#
# Task:
#  - Write the following Braitenberg behaviors by only looking at the bot's light sensors:
#     Aggresive: Follows Duckies;
#     Coward:    Hides from Duckies;
#     Lover:     Stares at Duckies at a short distance;
#     Stalker:   Stalks Duckies from a somewhat long distance.
#
# Don't forget to run this from the Duckievillage root directory!
# From within the root directory, run python with the -m flag.
#   python3 -m assignments.manual
#
# Submission instructions:
#  0. Add your name and USP number to the header's header.
#  1. Make sure everything is running fine and there are no errors during startup. If the code does
#     not even start the environment, you will receive a zero.
#  2. Test your code and make sure it's doing what it's supposed to do.
#  3. Copy this file to ../solutions/
#  4. Push changes to your fork. You will be evaluated from what's in your repository!

import sys
import math
import pyglet
import numpy as np
from pyglet.window import key
import gym
import gym_duckietown
from duckievillage import DuckievillageEnv
from PIL import Image
from . import utils

# We'll use our version of Duckietown: Duckievillage. This environment will be where we'll run most
# our tasks in.
env = DuckievillageEnv(
  seed = 101,
  map_name = './maps/nothing.yaml',
  draw_curve = False,
  draw_bbox = False,
  domain_rand = False,
  color_sky = [0, 0, 0],
  user_tile_start = (0, 0),
  distortion = False,
  top_down = False,
  cam_height = 10,
  is_external_map = True,
  light_sensors = 4,
  light_priority = 'h',
)
for o in env.objects: o.scale = 0.085

behavior = sys.argv[1]

angle = env.unwrapped.cam_angle[0]

env.start_pose = [np.array([0.5, 0, 0.5]), 150]
env.reset()
# This function is used to draw the environment to a graphical user interface using Pyglet.
env.render()

def explore(action):
  pass

top_threshold = 0.09
bot_threshold = 0.24

def aggressive(action, sensors):
  pass

def coward(action, sensors):
  pass

def lover(action, sensors):
  pass

def stalker(action, sensors):
  pass

behaviors = {f.__name__: f for f in [aggressive, coward, lover, stalker]}

# We use this function for on-press key events (not something we use for real-time feedback,
# though). We'll register ESC as our way out of the Matrix.
@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
  if symbol == key.ESCAPE:
    env.close()
    sys.exit(0)
  if symbol == key.F:
    env.next_view()
  elif symbol == key.PAGEUP:
    env.unwrapped.cam_angle[0] = 0
  elif symbol == key.PAGEDOWN:
    env.unwrapped.cam_angle[0] = angle

  env.render()

# KeyStateHandler handles key states.
key_handler = key.KeyStateHandler()
# Let's register our key handler to the environment's key listener.
env.unwrapped.window.push_handlers(key_handler)

# This function handles every frame update. Parameter dt is the elapsed time, in milliseconds,
# since the last update call.
def update(dt):
  # At each step, the agent accepts an action array (also accepts numpy.arrays):
  #   action = [forward_velocity, steering_angle]
  action = [0.0, 0.0]

  ls = env.lightsensor.measure()
  behaviors[behavior](action, ls)

  if key_handler[key.W]:
    action += np.array([0.5, 0.0])
  if key_handler[key.A]:
    action += np.array([0.0, 1.0])
  if key_handler[key.S]:
    action += np.array([-0.5, 0.0])
  if key_handler[key.D]:
    action += np.array([0.0, -1.0])

  # At each step, the environment may (or may not) change given your actions. Function step takes
  # as parameter the array action and returns an observation (what the robot is currently
  # seeing), a reward (mostly used for reinforcement learning), whether the episode is done (also
  # used for reinforcement learning) and some info on the elapsed episode.
  # Let's ignore return values for now.
  obs, reward, done, info = env.step(action)

  # Refresh at every update.
  env.render()

# Let's call update every 1.0 / frame_rate second.
pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

# Enter main event loop.
pyglet.app.run()

env.close()
