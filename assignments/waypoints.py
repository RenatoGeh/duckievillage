# Intro to Robotics - MAC0318
#
# Name:
# NUSP:
#
# ---
#
# Assignment 2 - Waypoint navigation
# Carefully read this header and follow submission instructions!
# Failure to follow instructions may result in a zero!
#
# Task:
#  - Create a waypoint navigating Duckiebot.
#  - Implement navigation by shortest distance.
#    + Given two points, the shortest distance route is a straight line between them.
#  - Implement navigation by axes.
#    + Given two points, navigate _only_ through the x and y axes.
#
# Take a look at the Waypoints class in duckievillage.py. Method arrived in DuckievillageEnv may
# also help you implement waypoint navigation.
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
  map_name = './maps/udem1.yaml',
  draw_curve = False,
  draw_bbox = False,
  domain_rand = False,
  distortion = False,
  top_down = False
)
# Set mark-mode whenever we run waypoints.py with the -m or --mark-waypoints flag.
waypoints = duckievillage.Waypoints('--read-from-file' in sys.argv, env)

env.reset()
env.render()

@env.unwrapped.window.event
def on_key_press(symbol, mods):
  if symbol == key.ESCAPE:
    waypoints.write('waypoints.txt')
    env.close()
    sys.exit(0)
  env.render()

# On mouse press, register waypoint.
@env.unwrapped.window.event
def on_mouse_press(x, y, button, mods):
  if button == mouse.LEFT:
    if x < 0 or x > duckievillage.WINDOW_WIDTH or y < 0 or y > duckievillage.WINDOW_HEIGHT:
      return
    # Convert coordinates from window position to Duckietown coordinates.
    px, py = env.convert_coords(x, y)
    waypoints.mark(px, py, x//2, y)

key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

def update(dt):
  action = [0.0, 0.0]

  # This is where you'll write the Duckie's logic.
  # You can fetch your duckiebot's position with env.get_position().

  obs, reward, done, info = env.step(action)

  env.render()
  waypoints.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

pyglet.app.run()

env.close()
