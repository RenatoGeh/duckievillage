# Intro to Robotics - MAC0318
#
# Name:
# NUSP:
#
# ---
#
# Assignment 5 - 2D Localization
# Carefully read this header and follow submission instructions!
# Failure to follow instructions may result in a zero!
#
# Don't forget to run this from the Duckievillage root directory!
# From within the root directory, run python with the -m flag.
#   python3 -m assignments.localization
#
# Submission instructions:
#  0. Add your name and USP number to the header's header.
#  1. Make sure everything is running fine and there are no errors during startup. If the code does
#     not even start the environment, you will receive a zero.
#  2. Test your code and make sure it's doing what it's supposed to do.
#  3. Append your NUSP to this file name.
#  4. Submit your work to PACA.

import sys
import math
import pyglet
import numpy as np
from pyglet.window import key
import gym
import gym_duckietown
from duckievillage import DuckievillageEnv, FRONT_VIEW_MODE

env = DuckievillageEnv(
  seed = 101,
  map_name = './maps/udem1.yaml',
  draw_curve = False,
  draw_bbox = False,
  domain_rand = False,
  distortion = False,
  top_down = False,
)

env.set_view(FRONT_VIEW_MODE)

env.reset()
env.render()

@env.unwrapped.window.event
def on_key_press(symbol, modifiers):
  if symbol == key.ESCAPE:
    env.close()
    sys.exit(0)
  elif symbol == key.SPACE:
    print(env.road_sensor.predict())
    print(env.roads())
  elif symbol == key.BACKSPACE:
    print(env.odometer.measure())
  elif symbol == key.V:
    env.toggle_single_view()
  env.render()

key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

def update(dt):
  action = np.array([0.0, 0.0])

  if key_handler[key.UP]:
    action += (0.5, 0.0)
  if key_handler[key.LEFT]:
    action += (0.0, 1.0)
  if key_handler[key.DOWN]:
    action += (-0.5, 0.0)
  if key_handler[key.RIGHT]:
    action += (0.0, -1.0)

  obs, reward, done, info = env.step(action)

  env.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

pyglet.app.run()

env.close()
