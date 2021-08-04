# Intro to Robotics - MAC0318
#
# Name:
# NUSP:
#
# ---
#
# Assignment 8 - Lane following as a regression task
# Carefully read this header and follow submission instructions!
# Failure to follow instructions may result in a zero!
#
# Don't forget to run this from the Duckievillage root directory!
# From within the root directory, run python with the -m flag.
#   python3 -m assignments.regression
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

import cv2
import tensorflow
import tensorflow.keras as keras

env = DuckievillageEnv(
  seed = 101,
  map_name = 'udem1',
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
  elif symbol == key.C:
    capture()
  env.render()

key_handler = key.KeyStateHandler()
env.unwrapped.window.push_handlers(key_handler)

# Loads a Tensorflow model.
def load_model(filepath):
  N = keras.models.load_model(filepath)
  N.summary()
  return N

# Converts an RGB image to grayscale.
def rgb2gray(I):
  return np.dot(I[...,:3], [0.2989, 0.5870, 0.1140])

# Preprocess image to fit our trained model.
def preprocess_image(I):
  J = cv2.resize(I, dsize=(160, 120))
  return rgb2gray(J).astype(np.uint8)

# Captures a single frame of our robot's front camera and applies the required preprocessing.
def capture():
  return preprocess_image(env.front())

# Implement your control system here. It should return an array of the action to be taken.
def control():
  raise NotImplemented

def update(dt):
  action = control()

  obs, reward, done, info = env.step(action)

  env.render()

pyglet.clock.schedule_interval(update, 1.0 / env.unwrapped.frame_rate)

pyglet.app.run()

env.close()
