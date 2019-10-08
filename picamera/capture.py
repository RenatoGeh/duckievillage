# Captures 96x48 RGB images from the PiCamera.
# Save collection of images into a .npy dataset.

import time
import picamera
import keyboard
import numpy as np

capture = True
L = []

with picamera.PiCamera() as cam:
  cam.resolution = (96, 48)
  while capture:
    print('Capturing')
    time.sleep(0.2)
    p = np.empty((48, 96, 3), dtype=np.uint8)
    cam.capture(p, 'rgb')
    L.append(p)
    if keyboard.is_pressed('q'):
      capture = False
      print('Done')

np.save('/tmp/data.npy', L, allow_pickle=True)

