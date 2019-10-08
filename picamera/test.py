# Test PiCamera, previewing display and saving test image to /tmp.

import time
import picamera

cam = picamera.PiCamera()
cam.resolution = (320, 240)
cam.start_preview()
time.sleep(2)
cam.capture('/tmp/test.jpg')
