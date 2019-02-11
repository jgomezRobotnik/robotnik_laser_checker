#!/usr/bin/env python
# coding=utf-8

import rospy
import time
from sensor_msgs.msg import LaserScan

DEFAULT_FREQ = 40.0 # Hz, 0.025 s

rospy.init_node('laser_checker')

tini = time.time()
tlast = time.time()

noKeepRateMessage = [] # (time stamp, actual freq)
emptyMessage = [] # time stamp

# Create subscriber callback
def callback(msg):

  global tini, tlast, log, noKeepRateMessage, emptyMessage

  # Time since last message
  tact = time.time()
  tfreq = tact - tlast
  tactfilt = tact - tini
  tlast = time.time()

  # If node is initialised
  if (tactfilt > 2):

    # Save the values in a txt
    log = open("laser_checker.log", "a") # Append the new values

    # If frequency is too slow
    if tfreq > 1.5*(1 / DEFAULT_FREQ): # In seconds
      noKeepRateMessage.append((tactfilt, 1/tfreq))
      log.write(str(tactfilt)+":  "+str(1/tfreq)+" Hz (too slow frequency)\n")

    # If empty message
    if len(msg.ranges) == 0:
      emptyMessage.append(tactfilt)
      log.write(str(tactfilt)+":  EMPTY MESSAGE ERROR\n")

    log.close()

  print(noKeepRateMessage)
  print("-------")
  print(emptyMessage)
  print("***********************")


sub = rospy.Subscriber('/rb1_base/front_laser/scan', LaserScan, callback)
rospy.spin()