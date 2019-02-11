#!/usr/bin/env python
# coding=utf-8

import rospy
import rospkg
import os
import time
import datetime
from sensor_msgs.msg import LaserScan

DEFAULT_FREQ = 40.0 # Hz, 0.025 s

rospy.init_node('laser_checker')

tact = rospy.Time(0)
tini = rospy.Time.now()
tlast = rospy.Time(0)

rp = rospkg.RosPack()

folder_path= os.path.join(rp.get_path('robotnik_laser_checker'), 'logs')

noKeepRateMessage = [] # (time stamp, actual freq)
emptyMessage = [] # time stamp

# Save the values in a txt
log_file = open(folder_path+"/laser_checker.log", "a+") # Append the new values
log_file.write("#"+str(datetime.datetime.now())+"\n")

# Create subscriber callback
def callback(msg):

  global tini, tlast, log, noKeepRateMessage, emptyMessage, folder_path

  # Time since last message
  tact = msg.header.stamp #time.time()
  tfreq = (tact - tlast).to_sec()
  tactfilt = tact - tini

  tlast = msg.header.stamp

  # If node is initialised
  if (tactfilt.to_sec() > 2):

    # If frequency is too slow
    if tfreq > 1.5*(1 / DEFAULT_FREQ): # In seconds
      noKeepRateMessage.append((tactfilt, 1/tfreq))
      log_file.write(str(tactfilt)+":  "+str(1/tfreq)+" Hz (too slow frequency)\n")

    # If empty message
    if len(msg.ranges) == 0:
      emptyMessage.append(tactfilt)
      log_file.write(str(tactfilt)+":  EMPTY MESSAGE ERROR\n")


  print(noKeepRateMessage)
  print("-------")
  print(emptyMessage)
  print("***********************")


sub = rospy.Subscriber('/rb1_base/front_laser/scan', LaserScan, callback)
rospy.spin()

log_file.close()