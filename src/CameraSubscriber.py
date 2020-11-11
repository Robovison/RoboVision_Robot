#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

#image_converter subscribes to the topic and holds the callback method.
class image_converter:
  def __init__(self):
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

  def callback(self, data):
      #TODO: process the received data
      print("this is the data received", data)

#main function holds the initializations for the node and image_converter 
def main(args):
  image_converter()
  rospy.init_node('image_converter', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down, you pressed ctrl - c")
  

if __name__ == '__main__':
    main(sys.argv)