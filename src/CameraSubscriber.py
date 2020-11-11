#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

#image_converter class: holds methods that subscribe to topic, process data and publish to topic
#subscribe()              : subscribes to the topic and holds the callback method.
#publish(processedData)   : publish processData to a topic
class image_converter:
  def __init__(self):
      self.subscribe()

  def subscribe(self):
      self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

  def callback(self, data):
      #TODO: process the received data
      print("this is the data received", data)

      #After the process is finished, publish the data
      self.publish("Finished proccessing")

  def publish(self, processedData):
      data_pub = rospy.Publisher('topicName', String,queue_size=10)
      rate = rospy.Rate(10)
      while not rospy.is_shutdown():
          rospy.loginfo(processedData)
          data_pub.publish(processedData)
          rate.sleep()



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