#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

/**
 * This node is our camera node. We will be using it for retrieving raw input from our D-435 camera, applying some machine-learned algorithm to said input and then
 * feeding the processed data as an output.
 */
class ImageRawSubscriber{

  ros::NodeHandle n;

  /**
   * This method subscribes to the /camera/color/image_raw topic 
   */
  public:
    void subscribe(){
      ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 1000, &ImageRawSubscriber::callbackImagePtr, this);

      ros::spin();
    }

  /**
   * Callback method for subscriber
   */
  void callbackImagePtr(const sensor_msgs::ImageConstPtr& msg){
    //TODO stuff here with the data received.(the data is a stream of uint8[] data)
    ROS_INFO("I heard: [%ud]", msg->data);
  }
  
};

int main(int argc, char **argv){
  ros::init(argc, argv, "robovision_robot");
  ImageRawSubscriber imgRawSubscriber;
  imgRawSubscriber.subscribe();

  return 0;
}