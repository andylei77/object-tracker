//
// Created by andy on 19-3-7.
//

#include "tracker/cascaded_camera_tracker.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "object_recognition_msgs/RecognizedObject.h"

//from std_msgs.msg import String
//from object_recognition_msgs.msg import *
//from shape_msgs.msg import *
//from geometry_msgs.msg import *

void chatterCallback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
}
