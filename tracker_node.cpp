//
// Created by andy on 19-3-7.
//

#include "tracker/cascaded_camera_tracker.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "object_recognition_msgs/RecognizedObject.h"

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using apollo::perception::CascadedCameraTracker;
using apollo::perception::VisualObject;

std::shared_ptr<CascadedCameraTracker> g_tracker;

void chatterCallback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img = cv_ptr->image;
  double timestamp = cv_ptr->header.stamp.toSec();
  std::vector<std::shared_ptr<VisualObject>> vobjects;

  for (int i = 0; i < msg->objects.size(); ++i) {
    auto vobj_ptr = std::make_shared<VisualObject>();
    auto obj = msg->objects[i];
    //vobj->type = obj.type.key;
    //vobj->type_probs = obj.confidence;
    vobj_ptr->upper_left[0] = obj.bounding_contours[0].x;
    vobj_ptr->upper_left[1] = obj.bounding_contours[0].y;
    vobj_ptr->lower_right[0] = obj.bounding_contours[1].x;
    vobj_ptr->lower_right[1] = obj.bounding_contours[1].y;
    std::cout << " type:" << obj.type.key
              << " score:" << obj.confidence
              << " x1y1x2y2:"
              << vobj_ptr->upper_left[0] << ","
              << vobj_ptr->upper_left[1] << ","
              << vobj_ptr->lower_right[0] << ","
              << vobj_ptr->lower_right[1];
    vobjects.push_back(vobj_ptr);
  }


  g_tracker->Associate(img, timestamp, &vobjects);

  for (int j = 0; j < vobjects.size(); ++j) {
    auto obj = vobjects[j];

    cv::rectangle(img, cv::Point(obj->upper_left[0], obj->upper_left[1]),
              cv::Point(obj->lower_right[0], obj->lower_right[1]), cv::Scalar(0, 255, 0), 2);

    std::string id = std::to_string(static_cast<int>(obj->track_id));
    cv::putText(img, id, cv::Point(obj->upper_left[0], obj->upper_left[1]), cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
          1.2, cv::Scalar(0, 0, 255), 3, 8);
  }

  // Update GUI Window
  cv::imshow("test", img);
  cv::waitKey(10);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  g_tracker = std::make_shared<CascadedCameraTracker>();
  g_tracker->Init();

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
}
