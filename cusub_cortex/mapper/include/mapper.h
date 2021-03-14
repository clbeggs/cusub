#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Core>
#include <boost/bind.hpp>

class Mapper {
  /* docstring */

 public:
  Mapper();
  ~Mapper();

  void synced_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& bound_box,
                       const sensor_msgs::ImageConstPtr& image);

 private:
  Eigen::MatrixXf particles;
  Eigen::MatrixXf landmarks;
  ros::NodeHandle nh;
};
