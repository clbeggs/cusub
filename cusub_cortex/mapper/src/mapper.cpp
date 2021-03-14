#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <mapper.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Core>
#include <boost/bind.hpp>
#include <iostream>

Mapper::Mapper() {
  // Setup Eigen Matrices
  this->particles = Eigen::MatrixXf::Zero(100, 3);
  this->landmarks = Eigen::MatrixXf::Zero(100, 3);

  // Setup ROS subscribers
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bound_box(
      this->nh,
      "/leviathan/cusub_perception/darknet_ros/bounding_boxes",
      3);

  message_filters::Subscriber<sensor_msgs::Image> sub_image(
      this->nh,
      "/leviathan/darknet_ros/original_image",
      3);

  // Setup ROS synchronizer
  typedef message_filters::sync_policies::
      ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image>
          DarknetSyncPolicy;

  message_filters::Synchronizer<DarknetSyncPolicy> sync(DarknetSyncPolicy(10),
                                                        sub_bound_box,
                                                        sub_image);

  sync.registerCallback(boost::bind(&Mapper::synced_callback, this, _1, _2));

  // this->sub_bound_box.subscribe(this->nh,
  // "/leviathan/cusub_perception/darknet_ros/bounding_boxes", 3);
  // this->sub_image.subscribe(this->nh,
  // "/leviathan/darknet_ros/original_image", 3); this->sync.reset(new
  // SyncPol(DarknetSyncPolicy(10), sub_image, sub_bound_box));
  // sync.registerCallback(boost::bind(&Mapper::synced_callback, _1, _2));
}

Mapper::~Mapper() {
  this->particles.resize(0, 0);
  this->landmarks.resize(0, 0);
}

void Mapper::synced_callback(
    const darknet_ros_msgs::BoundingBoxesConstPtr& bound_box,
    const sensor_msgs::ImageConstPtr& image) {
  ROS_INFO("Synch. success.");
}

int main() { std::cout << "Hello" << std::endl; }
