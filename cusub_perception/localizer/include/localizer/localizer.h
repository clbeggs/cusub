#ifndef LOCALIZER_CLASS_SRC_LOCALIZER_CLASS_H_
#define LOCALIZER_CLASS_SRC_LOCALIZER_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <vector>
#include <map>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <localizer/pose_generator.h>
#include <localizer/start_gate_hough.h>
#include <localizer/ignore_pg.h>
#include <localizer/jiangshi_watershed.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace localizer_ns
{
  class Localizer : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    void loadRosParams(ros::NodeHandle& nh);
    void darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs);
    bool checkBox(const darknet_ros_msgs::BoundingBox& bb);
    map<string, pose_generator::PoseGenerator*> mappings;
    ros::Publisher pub;
    ros::Subscriber sub;
    int detection_num;
  };

  namespace pose_gen_decls
  {
    // Pose Generators Declarations
    pose_generator::StartGateHough sgh;
    pose_generator::IgnorePG ignore_pg;
    pose_generator::JiangshiWatershed jw;
  }
  // Pose Generator Mappings
  map<string, pose_generator::PoseGenerator*> sel_mappings =  {
    { "hough", &pose_gen_decls::sgh},
    {"bouy_pnp", &pose_gen_decls::sgh},
    {"ignore", &pose_gen_decls::ignore_pg},
    {"jiangshi_watershed", &pose_gen_decls::jw}
  };
  
}

#endif

/*  Steps to creating a New Pose Generator

1) Create header file in includes/localizer/, subclass PoseGenerator
2) Create source file in src/
3) Instaniate class above in this file in namespace pose_gen_decls
4) Add its corresponding string for the config file in sel_mappings above
5) In CMakeLists.txt, add the src file path to add_library(localizer ...)
6) Add the test file in unit_tests/
7) In CMakeLists.txt, add the executable and link targets for the unit tester

 */