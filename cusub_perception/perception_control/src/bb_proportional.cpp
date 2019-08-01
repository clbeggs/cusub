#include <perception_control/bb_proportional.h>

namespace perception_control
{
    BBProportional::BBProportional(ros::NodeHandle& nh) : BBController(nh)
    {
        ROS_INFO("loading BB Proportional");
        std::string downcamNamespace = "perception_control/bb_controllers/proportional/downcam";
        nh.getParam(downcamNamespace + "/drive/max_setpoint", downcam_drive_max_setpoint);
        nh.getParam(downcamNamespace + "/drive/maxout_pixel_dist", downcam_drive_maxout_pixel_dist);
        nh.getParam(downcamNamespace + "/strafe/max_setpoint", downcam_strafe_max_setpoint);
        nh.getParam(downcamNamespace + "/strafe/maxout_pixel_dist", downcam_strafe_maxout_pixel_dist);
    }

    void BBProportional::respondDowncamDrive(float diff)
    {
        diff = - diff;
        std_msgs::Float64 driveSet;
        float drive_slope = downcam_drive_max_setpoint / ( (float)downcam_drive_maxout_pixel_dist );
        driveSet.data = drive_slope * diff;
        if (abs(driveSet.data) > downcam_drive_max_setpoint) // check if we've exceeded our max setpoint range
        {
            driveSet.data = driveSet.data > 0 ? downcam_drive_max_setpoint : -downcam_drive_max_setpoint;
        }
        driveSet.data += driveState;
        drivePub.publish(driveSet);
    }

    void BBProportional::respondDowncamStrafe(float diff)
    {
        std_msgs::Float64 strafeSet;
        float strafe_slope = downcam_strafe_max_setpoint / ( (float)downcam_strafe_maxout_pixel_dist );
        strafeSet.data = strafe_slope * diff;
        if (abs(strafeSet.data) > downcam_strafe_max_setpoint) // check if we've exceeded our max setpoint range
        {
            strafeSet.data = strafeSet.data > 0 ? downcam_strafe_max_setpoint : -downcam_strafe_max_setpoint;
        }
        strafeSet.data += strafeState;
        strafePub.publish(strafeSet);
    }

    void BBProportional::respondDowncamYaw(float diff)
    {
        ROS_INFO("Not yet implemented");
    }
    void BBProportional::respondDowncamDepth(float diff)
    {
        ROS_INFO("Not yet implemented");
    }
    void BBProportional::respondOccamDrive(float diff)
    {
        ROS_INFO("Not yet implemented");
    }
    void BBProportional::respondOccamStrafe(float diff)
    {
        ROS_INFO("Not yet implemented");
    }
    void BBProportional::respondOccamYaw(float diff)
    {
        ROS_INFO("Not yet implemented");
    }
    void BBProportional::respondOccamDepth(float diff)
    {
        ROS_INFO("Not yet implemented");
    }
}