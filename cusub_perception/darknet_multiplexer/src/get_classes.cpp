#include <darknet_multiplexer/get_classes.h>

namespace darknet_get_classes_ns
{
    void GetClasses::onInit()
    {
        NODELET_INFO("Loading Get Classes Server");
        // Set up the rosservice & subscriber to darknet
        ros::NodeHandle& nh = getMTNodeHandle();
        recording = false;
        // darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &GetClasses::darknetCallback, this);
        darknetSub = nh.subscribe("tracking_boxes", 1, &GetClasses::darknetCallback, this);
        service = nh.advertiseService("cusub_perception/darknet_multiplexer/get_classes", &GetClasses::handle, this);
        // service = nh.advertiseService("cusub_perception/darknet_multiplexer/configure_active_cameras", &Multiplexer::configureActives, this);
    }
    void GetClasses::darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs)
    {
        if ( !recording ) { return; }
        for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
        {
            if (std::find(classes.begin(), classes.end(),box.Class) == classes.end()) // check if the class already exists in the vector
            {
                classes.push_back(box.Class);
            }
        }
    }
    bool GetClasses::handle(darknet_multiplexer::DarknetClasses::Request& request,
                            darknet_multiplexer::DarknetClasses::Response& response)
                            {
                                NODELET_INFO("GetClasses received request");
                                classes.clear();
                                recording = true;
                                request.monitor_time.sleep();
                                recording = false;
                                response.classes = classes;
                                return true;
                            }
}
PLUGINLIB_EXPORT_CLASS(darknet_get_classes_ns::GetClasses, nodelet::Nodelet);