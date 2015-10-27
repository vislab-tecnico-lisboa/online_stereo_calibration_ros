#ifndef PerfectStereoCalibrationRos_H
#define PerfectStereoCalibrationRos_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <spherical_multiple_filter_stereo_calib_lib.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <tf/transform_datatypes.h>
using namespace cv;
class PerfectStereoCalibrationRos
{
    std::string left_camera_frame;
    std::string right_camera_frame;
    std::string ego_frame;

    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle;
    boost::shared_ptr<tf::TransformListener> listener;

    ros::Publisher left_to_right_pub;
    ros::Publisher left_to_center_pub;
    tf::StampedTransform l_eye_transform;
    tf::StampedTransform r_l_eye_transform;
public:

    PerfectStereoCalibrationRos();

    PerfectStereoCalibrationRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_);

    void streamTF();
};

#endif // PerfectStereoCalibrationRos_H
