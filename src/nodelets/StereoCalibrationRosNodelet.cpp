#include "nodelets/StereoCalibrationRosNodelet.h"

namespace online_stereo_calibration_ros
{
    void StereoCalibrationRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new StereoCalibrationRos(getNodeHandle(), getPrivateNodeHandle()));
    }
}
// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(online_stereo_calibration_ros::StereoCalibrationRosNodelet,nodelet::Nodelet)
