#include "nodelets/PerfectStereoCalibrationRosNodelet.h"

namespace online_stereo_calibration_ros
{
    void PerfectStereoCalibrationRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new PerfectStereoCalibrationRos(getNodeHandle(), getPrivateNodeHandle()));
    }
}
// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(online_stereo_calibration_ros::PerfectStereoCalibrationRosNodelet,nodelet::Nodelet)
