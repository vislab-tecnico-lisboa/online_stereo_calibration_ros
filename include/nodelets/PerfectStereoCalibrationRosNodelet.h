#ifndef PERFECTSTEREOCALIBRATIONROSNODELET_H
#define PERFECTSTEREOCALIBRATIONROSNODELET_H

#include <nodelet/nodelet.h>
#include "PerfectStereoCalibrationRos.h"

namespace online_stereo_calibration_ros
{
class PerfectStereoCalibrationRosNodelet: public nodelet::Nodelet
{
public:
    PerfectStereoCalibrationRosNodelet(){}
    ~PerfectStereoCalibrationRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<PerfectStereoCalibrationRos> inst_;
};
}

#endif // PERFECTSTEREOCALIBRATIONROSNODELET_H
