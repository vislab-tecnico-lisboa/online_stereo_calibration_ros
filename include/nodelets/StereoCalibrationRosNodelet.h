#ifndef STEREOCALIBRATIONROSNODELET_H
#define STEREOCALIBRATIONROSNODELET_H

#include <nodelet/nodelet.h>
#include "StereoCalibrationRos.h"

namespace online_stereo_calibration_ros
{
class StereoCalibrationRosNodelet: public nodelet::Nodelet
{
public:
    StereoCalibrationRosNodelet(){}
    ~StereoCalibrationRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<StereoCalibrationRos> inst_;
};
}


#endif // STEREOCALIBRATIONROSNODELET_H
