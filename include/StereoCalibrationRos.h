#ifndef STEREOCALIBRATIONROS_H
#define STEREOCALIBRATIONROS_H

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

using namespace cv;
class StereoCalibrationRos
{
    std::string left_camera_frame;
    std::string right_camera_frame;
    std::string ego_frame;

    tf::StampedTransform r_l_eye_transform;
    image_transport::ImageTransport it;

    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle;
    boost::shared_ptr<tf::TransformListener> listener;
    boost::shared_ptr<spherical_multiple_filter_stereo_calib> stereo_calibration;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::JointState> MySyncPolicy;

    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > left_image_sub;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > right_image_sub;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::JointState> > joint_state_sub;

    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;

    ros::Publisher left_to_right_pub;
    ros::Publisher left_to_center_pub;

    image_transport::Publisher stereo_calib_panel_image_publisher;


public:


    StereoCalibrationRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_);

    spherical_multiple_filter_stereo_calib_params fillStereoCalibParams(const unsigned int & width, const unsigned int & height, const cv::Mat & left_cam_intrinsic, const cv::Mat & right_cam_intrinsic, const double & baseline, const double & resize_factor);


    void callback(const sensor_msgs::ImageConstPtr& left_image,
                  const sensor_msgs::ImageConstPtr& right_image,
                  const sensor_msgs::JointStateConstPtr& joint_states);


    cv::Mat Quaternion(double rx, double ry, double rz){
        //quaternion vec
        Mat Q(4,1,CV_64F);

        //Teta = ||w*dT||
        double teta=sqrt((rx*rx)+(ry*ry)+(rz*rz));


        Mat U(3,1,CV_64F);

        if (teta<1e-6){
            Q.at<double>(0,0)=1;
            Q.at<double>(1,0)=0;
            Q.at<double>(2,0)=0;
            Q.at<double>(3,0)=0;
            return Q;
        }

        U.at<double>(0,0)=(rx)/teta;
        U.at<double>(1,0)=(ry)/teta;
        U.at<double>(2,0)=(rz)/teta;

        Q.at<double>(0,0) = cos(teta/2);
        Q.at<double>(1,0) = sin(teta/2)*U.at<double>(0,0);
        Q.at<double>(2,0) = sin(teta/2)*U.at<double>(1,0);
        Q.at<double>(3,0) = sin(teta/2)*U.at<double>(2,0);

        return Q;
    }

    cv::Mat Quaternion(cv::Mat Rotation_Matrix)
    {
        Mat Q(4,1,CV_64F);

        Q.at<double>(0,0) = sqrt(1 + Rotation_Matrix.at<double>(0,0) + Rotation_Matrix.at<double>(1,1) + Rotation_Matrix.at<double>(2,2))/2;
        Q.at<double>(1,0) = (Rotation_Matrix.at<double>(2,1) - Rotation_Matrix.at<double>(1,2))/(4*Q.at<double>(0,0));
        Q.at<double>(2,0) = (Rotation_Matrix.at<double>(0,2) - Rotation_Matrix.at<double>(2,0))/(4*Q.at<double>(0,0));
        Q.at<double>(3,0) = (Rotation_Matrix.at<double>(1,0) - Rotation_Matrix.at<double>(0,1))/(4*Q.at<double>(0,0));

        return Q;

    }

    cv::Mat Quaternion_to_Euler(cv::Mat Quaternion)
    {

        cv::Mat Euler(3,1,CV_64F);
        Mat R = Rotation_Matrix(Quaternion);

        Euler.at<double>(0,0) = atan(R.at<double>(1,0)/R.at<double>(0,0));
        Euler.at<double>(1,0) = atan( -R.at<double>(2,0) / ( cos(Euler.at<double>(0,0))*R.at<double>(0,0) + sin(Euler.at<double>(0,0))*R.at<double>(1,0)));
        Euler.at<double>(2,0) = atan( ( sin(Euler.at<double>(0,0))*R.at<double>(0,2) - cos(Euler.at<double>(0,0))*R.at<double>(1,2)) / ( cos(Euler.at<double>(0,0))*R.at<double>(1,1) - sin(Euler.at<double>(0,0))*R.at<double>(0,1)));

        return Euler;
    }

    cv::Mat Rotation_Matrix(cv::Mat Quaternion)
    {
        Mat R(3,3,CV_64F);

        double q0 = Quaternion.at<double>(0,0);
        double qx = Quaternion.at<double>(1,0);
        double qy = Quaternion.at<double>(2,0);
        double qz = Quaternion.at<double>(3,0);

        R.at<double>(0,0) = q0*q0 + qx*qx - qy*qy - qz*qz;
        R.at<double>(0,1) = 2*(qx*qy - q0*qz);
        R.at<double>(0,2) = 2*(qz*qx - q0*qy);
        R.at<double>(1,0) = 2*(qx*qy + q0*qz);
        R.at<double>(1,1) = q0*q0 - qx*qx + qy*qy - qz*qz;
        R.at<double>(1,2) = 2*(qy*qz - q0*qx);
        R.at<double>(2,0) = 2*(qz*qx - q0*qy);
        R.at<double>(2,1) = 2*(qy*qz + q0*qx);
        R.at<double>(2,2) = q0*q0 - qx*qx - qy*qy + qz*qz;

        return R;
    }

};

#endif // STEREOCALIBRATIONROS_H
