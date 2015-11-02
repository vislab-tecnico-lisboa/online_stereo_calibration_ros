#include "PerfectStereoCalibrationRos.h"

PerfectStereoCalibrationRos::PerfectStereoCalibrationRos()
{
}

PerfectStereoCalibrationRos::PerfectStereoCalibrationRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
    nh(nh_),
    private_node_handle(private_node_handle_),
    listener(new tf::TransformListener(ros::Duration(5.0)))

{
    private_node_handle.param<std::string>("ego_frame", ego_frame, "ego_frame");
    private_node_handle.param<std::string>("left_camera_frame", left_camera_frame, "left_camera_frame");
    private_node_handle.param<std::string>("right_camera_frame", right_camera_frame, "right_camera_frame");

    ROS_INFO_STREAM("ego_frame: "<<ego_frame);
    ROS_INFO_STREAM("left_camera_frame: "<<left_camera_frame);
    ROS_INFO_STREAM("right_camera_frame: "<<right_camera_frame);

    left_to_right_pub=nh.advertise<geometry_msgs::TransformStamped>("left_to_right_tf", 1);
    left_to_center_pub=nh.advertise<geometry_msgs::TransformStamped>("left_to_center_tf", 1);
    stereo_left_camera_info_pub=nh.advertise<sensor_msgs::CameraInfo>("left_stereo_camera_info", 1);
    stereo_right_camera_info_pub=nh.advertise<sensor_msgs::CameraInfo>("right_stereo_camera_info", 1);

    left_camera_info_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > (new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, "left_camera_info", 10));
    right_camera_info_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > (new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, "right_camera_info", 10));
    sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_camera_info_sub, *right_camera_info_sub));
    sync->registerCallback(boost::bind(&PerfectStereoCalibrationRos::callback, this, _1, _2));
}

void PerfectStereoCalibrationRos::callback(const sensor_msgs::CameraInfoConstPtr & left_camera_info_msg,const sensor_msgs::CameraInfoConstPtr & right_camera_info_msg)
{
    //set the cameras intrinsic parameters
    cv::Mat left_cam_intrinsic = cv::Mat::eye(3,3,CV_64F);
    left_cam_intrinsic.at<double>(0,0) = left_camera_info_msg->K.at(0);
    left_cam_intrinsic.at<double>(1,1) = left_camera_info_msg->K.at(4);
    left_cam_intrinsic.at<double>(0,2) = left_camera_info_msg->K.at(2);
    left_cam_intrinsic.at<double>(1,2) = left_camera_info_msg->K.at(5);

    cv::Mat right_cam_intrinsic = cv::Mat::eye(3,3,CV_64F);
    right_cam_intrinsic.at<double>(0,0) = right_camera_info_msg->K.at(0);
    right_cam_intrinsic.at<double>(1,1) = right_camera_info_msg->K.at(4);
    right_cam_intrinsic.at<double>(0,2) = right_camera_info_msg->K.at(2);
    right_cam_intrinsic.at<double>(1,2) = right_camera_info_msg->K.at(5);

    while(ros::ok())
    {
        try
        {
            listener->waitForTransform(ego_frame, left_camera_frame, ros::Time(0), ros::Duration(10.0));
            listener->lookupTransform (ego_frame, left_camera_frame, ros::Time(0), l_eye_transform);
            listener->waitForTransform(right_camera_frame, left_camera_frame, ros::Time(0), ros::Duration(10.0));
            listener->lookupTransform (right_camera_frame, left_camera_frame, ros::Time(0), r_l_eye_transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }

    geometry_msgs::TransformStamped left_to_right_tf_msg;
    tf::transformStampedTFToMsg (r_l_eye_transform, left_to_right_tf_msg);
    left_to_right_pub.publish(left_to_right_tf_msg);

    geometry_msgs::TransformStamped left_to_center_tf_msg;
    tf::transformStampedTFToMsg (l_eye_transform, left_to_center_tf_msg);
    left_to_center_pub.publish(left_to_center_tf_msg);

    cv::Mat R_left_cam_to_right_cam=cv::Mat(3,3,CV_64F);
    R_left_cam_to_right_cam.at<double>(0,0)=r_l_eye_transform.getBasis().getColumn(0)[0];
    R_left_cam_to_right_cam.at<double>(1,0)=r_l_eye_transform.getBasis().getColumn(0)[1];
    R_left_cam_to_right_cam.at<double>(2,0)=r_l_eye_transform.getBasis().getColumn(0)[2];
    R_left_cam_to_right_cam.at<double>(0,1)=r_l_eye_transform.getBasis().getColumn(1)[0];
    R_left_cam_to_right_cam.at<double>(1,1)=r_l_eye_transform.getBasis().getColumn(1)[1];
    R_left_cam_to_right_cam.at<double>(2,1)=r_l_eye_transform.getBasis().getColumn(1)[2];
    R_left_cam_to_right_cam.at<double>(0,2)=r_l_eye_transform.getBasis().getColumn(2)[0];
    R_left_cam_to_right_cam.at<double>(1,2)=r_l_eye_transform.getBasis().getColumn(2)[1];
    R_left_cam_to_right_cam.at<double>(2,2)=r_l_eye_transform.getBasis().getColumn(2)[2];

    cv::Mat t_left_cam_to_right_cam=cv::Mat(3,1,CV_64F);
    t_left_cam_to_right_cam.at<double>(0,0) = r_l_eye_transform.getOrigin()[0];
    t_left_cam_to_right_cam.at<double>(1,0) = r_l_eye_transform.getOrigin()[1];
    t_left_cam_to_right_cam.at<double>(2,0) = r_l_eye_transform.getOrigin()[2];

    cv::Mat R1,R2,P1,P2,Q;
    cv::stereoRectify(left_cam_intrinsic,
                      cv::Mat::zeros(5,1,CV_64F),
                      right_cam_intrinsic,
                      cv::Mat::zeros(5,1,CV_64F),
                      cv::Size(left_camera_info_msg->width,
                               left_camera_info_msg->height),
                      R_left_cam_to_right_cam,
                      t_left_cam_to_right_cam,
                      R1,
                      R2,
                      P1,
                      P2,
                      Q,
                      CV_CALIB_ZERO_DISPARITY,
                      0,
                      cv::Size(left_camera_info_msg->width,
                               left_camera_info_msg->height));


 //   std::cout << "p1: "<<P1 << std::endl;
//    std::cout << "p2: "<< P2 << std::endl;

    sensor_msgs::CameraInfo left_stereo_camera_info;
    left_stereo_camera_info.header=left_camera_info_msg->header;
    left_stereo_camera_info.K=left_camera_info_msg->K;
    left_stereo_camera_info.R.at(0)=R1.at<double>(0,0);
    left_stereo_camera_info.R.at(1)=R1.at<double>(0,1);
    left_stereo_camera_info.R.at(2)=R1.at<double>(0,2);
    left_stereo_camera_info.R.at(3)=R1.at<double>(1,0);
    left_stereo_camera_info.R.at(4)=R1.at<double>(1,1);
    left_stereo_camera_info.R.at(5)=R1.at<double>(1,2);
    left_stereo_camera_info.R.at(6)=R1.at<double>(2,0);
    left_stereo_camera_info.R.at(7)=R1.at<double>(2,1);
    left_stereo_camera_info.R.at(8)=R1.at<double>(2,2);

    left_stereo_camera_info.P.at(0)=P1.at<double>(0,0);
    left_stereo_camera_info.P.at(1)=P1.at<double>(0,1);
    left_stereo_camera_info.P.at(2)=P1.at<double>(0,2);
    left_stereo_camera_info.P.at(3)=P1.at<double>(0,3);
    left_stereo_camera_info.P.at(4)=P1.at<double>(1,0);
    left_stereo_camera_info.P.at(5)=P1.at<double>(1,1);
    left_stereo_camera_info.P.at(6)=P1.at<double>(1,2);
    left_stereo_camera_info.P.at(7)=P1.at<double>(1,3);
    left_stereo_camera_info.P.at(8)=P1.at<double>(2,0);
    left_stereo_camera_info.P.at(9)=P1.at<double>(2,1);
    left_stereo_camera_info.P.at(10)=P1.at<double>(2,2);
    left_stereo_camera_info.P.at(11)=P1.at<double>(2,3);

    sensor_msgs::CameraInfo right_stereo_camera_info;
    right_stereo_camera_info.header=right_camera_info_msg->header;

    right_stereo_camera_info.K=right_camera_info_msg->K;
    right_stereo_camera_info.R.at(0)=R2.at<double>(0,0);
    right_stereo_camera_info.R.at(1)=R2.at<double>(0,1);
    right_stereo_camera_info.R.at(2)=R2.at<double>(0,2);
    right_stereo_camera_info.R.at(3)=R2.at<double>(1,0);
    right_stereo_camera_info.R.at(4)=R2.at<double>(1,1);
    right_stereo_camera_info.R.at(5)=R2.at<double>(1,2);
    right_stereo_camera_info.R.at(6)=R2.at<double>(2,0);
    right_stereo_camera_info.R.at(7)=R2.at<double>(2,1);
    right_stereo_camera_info.R.at(8)=R2.at<double>(2,2);

    right_stereo_camera_info.P.at(0)=P2.at<double>(0,0);
    right_stereo_camera_info.P.at(1)=P2.at<double>(0,1);
    right_stereo_camera_info.P.at(2)=P2.at<double>(0,2);
    right_stereo_camera_info.P.at(3)=P2.at<double>(0,3);
    right_stereo_camera_info.P.at(4)=P2.at<double>(1,0);
    right_stereo_camera_info.P.at(5)=P2.at<double>(1,1);
    right_stereo_camera_info.P.at(6)=P2.at<double>(1,2);
    right_stereo_camera_info.P.at(7)=P2.at<double>(1,3);
    right_stereo_camera_info.P.at(8)=P2.at<double>(2,0);
    right_stereo_camera_info.P.at(9)=P2.at<double>(2,1);
    right_stereo_camera_info.P.at(10)=P2.at<double>(2,2);
    right_stereo_camera_info.P.at(11)=P2.at<double>(2,3);


    stereo_left_camera_info_pub.publish(left_stereo_camera_info);
    stereo_right_camera_info_pub.publish(right_stereo_camera_info);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_calibration");

    ros::NodeHandle nh;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle("~");

    // Declare variables that can be modified by launch file or command line.
    int rate;

    private_node_handle.param("rate", rate, 100);

    PerfectStereoCalibrationRos stereo_ros(nh,private_node_handle);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

