#include "StereoCalibrationRos.h"

StereoCalibrationRos::StereoCalibrationRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
    nh(nh_),
    private_node_handle(private_node_handle_),
    listener(new tf::TransformListener(ros::Duration(5.0))),
    it(nh_)
{
    ROS_INFO("Initializing...");
    std::string left_camera_info_topic;
    std::string right_camera_info_topic;
    std::string joint_states_topic;
    private_node_handle.param<std::string>("ego_frame", ego_frame, "ego_frame");
    private_node_handle.param<std::string>("left_camera_frame", left_camera_frame, "left_camera_frame");
    private_node_handle.param<std::string>("right_camera_frame", right_camera_frame, "right_camera_frame");


    private_node_handle.param<std::string>("left_camera_info_topic", left_camera_info_topic, "left_camera_info_topic");
    private_node_handle.param<std::string>("right_camera_info_topic", right_camera_info_topic, "right_camera_info_topic");
    private_node_handle.param<std::string>("joint_states_topic", joint_states_topic, "joint_states_topic");
    private_node_handle.param("resize_factor",resize_factor,2.5);

    ROS_INFO_STREAM("ego_frame: "<<ego_frame);
    ROS_INFO_STREAM("left_camera_frame: "<<left_camera_frame);
    ROS_INFO_STREAM("right_camera_frame: "<<right_camera_frame);

    ROS_INFO_STREAM("left_camera_info_topic:"<<left_camera_info_topic);
    ROS_INFO_STREAM("right_camera_info_topic:"<<right_camera_info_topic);
    ROS_INFO_STREAM("joint_states_topic:"<<joint_states_topic);

    ROS_INFO_STREAM("resize_factor: "<<resize_factor);


    sensor_msgs::CameraInfoConstPtr left_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(left_camera_info_topic, ros::Duration(30));
    sensor_msgs::CameraInfoConstPtr right_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(right_camera_info_topic, ros::Duration(30));

    //set the cameras intrinsic parameters
    cv::Mat left_cam_intrinsic = Mat::eye(3,3,CV_64F);
    left_cam_intrinsic.at<double>(0,0) = left_camera_info->K.at(0);
    left_cam_intrinsic.at<double>(1,1) = left_camera_info->K.at(4);
    left_cam_intrinsic.at<double>(0,2) = left_camera_info->K.at(2);
    left_cam_intrinsic.at<double>(1,2) = left_camera_info->K.at(5);

    cv::Mat right_cam_intrinsic = Mat::eye(3,3,CV_64F);
    right_cam_intrinsic.at<double>(0,0) = right_camera_info->K.at(0);
    right_cam_intrinsic.at<double>(1,1) = right_camera_info->K.at(4);
    right_cam_intrinsic.at<double>(0,2) = right_camera_info->K.at(2);
    right_cam_intrinsic.at<double>(1,2) = right_camera_info->K.at(5);
    unsigned int width=(unsigned int)left_camera_info->width;
    unsigned int height=(unsigned int)left_camera_info->height;

    try
    {
        listener->waitForTransform(left_camera_frame, right_camera_frame, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(left_camera_frame, right_camera_frame, ros::Time(0), r_l_eye_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }


    double baseline = (double)r_l_eye_transform.getOrigin().length();

    stereo_calibration=boost::shared_ptr<spherical_multiple_filter_stereo_calib> (new spherical_multiple_filter_stereo_calib(fillStereoCalibParams(width,height,left_cam_intrinsic,right_cam_intrinsic,baseline,resize_factor)));
    left_image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(nh, "left_image", 10));
    right_image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(nh, "right_image", 10));
    joint_state_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::JointState> > (new message_filters::Subscriber<sensor_msgs::JointState>(nh, "joint_states", 10));

    sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_image_sub, *right_image_sub, *joint_state_sub));
    sync->registerCallback(boost::bind(&StereoCalibrationRos::callback, this, _1, _2, _3));

    left_to_right_pub=nh.advertise<geometry_msgs::TransformStamped>("left_to_right_tf", 1);
    left_to_center_pub=nh.advertise<geometry_msgs::TransformStamped>("left_to_center_tf", 1);

    stereo_calib_panel_image_publisher = it.advertise("stereo_calibration_panel_image", 3);

    ROS_INFO("Done.");
}

spherical_multiple_filter_stereo_calib_params StereoCalibrationRos::fillStereoCalibParams(const unsigned int & width, const unsigned int & height, const cv::Mat & left_cam_intrinsic, const cv::Mat & right_cam_intrinsic, const double & baseline, const double & resize_factor)
{
    ROS_INFO("Filling stereo params for online calibration...");

    spherical_multiple_filter_stereo_calib_params params;
    params.baseline = baseline;//in mm

    //set the parameters for the stereo calibration system
    params.left_cam_resx = width/resize_factor;
    params.left_cam_resy = height/resize_factor;
    params.left_cam_cx = left_cam_intrinsic.at<double>(0,2)/resize_factor;
    params.left_cam_cy = left_cam_intrinsic.at<double>(1,2)/resize_factor;
    params.left_cam_fx = left_cam_intrinsic.at<double>(0,0)/resize_factor;
    params.left_cam_fy = left_cam_intrinsic.at<double>(1,1)/resize_factor;
    params.right_cam_resx = width/resize_factor;
    params.right_cam_resy = height/resize_factor;
    params.right_cam_cx = right_cam_intrinsic.at<double>(0,2)/resize_factor;
    params.right_cam_cy = right_cam_intrinsic.at<double>(1,2)/resize_factor;
    params.right_cam_fx = right_cam_intrinsic.at<double>(0,0)/resize_factor;
    params.right_cam_fy = right_cam_intrinsic.at<double>(1,1)/resize_factor;

    ROS_INFO("Done.");

    return params;
}

// VERY IMPORTANT: SUBSCRIBE RECTIFIED IMAGES
void StereoCalibrationRos::callback(const sensor_msgs::ImageConstPtr& left_image,
                                    const sensor_msgs::ImageConstPtr& right_image,
                                    const sensor_msgs::JointStateConstPtr& joint_states)
{
    cv::Mat stereo_encoders = cv::Mat::zeros(6,1,CV_64F);

    int i;
    for(i=0;i<joint_states->name.size();++i)
    {
        // LEFT Y angles
        if(joint_states->name[i]==std::string("l_eye_vergence_joint")||joint_states->name[i]==std::string("l_eye_version_joint"))
        {
            stereo_encoders.at<double>(0,0)+=joint_states->position[i];
            std::cout << "joint_states["<<i<<"]:"<< joint_states->name[i] << std::endl;
        }

        // RIGHT Y angles
        if(joint_states->name[i]==std::string("r_eye_vergence_joint")||joint_states->name[i]==std::string("r_eye_version_joint"))
        {
            stereo_encoders.at<double>(1,0)+=joint_states->position[i];
            std::cout << "joint_states["<<i<<"]:"<< joint_states->name[i] << std::endl;
        }

        // LEFT AND RIGHT TILT angles
        if(joint_states->name[i]==std::string("eyes_tilt_joint"))
        {
            stereo_encoders.at<double>(2,0)=joint_states->position[i];
            stereo_encoders.at<double>(3,0)=joint_states->position[i];
            std::cout << "joint_states["<<i<<"]:"<< joint_states->name[i] << std::endl;
        }


    }

    ROS_INFO("Stereo calibration callback...");
    ros::WallTime startTime = ros::WallTime::now();

    // 1. Get uncalibrated color images
    cv::Mat left_image_mat =cv_bridge::toCvCopy(left_image, "bgr8")->image;
    cv::Mat right_image_mat =cv_bridge::toCvCopy(right_image, "bgr8")->image;


    cv::Mat left_image_resized_mat;
    cv::Mat right_image_resized_mat;

    cv::resize(left_image_mat,
               left_image_resized_mat,
               cv::Size(left_image_mat.cols/resize_factor,left_image_mat.rows/resize_factor));
    cv::resize(right_image_mat,
               right_image_resized_mat,
               cv::Size(right_image_mat.cols/resize_factor,right_image_mat.rows/resize_factor));


    // 3. calibrate given angles
    ROS_INFO("Calibrate stereo...");
    stereo_calibration->calibrate(left_image_resized_mat,
                                  right_image_resized_mat);

    ROS_INFO("Done.");

    //get the calibrated transformations between the two cameras
    spherical_multiple_filter_stereo_calib_data scd;
    scd =  stereo_calibration->get_calibrated_transformations();

    geometry_msgs::TransformStamped right_to_left_transform_stamped;
    right_to_left_transform_stamped.header.stamp=left_image->header.stamp;
    right_to_left_transform_stamped.header.frame_id="r_camera_vision_link";
    right_to_left_transform_stamped.header.stamp=ros::Time::now();
    right_to_left_transform_stamped.child_frame_id="l_camera_vision_link";
    cv::Mat r_right_cam_to_left_cam_quaternion=Quaternion(scd.R_left_cam_to_right_cam);

    right_to_left_transform_stamped.transform.rotation.w=r_right_cam_to_left_cam_quaternion.at<double>(0,0);
    right_to_left_transform_stamped.transform.rotation.x=r_right_cam_to_left_cam_quaternion.at<double>(1,0);
    right_to_left_transform_stamped.transform.rotation.y=r_right_cam_to_left_cam_quaternion.at<double>(2,0);
    right_to_left_transform_stamped.transform.rotation.z=r_right_cam_to_left_cam_quaternion.at<double>(3,0);
    right_to_left_transform_stamped.transform.translation.x=scd.t_left_cam_to_right_cam.at<double>(0,0);
    right_to_left_transform_stamped.transform.translation.y=scd.t_left_cam_to_right_cam.at<double>(1,0);
    right_to_left_transform_stamped.transform.translation.z=scd.t_left_cam_to_right_cam.at<double>(2,0);

    left_to_right_pub.publish(right_to_left_transform_stamped);

    tf::StampedTransform l_center_transform;

    try
    {
        listener->waitForTransform(ego_frame, left_camera_frame, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(ego_frame, left_camera_frame,
                                 ros::Time(0), l_center_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    geometry_msgs::TransformStamped l_center_msg;
    tf::transformStampedTFToMsg (l_center_transform, l_center_msg);
    left_to_center_pub.publish(l_center_msg);

    /*left_to_center_transform_stamped.header=left_image->header;
    left_to_center_transform_stamped.header.stamp=ros::Time::now();
    left_to_center_transform_stamped.child_frame_id="eyes_center_link";
    cv::Mat r_left_cam_to_center_cam_quaternion=Quaternion(scd.transformation_left_cam_to_baseline_center(cv::Range(0,3),cv::Range(0,3)));

    left_to_center_transform_stamped.transform.rotation.w=r_left_cam_to_center_cam_quaternion.at<double>(0,0);
    left_to_center_transform_stamped.transform.rotation.x=r_left_cam_to_center_cam_quaternion.at<double>(1,0);
    left_to_center_transform_stamped.transform.rotation.y=r_left_cam_to_center_cam_quaternion.at<double>(2,0);
    left_to_center_transform_stamped.transform.rotation.z=r_left_cam_to_center_cam_quaternion.at<double>(3,0);
    left_to_center_transform_stamped.transform.translation.x=scd.transformation_left_cam_to_baseline_center.at<double>(0,0);
    left_to_center_transform_stamped.transform.translation.y=scd.transformation_left_cam_to_baseline_center.at<double>(1,0);
    left_to_center_transform_stamped.transform.translation.z=scd.transformation_left_cam_to_baseline_center.at<double>(2,0);

    left_to_center_pub.publish(left_to_center_transform_stamped);*/

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();


    cv::Mat calib_panel_image=EyesStereoModel(scd.ty,scd.tz,scd.rx,scd.ry,scd.rz);

    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", calib_panel_image).toImageMsg();

    stereo_calib_panel_image_publisher.publish(image_msg);
    ROS_INFO(" TOTAL TIME STEREO CALIBRATION:  %f sec", total_elapsed);
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
    ROS_INFO_STREAM("rate:"<< rate);
    StereoCalibrationRos stereo_ros(nh,private_node_handle);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end
