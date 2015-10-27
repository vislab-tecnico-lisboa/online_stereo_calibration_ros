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
}

void PerfectStereoCalibrationRos::streamTF()
{
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
    //std::cout << r_l_eye_transform.getOrigin().getX() << " " << r_l_eye_transform.getOrigin().getY() << " " <<r_l_eye_transform.getOrigin().getZ() << std::endl;

    geometry_msgs::TransformStamped left_to_center_tf_msg;
    tf::transformStampedTFToMsg (l_eye_transform, left_to_center_tf_msg);
    left_to_center_pub.publish(left_to_center_tf_msg);
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
        stereo_ros.streamTF();
        r.sleep();
    }

    return 0;
}

