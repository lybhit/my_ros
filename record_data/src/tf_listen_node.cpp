#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <string>

// class odomTf
// {
// public:
//   odomTf();
//   ~odomTf();

// private:

//   void odomHandler(const nav_msgs::OdometryConstPtr &raw_msg);

//   ros::NodeHandle nh_;
//   ros::Subscriber subOdom_;
//   tf::TransformBroadcaster* tfb_;
//   geometry_msgs::TransformStamped odom_trans_;
// };

// odomTf::odomTf()
// {
//   tfb_ = new tf::TransformBroadcaster();

//   subOdom_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 3, &odomTf::odomHandler, this);
// }

// odomTf::~odomTf()
// {
//   delete tfb_;
// }

// void odomTf::odomHandler(const nav_msgs::OdometryConstPtr &raw_msg)
// {
//   odom_trans_.header.stamp = ros::Time::now();
//   odom_trans_.header.frame_id = "odom";
//   odom_trans_.child_frame_id = "base_link";
//   odom_trans_.transform.translation.x = raw_msg->pose.pose.position.x;
//   odom_trans_.transform.translation.y = raw_msg->pose.pose.position.y;
//   odom_trans_.transform.translation.z = 0.0;
//   odom_trans_.transform.rotation = raw_msg->pose.pose.orientation;

  
//   tfb_->sendTransform(odom_trans_);

// 	// float covariance[36] = {0.001, 0, 0, 0, 0, 0, //covariance on gps_x
//  //                          0, 0.001, 0, 0, 0, 0, //covariance on gps_y
//  //                          0, 0, 9999, 0, 0, 0, //covariance on gps_z
//  //                          0, 0, 0, 9999, 0, 0, //large covariance on rot x
//  //                          0, 0, 0, 0, 9999, 0, //large covariance on rot y
//  //                          0, 0, 0, 0, 0, 0.02};//large covariance on rot z

//  //    nav_msgs::Odometry msg;
//  //    msg = *raw_msg;

//  //    for(int i = 0; i < 36; ++i)
//  //    {
//  //    	msg.pose.covariance[i] = covariance[i];
//  //    }

//  //    pubOdom.publish(msg);

// }

// void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg)
// {
// 	float covariance[9] = {0.001, 0., 0., 0., 0.001, 0.001, 0., 0., 0.001};

// 	sensor_msgs::Imu msg;
// 	msg = *imu_msg;

// 	for(int i = 0; i < 9; ++i)
// 	{
// 		msg.orientation_covariance[i] = covariance[i];
// 		msg.angular_velocity_covariance[i] = covariance[i];
// 		msg.linear_acceleration_covariance[i] = covariance[i];
// 	}

// 	pubImu.publish(msg);

// }



int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf_listen_node");

  // odomTf odom_tf;

  tf::TransformListener tf_;

  ros::Rate r(50);

  std::string odom_frame_id_("odom");
  std::string base_frame_id_("base_link");

  while(ros::ok())
  {
    tf::Stamped<tf::Pose> odom_pose;
    tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                              tf::Vector3(0, 0, 0)), ros::Time(0), base_frame_id_);
    try {
        tf_.transformPose(odom_frame_id_, ident, odom_pose);
    }
    catch (tf::TransformException e) {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double x = odom_pose.getOrigin().x();
    double y = odom_pose.getOrigin().y();
    double yaw,pitch, roll;
    odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

    ROS_INFO("transform pose: %f, %f, %f", x, y, yaw);

    ros::spinOnce();

    r.sleep();
  }

  // ros::spin();

  return 0;
}