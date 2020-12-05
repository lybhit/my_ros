#include <fstream>
#include <memory>
#include <cmath>
#include <string>
#include <time.h>
#include <stdlib.h>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "boost/date_time/posix_time/posix_time.hpp"
#include <string.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/QuaternionStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

class ReadPose
{
public:
  ReadPose();
  ~ReadPose();

private:
	void poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & cluster_cloud);
	void gpsCb(const sensor_msgs::NavSatFixConstPtr & gps_msg);
	void sigRev(const std_msgs::StringConstPtr& sig_msg);
	void odomGpsCb(const nav_msgs::OdometryConstPtr & odom_gps_msg);

	ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber subPos_;
  ros::Subscriber subSignal_;
  ros::Subscriber subGps_;
  ros::Subscriber subOdomGps_;

	std::ofstream out_file_1_;
  std::string file_path_1_;

	std::ofstream out_file_2_;
  std::string file_path_2_;

	std::ofstream out_file_3_;
  std::string file_path_3_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string base_frame_id_;

  bool flag_read_1_;

  boost::mutex sig_mutex_;
  int counter_1_;
  int counter_2_;
  int counter_3_;

};

ReadPose::ReadPose():
  flag_read_1_(false),
  counter_1_(0),
  counter_2_(0),
  private_nh_("~")
{
	private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
	private_nh_.param("file_path_1", file_path_1_, std::string("/home/lyb/filtered_points"));
	private_nh_.param("file_path_2", file_path_2_, std::string("/home/lyb/gps_points"));
	private_nh_.param("file_path_3", file_path_3_, std::string("/home/lyb/odom_gps_points"));

  subPos_= nh_.subscribe("pose_with_cov", 2, &ReadPose::poseCb, this);
  subGps_= nh_.subscribe("ub482/bp_fix", 2, &ReadPose::gpsCb, this);
  subSignal_ = nh_.subscribe("signal", 1, &ReadPose::sigRev, this);
  subOdomGps_ = nh_.subscribe("odometry/gps", 2,  &ReadPose::odomGpsCb, this);

	tf_.reset(new tf2_ros::Buffer());

  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  std::string file_suffix = iso_time_str + ".txt";
  std::string file_1 = file_path_1_ + file_suffix;

  std::string file_2 = file_path_2_ + file_suffix;
  std::string file_3 = file_path_3_ + file_suffix;

	const char* p1 = file_1.c_str();
  out_file_1_.open(p1, std::ios::app);

	const char* p2 = file_2.c_str();
  out_file_2_.open(p2, std::ios::app);

	const char* p3 = file_3.c_str();
  out_file_3_.open(p3, std::ios::app);
}

ReadPose::~ReadPose()
{
	out_file_1_.close();
	out_file_2_.close();
	out_file_3_.close();
}

void ReadPose::poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & pos)
{
  boost::mutex::scoped_lock lr(sig_mutex_);
  ROS_INFO_ONCE("subscribe pose_with_cov.");

  if((counter_1_++) % 5 != 0)
  {
    return;
  }

  float belief = pos->pose.pose.position.z;
  float cov_x = pos->pose.covariance.at(0);
  float cov_y = pos->pose.covariance.at(7);
  float cov_theta = pos->pose.covariance.at(35);

  // if(flag_read_1_)
  // {
    // for(int i = 0; i < 4; ++i)
    // {
  out_file_1_ << belief << ' ' << cov_x << ' ' << cov_y << ' '<< cov_theta << '\n';
    // }
  // }
  

  // flag_read_1_ = false;

}

void ReadPose::gpsCb(const sensor_msgs::NavSatFixConstPtr & gps_msg)
{
  ROS_INFO_ONCE("subscribe gps");
  if((counter_2_++) % 2 != 0)
  {
    return;
  }

  float cov_x = gps_msg->position_covariance.at(0);
  float cov_y = gps_msg->position_covariance.at(4);

  out_file_2_ << cov_x << ' ' << cov_y<< '\n';

}

void ReadPose::odomGpsCb(const nav_msgs::OdometryConstPtr & odom_gps_msg)
{
  ROS_INFO_ONCE("subscribe odometry/gps");
  // if((counter_3_++) % 3 != 0)
  // {
  //   return;
  // }

  out_file_3_ << odom_gps_msg->pose.pose.position.x << ' '<< odom_gps_msg->pose.pose.position.y << ' '<< odom_gps_msg->pose.covariance.at(0) << ' ' << odom_gps_msg->pose.covariance.at(7) << ' ' << odom_gps_msg->pose.covariance.at(35) << '\n';

}

void ReadPose::sigRev(const std_msgs::StringConstPtr& sig_msg)
{
	boost::mutex::scoped_lock lr(sig_mutex_);

	if(sig_msg->data == "record")
	{
		flag_read_1_ = true;
	}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_pose_cov");

  ReadPose ReadPose;

  ros::spin();

  return 0;
}