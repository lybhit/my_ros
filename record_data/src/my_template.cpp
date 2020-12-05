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

	ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber subPos_;

  std::string base_frame_id_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

	std::ofstream out_file_1_;
  std::string file_path_1_;

  bool flag_read_1_;

  boost::mutex sig_mutex_;
  int counter_1_;

};

ReadPose::ReadPose():
  flag_read_1_(false),
  counter_1_(0),
  private_nh_("~")
{
	private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
	private_nh_.param("file_path_1", file_path_1_, std::string("/home/lyb/filtered_points"));

  subPos_= nh_.subscribe("pose_with_cov", 2, &ReadPose::poseCb, this);

	tf_.reset(new tf2_ros::Buffer());

  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  std::string file_suffix = iso_time_str + ".txt";
  std::string file_1 = file_path_1_ + file_suffix;

	const char* p1 = file_1.c_str();
  out_file_1_.open(p1, std::ios::app);

}

ReadPose::~ReadPose()
{
	out_file_1_.close();
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

  out_file_1_ << belief << ' ' << cov_x << ' ' << cov_y << ' '<< cov_theta << '\n';
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_pose_cov");

  ReadPose ReadPose;

  ros::spin();

  return 0;
}