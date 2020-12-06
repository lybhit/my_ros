
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

class RecordPoseInfo
{
public:
  RecordPoseInfo();
  ~RecordPoseInfo();

private:
	void poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & cluster_cloud);
	void gpsCb(const sensor_msgs::NavSatFixConstPtr & gps_msg);
	void sigRev(const std_msgs::StringConstPtr& sig_msg);
  void odomCb(const nav_msgs::OdometryConstPtr & odom_gps_msg);
  void odomGpsCb(const nav_msgs::OdometryConstPtr & odom_gps_msg);

	ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber subPos_;
  ros::Subscriber subSignal_;
  ros::Subscriber subGps_;
  ros::Subscriber subOdom_;
  ros::Subscriber subOdomGps_;

	std::ofstream out_file_1_, out_file_2_, out_file_3_, out_file_4_;
  std::string fp_amcl_, fp_gps_, fp_odomF_, fp_odomG_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string base_frame_id_;

  bool flag_read_1_;

  boost::mutex sig_mutex_;
  int counter_1_, counter_2_, counter_3_, counter_4_;

  geometry_msgs::PoseWithCovarianceStamped pos_;

};

RecordPoseInfo::RecordPoseInfo():
  flag_read_1_(false),
  counter_1_(0),
  counter_2_(0),
  counter_3_(0),
  counter_4_(0),
  private_nh_("~")
{
	private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));

	private_nh_.param("fp_amcl", fp_amcl_, std::string("/home/lyb/amcl_points"));
	private_nh_.param("fp_gps", fp_gps_, std::string("/home/lyb/gps_points"));
	private_nh_.param("fp_odomF", fp_odomF_, std::string("/home/lyb/odom_filtered"));
	private_nh_.param("fp_odomG", fp_odomG_, std::string("/home/lyb/odom_gps"));

  subPos_= nh_.subscribe("pose_with_cov", 2, &RecordPoseInfo::poseCb, this);
  subGps_= nh_.subscribe("ub482/bp_fix", 2, &RecordPoseInfo::gpsCb, this);
  subOdom_= nh_.subscribe("odometry/filtered", 2, &RecordPoseInfo::odomCb, this);
  subOdomGps_= nh_.subscribe("odometry/gps", 2, &RecordPoseInfo::odomGpsCb, this);

  subSignal_ = nh_.subscribe("signal", 1, &RecordPoseInfo::sigRev, this);

	tf_.reset(new tf2_ros::Buffer());

  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  std::string file_suffix = iso_time_str + ".txt";
  std::string file_1 = fp_amcl_ + file_suffix;
  std::string file_2 = fp_gps_ + file_suffix;
  std::string file_3 = fp_odomF_ + file_suffix;
  std::string file_4 = fp_odomG_ + file_suffix;

	const char* p1 = file_1.c_str();
  out_file_1_.open(p1, std::ios::app);

	const char* p2 = file_2.c_str();
  out_file_2_.open(p2, std::ios::app);

	const char* p3 = file_3.c_str();
  out_file_3_.open(p3, std::ios::app);

	const char* p4 = file_4.c_str();
  out_file_4_.open(p4, std::ios::app);
}

RecordPoseInfo::~RecordPoseInfo()
{
	out_file_1_.close();
	out_file_2_.close();
	out_file_3_.close();
	out_file_4_.close();
}

void RecordPoseInfo::poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & pos)
{
  boost::mutex::scoped_lock lr(sig_mutex_);
  ROS_INFO_ONCE("subscribe pose_with_cov.");

  boost::posix_time::ptime my_posix_time = pos->header.stamp.toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  if((counter_1_++) % 5 != 0)
  {
    return;
  }
  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg = pos->pose.pose.orientation;
  tf2::convert(quat_msg , quat_tf);
  double roll = 0.;
  double pitch = 0.;
  double yaw = 0.;

  // quat_tf.getRPY(roll, pitch, yaw);
  tf2::getEulerYPR(quat_tf, yaw, pitch, roll);

  float belief = pos->pose.pose.position.z;
  float cov_x = pos->pose.covariance.at(0);
  float cov_y = pos->pose.covariance.at(7);
  float cov_theta = pos->pose.covariance.at(35);

  // if(flag_read_1_)
  // {
      out_file_1_ <<  iso_time_str << ' '<<pos->pose.pose.position.x << ' ' <<pos->pose.pose.position.y << ' ' << yaw<< ' '<<belief << ' ' << cov_x << ' ' << cov_y << ' '<< cov_theta << '\n';
  // }
  

  // flag_read_1_ = false;

}

void RecordPoseInfo::gpsCb(const sensor_msgs::NavSatFixConstPtr & gps_msg)
{
  // if((counter_2_++) % 2 != 0)
  // {
  //   return;
  // }
  boost::posix_time::ptime my_posix_time = gps_msg->header.stamp.toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  float cov_x = gps_msg->position_covariance.at(0);
  float cov_y = gps_msg->position_covariance.at(4);

  out_file_2_ << iso_time_str <<' '<<cov_x << ' ' << cov_y<< '\n';

}

void RecordPoseInfo::odomCb(const nav_msgs::OdometryConstPtr & odom_msg)
{
  boost::posix_time::ptime my_posix_time = odom_msg->header.stamp.toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg = odom_msg->pose.pose.orientation;
  tf2::convert(quat_msg , quat_tf);
  double roll = 0.;
  double pitch = 0.;
  double yaw = 0.;

  // quat_tf.setRPY(roll, pitch, yaw);
  tf2::getEulerYPR(quat_tf, yaw, pitch, roll);
  

  float cov_x = odom_msg->pose.covariance.at(0);
  float cov_y = odom_msg->pose.covariance.at(7);
  float cov_theta = odom_msg->pose.covariance.at(35);

  out_file_3_ << iso_time_str<< ' '<<odom_msg->pose.pose.position.x << ' ' << odom_msg->pose.pose.position.y<< ' '<<yaw << ' '<<  cov_x<<  ' '<< cov_y<< ' '<< cov_theta<<'\n';
}

void RecordPoseInfo::odomGpsCb(const nav_msgs::OdometryConstPtr & odom_gps_msg)
{
  boost::posix_time::ptime my_posix_time = odom_gps_msg->header.stamp.toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg = odom_gps_msg->pose.pose.orientation;
  tf2::convert(quat_msg , quat_tf);
  double roll = 0.;
  double pitch = 0.;
  double yaw = 0.;

  // quat_tf.setRPY(roll, pitch, yaw);
  tf2::getEulerYPR(quat_tf, yaw, pitch, roll);

  float cov_x = odom_gps_msg->pose.covariance.at(0);
  float cov_y = odom_gps_msg->pose.covariance.at(7);
  float cov_theta = odom_gps_msg->pose.covariance.at(35);

  out_file_4_ << iso_time_str<< ' '<<odom_gps_msg->pose.pose.position.x << ' ' << odom_gps_msg->pose.pose.position.y<< ' '<<yaw << ' '<<  cov_x<<  ' '<< cov_y<< ' '<< cov_theta<<'\n';
}


void RecordPoseInfo::sigRev(const std_msgs::StringConstPtr& sig_msg)
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

  RecordPoseInfo recordPoseInfo;

  ros::spin();

  return 0;
}