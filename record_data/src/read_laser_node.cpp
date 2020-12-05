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

class ReadLaser
{
public:
  ReadLaser();
  ~ReadLaser();

private:
	void laserRev(const sensor_msgs::LaserScanConstPtr& laser_scan);
	void filterCloudRev(const geometry_msgs::PoseArrayConstPtr & point_cloud);
	void clusterCloudRev(const geometry_msgs::PoseArrayConstPtr & cluster_cloud);
  void clusterHighScoreCloudRev(const geometry_msgs::PoseArrayConstPtr & cluster_cloud);
	void sigRev(const std_msgs::StringConstPtr& sig_msg);

	ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber subLaser_;
  ros::Subscriber subSignal_;
  ros::Subscriber subFilterCloud_;
  ros::Subscriber subClusterCloud_;
  ros::Subscriber subHighScoreClusterCloud_;

	std::ofstream out_file_1_;
	std::ofstream out_file_2_;
	std::ofstream out_file_3_;
  std::string file_path_1_;
  std::string file_path_2_;
  std::string file_path_3_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string base_frame_id_;

  std::vector<Eigen::Vector3f> laser_points;

  double range_max_;
  double range_min_;

  bool flag_read_1_;
  bool flag_read_2_;
  bool flag_read_3_;
  bool scan_read_;
  bool filter_cloud_read_;
  bool clustered_cloud_read_;
  bool high_score_clustered_cloud_read_;

  boost::mutex sig_mutex_;

};

ReadLaser::ReadLaser():
  flag_read_1_(false),
  flag_read_2_(false),
  flag_read_3_(false),
  // scan_read_(false),
  // filter_cloud_read_(false),
  // clustered_cloud_read_(false),
  private_nh_("~")
{
	private_nh_.param("scan_read", scan_read_, false);
	private_nh_.param("filter_cloud_read", filter_cloud_read_, false);
	private_nh_.param("clustered_cloud_read", clustered_cloud_read_, false);
	private_nh_.param("high_score_clustered_cloud_read", high_score_clustered_cloud_read_, false);

	private_nh_.param("range_max", range_max_, 100.);
	private_nh_.param("range_min", range_min_, 0.2);

	private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
	private_nh_.param("file_path_1", file_path_1_, std::string("/home/lyb/filtered_points"));
	private_nh_.param("file_path_2", file_path_2_, std::string("/home/lyb/clustered_particlepoints"));
	private_nh_.param("file_path_3", file_path_3_, std::string("/home/lyb/high_score_clustered_particlepoints"));

  subLaser_ = nh_.subscribe("scan", 2, &ReadLaser::laserRev, this);
  subSignal_ = nh_.subscribe("signal", 1, &ReadLaser::sigRev, this);
  subFilterCloud_ = nh_.subscribe("filter_cloud", 2, &ReadLaser::filterCloudRev, this);
  subClusterCloud_ = nh_.subscribe("clustered_particlecloud", 2, &ReadLaser::clusterCloudRev, this);
  subHighScoreClusterCloud_ = nh_.subscribe("high_score_clustered_particlecloud", 2, &ReadLaser::clusterHighScoreCloudRev, this);

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

ReadLaser::~ReadLaser()
{
	out_file_1_.close();
	out_file_2_.close();
	out_file_3_.close();
}


void ReadLaser::laserRev(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  if(!scan_read_)
  {
    return;
  }

	boost::mutex::scoped_lock lr(sig_mutex_);

	geometry_msgs::TransformStamped laser_to_base;
  laser_to_base = tf_->lookupTransform(base_frame_id_, laser_scan->header.frame_id, ros::Time(0));

  unsigned int num = laser_scan->ranges.size();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan->angle_min);
  geometry_msgs::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = laser_scan->header.stamp;
  min_q.header.frame_id = laser_scan->header.frame_id;
  tf2::convert(q, min_q.quaternion);

  q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
  inc_q.header = min_q.header;
  tf2::convert(q, inc_q.quaternion);
  try
  {
    tf_->transform(min_q, min_q, base_frame_id_);
    tf_->transform(inc_q, inc_q, base_frame_id_);
  }
  catch(tf2::TransformException& e)
  {
    ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
             e.what());
    return;
  }

  double angle_min = tf2::getYaw(min_q.quaternion);
  double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;


	laser_points.clear();
  laser_points.reserve(num);

  Eigen::Matrix<double, 2, 1> translation(laser_to_base.transform.translation.x, laser_to_base.transform.translation.y);
  double angle = tf2::getYaw(laser_to_base.transform.rotation);
  Eigen::Rotation2D<double> rotation(angle);
  Eigen::Matrix<double, 2, 2> rotation_matrix = rotation.toRotationMatrix();
  Eigen::Matrix<double, 3, 3> transform;
  transform << rotation_matrix, translation, 0.0, 0.0, 1.0;

  if(flag_read_1_)
  {
  	for(int i=0;i<num;i++)
	  {
	    Eigen::Vector3f tmp_point;

	    double r, theta;
	    // amcl doesn't (yet) have a concept of min range.  So we'll map short
	    // readings to max range.
	    if(laser_scan->ranges[i] <= range_min_)
	      r = range_max_;
	    else
	      r = laser_scan->ranges[i];
	    // Compute bearing
	    theta = angle_min +
	            (i * angle_increment);

	    tmp_point.x() = r * cos(theta);
	    tmp_point.y() = r * sin(theta);

	    const Eigen::Matrix<double, 3, 1> point(tmp_point.x(), tmp_point.y(), 1.);

	    const Eigen::Matrix<double, 3, 1> world = transform * point;

	    tmp_point.x() = world[0];
	    tmp_point.y() = world[1];
	    tmp_point.z() = world[2];

	    laser_points.emplace_back(tmp_point);

	  }

	  for(int j=0; j<laser_points.size(); ++j)
  	{
  		out_file_1_ << laser_points[j].x() << ' ' << laser_points[j].y() << '\n';
  	}

  	flag_read_1_ = false;

  }

}

void ReadLaser::filterCloudRev(const geometry_msgs::PoseArrayConstPtr & point_cloud)
{
  if(!filter_cloud_read_)
  {
    return;
  }
  boost::mutex::scoped_lock lr(sig_mutex_);

  int len = point_cloud->poses.size();

  if(flag_read_1_)
  {
    for(int i = 0; i < len; ++i)
    {
      out_file_1_ << point_cloud->poses[i].position.x << ' ' <<  point_cloud->poses[i].position.y<< '\n';
    }
  }
  

  flag_read_1_ = false;

}

void ReadLaser::clusterCloudRev(const geometry_msgs::PoseArrayConstPtr & cluster_cloud)
{
  if(!clustered_cloud_read_)
  {
    return;
  }

  boost::mutex::scoped_lock lr(sig_mutex_);

  int len = cluster_cloud->poses.size();

  if(flag_read_2_)
  {
    for(int i = 0; i < len; ++i)
    {
      out_file_2_ << cluster_cloud->poses[i].position.x << ' ' <<  cluster_cloud->poses[i].position.y << ' ' << cluster_cloud->poses[i].position.z<< '\n';
    }
  }
  
  flag_read_2_ = false;
}

void ReadLaser::clusterHighScoreCloudRev(const geometry_msgs::PoseArrayConstPtr & cluster_cloud)
{
  if(!high_score_clustered_cloud_read_)
  {
    return;
  }

  boost::mutex::scoped_lock lr(sig_mutex_);

  int len = cluster_cloud->poses.size();

  if(flag_read_3_)
  {
    for(int i = 0; i < len; ++i)
    {
      out_file_3_ << cluster_cloud->poses[i].position.x << ' ' <<  cluster_cloud->poses[i].position.y << ' ' << cluster_cloud->poses[i].position.z<< '\n';
    }
  }
  
  flag_read_3_ = false;
}

void ReadLaser::sigRev(const std_msgs::StringConstPtr& sig_msg)
{
	boost::mutex::scoped_lock lr(sig_mutex_);

	if(sig_msg->data == "record")
	{
		flag_read_1_ = true;
		flag_read_2_ = true;
		flag_read_3_ = true;
	}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_laser");

  ReadLaser readlaser;

  ros::spin();

  return 0;
}