#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <fstream>

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}

static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

class odomTf
{
public:
  odomTf();
  ~odomTf();

private:

  void odomFilterHandler(const nav_msgs::OdometryConstPtr &raw_msg);
  void odomGpsHandler(const nav_msgs::OdometryConstPtr &raw_msg);

  void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg);

  void mapPosHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pos_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber subOdomGps_;
  ros::Subscriber subOdomFilter_;
  ros::Subscriber subMapPos_;
  ros::Subscriber subImu_;

  ros::Publisher marker_publisher_;
  ros::Publisher pubYaw_;

  tf::TransformBroadcaster* tfb_;
  geometry_msgs::TransformStamped odom_trans_;

  std::vector<geometry_msgs::Point> pos_set_;

  unsigned marker_count_;

  unsigned counter_;

  visualization_msgs::Marker line_strip;

  std::ofstream out_file_1_;
  std::string file_path_1_;

  std::ofstream out_file_2_;
  std::string file_path_2_;

  std::ofstream out_file_3_;
  std::string file_path_3_;
};

odomTf::odomTf():marker_count_(0), counter_(0), private_nh_("~")
{

  private_nh_.param("file_path_1", file_path_1_, std::string("/home/lyb/odom_filter_pose.txt"));
  private_nh_.param("file_path_2", file_path_2_, std::string("/home/lyb/odom_gps_pose.txt"));
  private_nh_.param("file_path_3", file_path_3_, std::string("/home/lyb/amcl_pose_with_cov.txt"));

  tfb_ = new tf::TransformBroadcaster();

  subOdomFilter_ = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 2, &odomTf::odomFilterHandler, this);
  subOdomGps_ = nh_.subscribe<nav_msgs::Odometry>("/odometry/gps", 2, &odomTf::odomGpsHandler, this);
  subImu_ = nh_.subscribe<sensor_msgs::Imu>("/lpms_imu", 2, &odomTf::imuHandler, this);

  subMapPos_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pose_with_cov", 2, &odomTf::mapPosHandler, this);

  pubYaw_ = nh_.advertise<std_msgs::Float64>("/yaw",1);

  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);

  const char* p1 = file_path_1_.c_str();
  out_file_1_.open(p1, std::ios::app);

  const char* p2 = file_path_2_.c_str();
  out_file_2_.open(p2, std::ios::app);

  const char* p3 = file_path_3_.c_str();
  out_file_3_.open(p3, std::ios::app);
}

odomTf::~odomTf()
{
  delete tfb_;
  out_file_1_.close();
  out_file_2_.close();
  out_file_3_.close();
}

void odomTf::odomFilterHandler(const nav_msgs::OdometryConstPtr &raw_msg)
{

  out_file_1_ << raw_msg->pose.pose.position.x << ' ' << raw_msg->pose.pose.position.y << '\n';

  // odom_trans_.header.stamp = ros::Time::now();
  // odom_trans_.header.frame_id = "odom";
  // odom_trans_.child_frame_id = "base_link";
  // odom_trans_.transform.translation.x = raw_msg->pose.pose.position.x;
  // odom_trans_.transform.translation.y = raw_msg->pose.pose.position.y;
  // odom_trans_.transform.translation.z = 0.0;
  // odom_trans_.transform.rotation = raw_msg->pose.pose.orientation;

  // tfb_->sendTransform(odom_trans_);

	// float covariance[36] = {0.001, 0, 0, 0, 0, 0, //covariance on gps_x
 //                          0, 0.001, 0, 0, 0, 0, //covariance on gps_y
 //                          0, 0, 9999, 0, 0, 0, //covariance on gps_z
 //                          0, 0, 0, 9999, 0, 0, //large covariance on rot x
 //                          0, 0, 0, 0, 9999, 0, //large covariance on rot y
 //                          0, 0, 0, 0, 0, 0.02};//large covariance on rot z

 //    nav_msgs::Odometry msg;
 //    msg = *raw_msg;

 //    for(int i = 0; i < 36; ++i)
 //    {
 //    	msg.pose.covariance[i] = covariance[i];
 //    }

 //    pubOdom.publish(msg);

}

void odomTf::odomGpsHandler(const nav_msgs::OdometryConstPtr &raw_msg)
{
  out_file_2_ << raw_msg->pose.pose.position.x << ' ' << raw_msg->pose.pose.position.y << '\n';

}

void odomTf::mapPosHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pos_msg)
{
  if((counter_ ++) < 2 )
  {
    return;
  }

  geometry_msgs::Point p;
  p.x = pos_msg->pose.pose.position.x;
  p.y = pos_msg->pose.pose.position.y;

  out_file_3_ << p.x << ' ' << p.y << '\n';

  // visualization_msgs::Marker line_strip;
  // line_strip.header.frame_id = "map";
  // line_strip.header.stamp = ros::Time::now();
  // line_strip.ns = "yjh";
  // line_strip.id = 0;
  // line_strip.action = visualization_msgs::Marker::ADD;
  // // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  // line_strip.type = visualization_msgs::Marker::POINTS;
  // line_strip.scale.x = 0.1;
  // line_strip.scale.y = 0.1;
  // line_strip.scale.z = 0.1;
  // line_strip.color.r = 1.0;
  // line_strip.color.g = 0;
  // line_strip.color.b = 0.0;
  // line_strip.color.a = 1.0;
  // line_strip.lifetime = ros::Duration(0);
    
  // line_strip.points.push_back(p);

  // marker_publisher_.publish(line_strip);

  // ROS_INFO("get amcl_pose");

}



void odomTf::imuHandler(const sensor_msgs::ImuConstPtr &imu_msg)
{
  double msg;

  msg = tf::getYaw(imu_msg->orientation);

  ROS_INFO("Yaw  = %lf", msg);

	// for(int i = 0; i < 9; ++i)
	// {
	// 	msg.orientation_covariance[i] = covariance[i];
	// 	msg.angular_velocity_covariance[i] = covariance[i];
	// 	msg.linear_acceleration_covariance[i] = covariance[i];
	// }

	// pubYaw_.publish(msg);

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_tf_node");

  odomTf odom_tf;

  ros::spin();

  return 0;
}