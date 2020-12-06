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

class visualMarker
{
public:
  visualMarker();
  ~visualMarker();

private:

  void mapPosHandler(const geometry_msgs::PoseStampedConstPtr &pos_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber subMapPos_;

  ros::Publisher marker_publisher_;

  tf::TransformBroadcaster* tfb_;
  geometry_msgs::TransformStamped odom_trans_;

  std::vector<geometry_msgs::Point> pos_set_;

  unsigned marker_count_;

  unsigned counter_;

  // visualization_msgs::Marker line_strip;
  double scale_;
  double red_val_;
  double green_val_;
  double blue_val_;

  std::ofstream out_file_1_;
  std::string file_path_1_;
};

visualMarker::visualMarker():marker_count_(0), counter_(0), private_nh_("~")
{

  private_nh_.param("file_path_1", file_path_1_, std::string("/home/lyb/odom_filter_pose.txt"));
  private_nh_.param("scale", scale_, 0.1);
  private_nh_.param("red_val", red_val_, 1.0);
  private_nh_.param("green_val", green_val_, 0.);
  private_nh_.param("blue_val", blue_val_, 0.);

  tfb_ = new tf::TransformBroadcaster();

  subMapPos_ = nh_.subscribe<geometry_msgs::PoseStamped>("/pose", 2, &visualMarker::mapPosHandler, this);

  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);

  // const char* p1 = file_path_1_.c_str();
  // out_file_1_.open(p1, std::ios::app);

}

visualMarker::~visualMarker()
{
  delete tfb_;
  // out_file_1_.close();
}

void visualMarker::mapPosHandler(const geometry_msgs::PoseStampedConstPtr &pos_msg)
{
  int toggle_val = atoi(pos_msg->header.frame_id.data());
  geometry_msgs::Point p;
  p.x = pos_msg->pose.position.x;
  p.y = pos_msg->pose.position.y;

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "yjh";
  line_strip.id = toggle_val;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.lifetime = ros::Duration(1000);
  // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.type = visualization_msgs::Marker::POINTS;
  line_strip.scale.x = scale_;
  line_strip.scale.y = scale_;
  line_strip.scale.z = scale_;
  if(toggle_val >= 7 && toggle_val <=14)
  {
    line_strip.color.r = 0.;
    line_strip.color.g = green_val_;
    line_strip.color.b = 0;

  }else if(toggle_val>14)
  {
    line_strip.color.r = red_val_;
    line_strip.color.g = 0;
    line_strip.color.b = 0;
  }else
  {
    line_strip.color.r = 0;
    line_strip.color.g = 0;
    line_strip.color.b = blue_val_;
  }
  
  line_strip.color.a = 1.0;
  line_strip.lifetime = ros::Duration(0);
    
  line_strip.points.push_back(p);

  marker_publisher_.publish(line_strip);

  ROS_INFO("get map pose.");

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "visual_marker_node");

  visualMarker visual_marker;

  ros::spin();

  return 0;
}