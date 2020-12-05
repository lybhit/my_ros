#include <fstream>
#include <Eigen/Core>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

#include <boost/thread/mutex.hpp>


enum PointType{PASS, CHARGE};

class autoGetGoal
{
public:
  autoGetGoal();
  ~autoGetGoal();

private:

  void joyHandler(const sensor_msgs::JoyConstPtr &joy_msg);
  void poseHandler(const geometry_msgs::PoseStampedConstPtr &pos_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subJoy_;
  ros::Subscriber subPose_;
  tf::TransformListener tf_;

  std::string p_map_frame_;
  std::string p_base_frame_;

  std::ofstream out_file_;
  std::string file_path_;

  ros::Time time_START;

  bool stat_START;
  bool stat_RB;
  
  Eigen::Vector3f pose_;
  unsigned int point_id_;
  enum PointType point_type_;

  boost::mutex data_mutex_;

};

autoGetGoal::autoGetGoal():private_nh_("~"), stat_START(false), stat_RB(false),point_id_(1)
{

  private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
  private_nh_.param("map_frame", p_map_frame_, std::string("map"));

  private_nh_.param("file_path", file_path_, std::string("/home/lyb/nav_pose.txt"));

  const char* p = file_path_.c_str();
  out_file_.open(p, std::ios::app);

  subJoy_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 3, &autoGetGoal::joyHandler, this);
  subPose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/amcl_pose", 3, &autoGetGoal::poseHandler, this);
}

autoGetGoal::~autoGetGoal()
{

  out_file_.close();

}

void autoGetGoal::joyHandler(const sensor_msgs::JoyConstPtr &joy_msg)
{
  // boost::mutex::scoped_lock lk(data_mutex_);
  ROS_INFO("joy is active.");
  if(joy_msg->buttons[7])
  {
    stat_START = true;

    time_START = ros::Time::now();
  }

  if(stat_START)
  {
    if(joy_msg->buttons[5] && (time_START - ros::Time::now()).toSec() < 2.0)
    {
      stat_RB = true;
      ROS_INFO("stat_RB is true.");
    }else{
      stat_START = false;
      stat_RB = false;
    }
      
  }
  
}

void autoGetGoal::poseHandler(const geometry_msgs::PoseStampedConstPtr &pos_msg)
{
  // boost::mutex::scoped_lock lk(data_mutex_);
  geometry_msgs::Pose p;

  p.position.x = pos_msg->pose.position.x;
  p.position.y = pos_msg->pose.position.y;
  p.position.z = tf::getYaw(pos_msg->pose.orientation);

  point_type_ = PASS;

  if(stat_START && stat_RB)
  {
    out_file_ << point_id_ << ' '<< point_type_ << ' ' << p.position.x << ' ' << p.position.y << ' ' << p.position.z << '\n';
    point_id_++;

    ROS_INFO("point pose = %f, %f, %f", p.position.x, p.position.y, p.position.z);

    stat_START = false;
    stat_RB = false;
  }

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_get_goal_node");

  autoGetGoal get_goal;

  ros::spin();

  return 0;
}
