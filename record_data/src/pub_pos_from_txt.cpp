#include "read_txt.h"

#include <fstream>
#include <memory>
#include <cmath>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <sstream>

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
#include "dg_msgs/String.h"

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

class ReadTxtPose
{
public:
  ReadTxtPose();
  ~ReadTxtPose();

private:
	void taskCb(const dg_msgs::StringConstPtr & navigation_cmd);
	void gpsCb(const sensor_msgs::NavSatFixConstPtr & gps_msg);

	ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  
  ros::Publisher pubPos_;
  ros::Subscriber subPos_;

  std::string base_frame_id_;
  std::string file_2_read_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

	std::ofstream out_file_1_;
  std::string file_path_1_;

  bool flag_read_1_;

  boost::mutex sig_mutex_;
  int counter_1_;

};

ReadTxtPose::ReadTxtPose():
  flag_read_1_(false),
  counter_1_(0),
  private_nh_("~")
{
	private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
	private_nh_.param("file_2_read", file_2_read_, std::string("nodes.txt"));
	private_nh_.param("file_path_1", file_path_1_, std::string("/home/lyb/filtered_points"));

  subPos_ = nh_.subscribe("navigation_cmd", 2, &ReadTxtPose::taskCb, this);

  pubPos_ = nh_.advertise<geometry_msgs::PoseStamped>("nav_info", 1);

	tf_.reset(new tf2_ros::Buffer());

  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  std::string file_suffix = iso_time_str + ".txt";
  std::string file_1 = file_path_1_ + file_suffix;

	// const char* p1 = file_1.c_str();
  // out_file_1_.open(p1, std::ios::app);

}

ReadTxtPose::~ReadTxtPose()
{
	// out_file_1_.close();
}

void ReadTxtPose::taskCb(const dg_msgs::StringConstPtr & navigation_cmd)
{
  int id_num;

  std::vector<std::string> res;
  std::string result;
  std::string info = navigation_cmd->data;

  std::cout << info << std::endl;

  if(info.empty())
  {
    return;
  }

  std::stringstream input(info);
  
  while(input >> result)
  {
    res.push_back(result);
  }

  if(res[0] == "pass" && res[1] == "3")
  {
    id_num = atoi(res[2].c_str());
  }else{
    return;
  }

  std::string line_info;
  line_info = readLine(file_2_read_.c_str(), id_num);

  std::stringstream input_1(line_info);

  std::cout << input_1.str() << std::endl;

  std::string pointID, pointType;
  float pos_x, pos_y, pos_a;

  input_1 >> pointID >> pointType >> pos_x >> pos_y >> pos_a;

  // std::cout << pointID << std::endl;
  // std::cout << pointType<< std::endl;
  // std::cout << pos_x << std::endl;
  // std::cout << pos_y << std::endl;
  // std::cout << pos_a << std::endl;

  geometry_msgs::PoseStamped target_pos;
  target_pos.header.frame_id = pointID;
  target_pos.pose.position.x = pos_x;
  target_pos.pose.position.y = pos_y;
  target_pos.pose.position.z = 0.;

  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;
  quat_tf.setRPY(pos_a, 0, 0);
  tf2::convert(quat_tf, quat_msg);

  target_pos.pose.orientation = quat_msg;

  pubPos_.publish(target_pos);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_pose_cov");

  ReadTxtPose ReadTxtPose;

  ros::spin();

  return 0;
}