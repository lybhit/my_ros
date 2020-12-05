#include <fstream>
#include <memory>
#include <cmath>
#include <string>
#include <time.h>
#include <stdlib.h>

#include "jsoncpp/json/json.h"

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

struct NodeData{
  std::string id;
  float x;
  float y;
  float z;
  float ang;
};

class Txt2Json
{
public:
  Txt2Json();
  ~Txt2Json();

private:
	void laserRev(const sensor_msgs::LaserScanConstPtr& laser_scan);

  void convert();

	ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber subLaser_;
  ros::Subscriber subSignal_;
  ros::Subscriber subFilterCloud_;
  ros::Subscriber subClusterCloud_;
  ros::Subscriber subHighScoreClusterCloud_;

	std::ifstream in_file_1_;
  std::string file_path_1_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string base_frame_id_;

  std::vector<Eigen::Vector3f> laser_points;

  double range_max_;
  double range_min_;

  bool flag_read_1_;

  boost::mutex sig_mutex_;

};

Txt2Json::Txt2Json():
  flag_read_1_(false),
  private_nh_("~")
{

	private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
	private_nh_.param("file_path_1", file_path_1_, std::string("/home/lyb/nodes.txt"));

  subLaser_ = nh_.subscribe("scan", 2, &ReadLaser::laserRev, this);
  subSignal_ = nh_.subscribe("signal", 1, &ReadLaser::sigRev, this);

	tf_.reset(new tf2_ros::Buffer());

	const char* p1 = file_path_1_.c_str();
  in_file_1_.open(p1);
}

Txt2Json::~Txt2Json()
{
  if(in_file_1_.is_open())
  {
	  in_file_1_.close();

  }

}

void Txt2Json::convert()
{
  NodeData data;
  std::vector<NodeData> vec_data;
  if(in_file_1_.is_open())
  {
    while(!in_file_1_.eof())
    {
       in_file_1_ >> data.id >> data.x >> data.y >> data.z >> data.ang;
       vec_data.push_back(data);
    }
  }

  in_file_1_.close();

  int len = vec_data.size();
  std::vector<Json::Value> vec_root;
  vec_root.resize(len);
  
  for(int i = 0; i < len; ++i)
  {
    Json::Value root = vec_root[i];
    root["id"] = Json::Value(vec_data[i].id);

    Json::Value friends;

  }



}






  