#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include "rs_to_velodyne.hpp"


void runFromBag(const std::string &in_bag_fn, const std::string &out_bag_fn)
{
  rosbag::Bag bag;
  bag.open(in_bag_fn, rosbag::bagmode::Read);

  rosbag::Bag bag_out;
  bag_out.open(out_bag_fn, rosbag::bagmode::Write);

  std::vector<std::string> topics;
  topics.push_back(std::string("/imu_data_fix"));
  std::string scan_topic_name = "/rslidar_points"; // TODO determine what topic this actually is from ROS
  topics.push_back(scan_topic_name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Sleep for a second to let all subscribers connect
  ros::WallDuration(1.0).sleep();

  ros::WallTime start(ros::WallTime::now());

  BOOST_FOREACH(rosbag::MessageInstance const msg, view)
  {
    if (!ros::ok())
    {
      break;
    }

    // Process any ros messages or callbacks at this point
    // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());

    if(msg.getTopic() == "/rslidar_points")
    {
      sensor_msgs::PointCloud2::ConstPtr point_cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
      if (point_cloud_msg!= NULL)
      {
        Rs2Velodyne<RsPointXYZIRT, VelodynePointXYZIRT> r2v;
        r2v.setOutputType("XYZIRT");

        sensor_msgs::PointCloud2::ConstPtr mm = r2v.rsHandler_XYZIRT(*point_cloud_msg);
        if(mm != nullptr)
        {
          bag_out.write("velodyne_points", point_cloud_msg->header.stamp, mm);
        }
      }
    }else if(msg.getTopic() == "/imu_data_fix"){
      // ROS_INFO("topic is imu_data_fix");
      sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      if(imu_msg != nullptr)
      {
        bag_out.write("imu_data", msg.getTime(), imu_msg);
      }
    }

    // ROS_WARN_STREAM("Unsupported message type" << msg.getTopic());
  }

  bag.close();
  bag_out.close();

}


int main(int argc, char **argv) {
   ros::init(argc, argv, "rs_converter");
   ros::NodeHandle nh;
   
   std::string bag_in(argv[1]);
   std::string bag_out(argv[2]);
   std::cout << argv[1] << std::endl;
   std::cout << argv[2] << std::endl;

   runFromBag(bag_in, bag_out);

   ros::spinOnce();

   return 0;
}
