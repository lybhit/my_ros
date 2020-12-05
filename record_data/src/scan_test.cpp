#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

class ScanHandle
{
    public:
      ScanHandle();
      ~ScanHandle();

      void run();
    private:
      void send_scan_data(const ros::TimerEvent& te);
      void laserRev(const sensor_msgs::LaserScanConstPtr& laser_scan);
      void sigRev(const std_msgs::String& sig );

      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;

      ros::Subscriber subLaser_;
      ros::Subscriber subSignal_;
      ros::Publisher pubLaser_;

      sensor_msgs::LaserScan scan_msg_;
      bool read_once_;
      bool publish_scan_;
      bool send_scan_flag_;

      int rate_;
      ros::Timer updateTimer_;
};

ScanHandle::ScanHandle():
    private_nh_("~"),
    publish_scan_(false),
    send_scan_flag_(false)
{
    private_nh_.param("rate", rate_, 25);
    private_nh_.param("read_once", read_once_, true);
    private_nh_.param("publish_scan", publish_scan_, false);

    subLaser_ = nh_.subscribe("scan", 2, &ScanHandle::laserRev, this);
    subSignal_ = nh_.subscribe("pub_scan", 1, &ScanHandle::sigRev, this);
    pubLaser_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
}

ScanHandle::~ScanHandle()
{

}

void ScanHandle::run()
{
    if(publish_scan_)
    {
        updateTimer_ = ros::Timer(nh_.createTimer(ros::Duration(1./rate_),
                                                &ScanHandle::send_scan_data,
                                                this));
    }
}

void ScanHandle::laserRev(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    if(read_once_)
    {
      scan_msg_.header.frame_id = laser_scan->header.frame_id;
      scan_msg_.angle_increment = laser_scan->angle_increment;
      scan_msg_.angle_max = laser_scan->angle_max;
      scan_msg_.angle_min = laser_scan->angle_min;
      scan_msg_.range_max = laser_scan->range_max;
      scan_msg_.range_min = laser_scan->range_min;

      for(int i = 0; i<laser_scan->ranges.size(); ++i)
      {
          scan_msg_.ranges.push_back(laser_scan->ranges[i]);
      }

      read_once_  = false;
    }else
    {
        return;
    }
    
}

void ScanHandle::sigRev(const std_msgs::String& sig )
{
    if(sig.data == "pub_scan")
    {
        send_scan_flag_ = true;
    }
}

void ScanHandle::send_scan_data(const ros::TimerEvent& te)
{
    if(!send_scan_flag_)
    {
        return;
    }

    scan_msg_.header.stamp = ros::Time::now();

    pubLaser_.publish(scan_msg_);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "scan_test");

    ScanHandle scan_handle;
    scan_handle.run();

    ros::spin();

    return 0;   
}

