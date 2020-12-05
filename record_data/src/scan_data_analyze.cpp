#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#include <fstream>

#include <boost/thread.hpp>

class ScanAnalyze
{
    public:
      ScanAnalyze();
      ~ScanAnalyze();

    private:
      void laserRev(const sensor_msgs::LaserScanConstPtr& laser_scan);

      void calThread();
      void calProcess();

      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;

      ros::Subscriber subLaser_;

      std::vector<std::vector<float>> range_container;

      ros::Time tik_start_;
      ros::Time tik_real_;
      
      boost::recursive_mutex cal_mutex_;
      boost::condition_variable_any cal_cond_;
      boost::thread *anlyze_thread_;
      bool calRun_;

      std::string file_out_;
};

ScanAnalyze::ScanAnalyze():
    private_nh_("~"),
    calRun_(false)
{
    private_nh_.param("file_out", file_out_, std::string("/home/lyb/tools_ws/src/test_tools/record_data/scan_error.txt"));

    subLaser_ = nh_.subscribe("scan", 2, &ScanAnalyze::laserRev, this);

    //set up the planner's thread
    anlyze_thread_ = new boost::thread(boost::bind(&ScanAnalyze::calThread, this));

    tik_start_ = ros::Time::now();
}

ScanAnalyze::~ScanAnalyze()
{
    anlyze_thread_->interrupt();
    anlyze_thread_->join();

    delete anlyze_thread_;
}

void ScanAnalyze::calThread()
{
    bool wait_for_wake = false;
    bool done = false;
    boost::unique_lock<boost::recursive_mutex> lock(cal_mutex_);
    while(nh_.ok() && !done)
    {
        while(wait_for_wake || !calRun_)
        {
          cal_cond_.wait(lock);
          wait_for_wake = false;
        }

        lock.unlock();

        calProcess();

        done = true;
    }

    return;
}

void ScanAnalyze::calProcess()
{
    std::ofstream out_file_1_;
    const char* p = file_out_.c_str();
    out_file_1_.open(p, std::ios::app);
    out_file_1_.setf(std::ios::fixed, std::ios::floatfield);
    out_file_1_.precision(4);

    float mean = 0.;
    std::vector<float> mead_vec;
    int scan_len = range_container.front().size();
    int sample_len = range_container.size();
    for(int i = 0; i < scan_len; i++)
    {
        mean = 0.;
        for(int j=0; j < sample_len; j++)
        {
            mean += range_container[j][i];

        }
        mean /= sample_len;
        mead_vec.push_back(mean);
    }

    for(int i = 0; i < scan_len; i++)
    {
        for(int j=0; j < sample_len; j++)
        {
            range_container[j][i] -= mead_vec[i];
            out_file_1_ << range_container[j][i] << ' ';

        }

        out_file_1_ << '\n';
    }

    out_file_1_.close();
}


void ScanAnalyze::laserRev(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    tik_real_ = ros::Time::now();
    if((tik_real_ - tik_start_).toSec() >10.0)
    {
        boost::unique_lock<boost::recursive_mutex> lock(cal_mutex_);
        calRun_ = true;
        cal_cond_.notify_one();
        lock.unlock();

        return;
    }

    int len = laser_scan->ranges.size();

    std::vector<float> range_data;
    range_data.resize(len);
     
      for(int i = 0; i < len; ++i)
      {
          range_data[i] = laser_scan->ranges[i];
      }

     range_container.push_back(range_data);
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "scan_data_analyze");

    ScanAnalyze scan_analyze;

    ros::spin();

    return 0;   
}

