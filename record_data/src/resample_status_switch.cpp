#include <memory>
#include <cmath>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <sstream>
#include <limits>
#include <vector>
#include <ctime>

#include "boost/date_time/posix_time/posix_time.hpp"
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <stdio.h>
#include <unistd.h>

using namespace std;

class ResampleStatusSwitch
{
private:
  bool stop_resample_flag_;
  bool status_change_flag_;
  double match_ratio_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;

public:
  ResampleStatusSwitch();
  ~ResampleStatusSwitch(){}

  bool needStopResample();
  void holdStatusForSometime(double dur, double match_ratio);
  bool getStatusChangeFlag();

};

ResampleStatusSwitch::ResampleStatusSwitch():stop_resample_flag_(false){

}



bool ResampleStatusSwitch::getStatusChangeFlag(){
  return status_change_flag_;
}

void ResampleStatusSwitch::holdStatusForSometime(double dur, double match_ratio)
{
  static ros::Time last_time = ros::Time::now();

  if((ros::Time::now() - last_time).toSec() < dur)
  {
    match_ratio_ = match_ratio; 
    status_change_flag_ = false;
  }else{
    last_time = ros::Time::now();
    status_change_flag_ = true;

  }
  
}
bool ResampleStatusSwitch::needStopResample()
{
  static ros::Time resample_occur_time = ros::Time::now();
  bool tt = stop_resample_flag_;
  bool ratio_flag;

  if(match_ratio_ < 0.6)
  {
    ratio_flag = true;
  }else if(match_ratio_ > 0.7){
    ratio_flag = false;
  }

  tt ^= ratio_flag;
  if(tt)
  {
    //如果从不重采样切换到重采样时,要确保5s的时间内都满足重采样条件才重采样.避免地图中出现不连续的特征.
    if(!ratio_flag)
    { 
      if((ros::Time::now() - resample_occur_time).toSec() < 5)
      {
        ratio_flag = true;
      }
    }else{
      resample_occur_time = ros::Time::now();
    }
  }else{
    resample_occur_time = ros::Time::now();
  }

  stop_resample_flag_ = ratio_flag;
  ROS_INFO("stop resample flag = %d", stop_resample_flag_);
  return stop_resample_flag_;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "resample_status");
  ResampleStatusSwitch state_switch;
  vector<double> time_set{1,3,6,20,3,8,28,3,3,7};
  ros::Rate r(20);
  srand((unsigned int)time(NULL));
  state_switch.holdStatusForSometime(time_set[0], 0.3);
  while(ros::ok())
  { 
    double val = (double)rand()/RAND_MAX;
    ROS_INFO("random val = %f", val);
    int index = rand()%(time_set.size());
    double ratio;
    if(val > 0.5)
    {
      ratio = 0.8;
    }else{
      ratio = 0.5;
    }
    ROS_INFO("time duration = %f, ratio = %f", time_set[index], ratio);
    if(state_switch.getStatusChangeFlag())
    {
      state_switch.holdStatusForSometime(time_set[index], ratio);
    }
    state_switch.needStopResample();
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
