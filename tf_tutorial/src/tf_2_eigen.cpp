#include "ros/ros.h"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_eigen_convert");
  
  geometry_msgs::TransformStamped a;

  tf2::Quanternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, 1.0);
  geometry_msgs::Quaternion quat_msg;
  tf2::convert(quat_tf, quat_msg);
  a.transform.rotation = quat_msg;
  a.transform.translation.x = 1;
  a.transform.translation.y = 1;
  a.transform.translation.z = 1;
  //tf2::Transform t(quat_tf, tf2::Vector3(1,1,1));
  
  Eigen::Affine3d result;
  Eigen::Quaternionf quater;
  quater.x() = 0;
  quater.y() = 0;
  quater.z() = 0;
  quater.w() = 1;

  Eigen::Translation3f translation(0,0,0);


  tf2::doTransform(Eigen::Affine3d::Identity(), result, a);

  std::cout << result << endl;

  return 0;  


}
