#ifndef TF_COMMON_H
#define TF_COMMON_H

#include "ros/ros.h"
#include <tf/transform_listener.h> 

#include <Eigen/Dense>
#include <Eigen/Geometry>

class MyTF{
public:
  MyTF(const std::string base_frame_id, const std::string child_frame_id, const double timeout);
  ~MyTF();

  bool lookTransform();
  Eigen::Matrix3d getRotationMatrix();
  Eigen::Vector3d getEulerAngle();
  Eigen::Vector3d getTranslation();

private:
  Eigen::Matrix3d rot_matrix_;
  Eigen::Vector3d angle_;
  Eigen::Vector3d trans_;
  Eigen::Affine3d SE3_;

  std::string parent_frame_;
  std::string child_frame_;

  double time_out_;

  ros::NodeHandle n_;
  ros::NodeHandle private_n_;

  tf::TransformListener tf_listener_;
};

#endif