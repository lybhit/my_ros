#include "tf_tutorial/tf_common.h"

MyTF::MyTF(const std::string base_frame_id, const std::string child_frame_id, const double timeout)
: parent_frame_(base_frame_id),
  child_frame_(child_frame_id),
  time_out_(timeout){}

MyTF::~MyTF(){

}

bool MyTF::lookTransform(){
    bool ret = false;                                                                                                                                   
    try {                                                                                                                                               
       tf::StampedTransform tf_tmp;                                                                                                                     
       tf_listener_.waitForTransform(parent_frame_, child_frame_, ros::Time(0), ros::Duration(time_out_));                                              
       tf_listener_.lookupTransform(parent_frame_, child_frame_, ros::Time(0), tf_tmp); 
                                                                       
       double roll, pitch, yaw;                                                                                                                         
       auto rotation_matrix = tf::Matrix3x3(tf_tmp.getRotation());                                                                                      
       rotation_matrix.getRPY(roll, pitch, yaw);

       angle_ << roll, pitch, yaw;
       
       auto &origin = tf_tmp.getOrigin();                                                                                                                             
       auto quat = tf_tmp.getRotation();                                                                                                                              
       trans_ << origin.x(), origin.y(), origin.z(); 
                                                                                    
       Eigen::Quaterniond quat_eigen(quat.w(), quat.x(), quat.y(), quat.z());
       
       rot_matrix_ = quat_eigen.toRotationMatrix();                                                                                    

       SE3_ = Eigen::Translation3d(trans_) * Eigen::AngleAxisd(quat_eigen) * Eigen::Scaling(1.0);                                       
                                                                                                                                       
       ret = true;                                                                                                                     
     } catch (tf::TransformException ex) {                                                                                             
     ROS_WARN("%s", ex.what());                                                                                                        
   }                                                                                                                                   
   return ret; 
}

Eigen::Matrix3d MyTF::getRotationMatrix(){
  return rot_matrix_;
}

Eigen::Vector3d MyTF::getEulerAngle(){
  return angle_;
}

Eigen::Vector3d MyTF::getTranslation(){
  return trans_;
}
