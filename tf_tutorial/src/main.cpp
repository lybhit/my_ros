#include "tf_tutorial/tf_common.h"
#include "glog/logging.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_tutorial");
    ros::NodeHandle n;
    ros::NodeHandle private_n("~");

    FLAGS_log_dir = "/tmp";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    std::string base_frame;
    std::string child_frame;
    double timeout;

    private_n.param<std::string>("base_frame", base_frame, "base_link");
    private_n.param<std::string>("child_frame", child_frame, "laser_link");
    private_n.param<double>("time_out", timeout, 1);

    MyTF my_tf(base_frame, child_frame, timeout);

    Eigen::Matrix3d rot_matrix;
    Eigen::Vector3d euler_angle;
    Eigen::Vector3d trans;

    ros::Rate r(1);

    while(ros::ok()){
        if(my_tf.lookTransform()){
            rot_matrix = my_tf.getRotationMatrix();
            euler_angle = my_tf.getEulerAngle();
            trans = my_tf.getTranslation();

            LOG(INFO) << "ROT Matrix info:";
            LOG(INFO) << rot_matrix;
            LOG(INFO) << "Euler angle info:";
            LOG(INFO) << euler_angle;
            LOG(INFO) << "Translation info:";
            LOG(INFO) << trans;
        }else{
            LOG(WARNING) << "Can not get tf info!";
        }

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}