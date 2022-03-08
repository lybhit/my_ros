#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <math.h>


int main(int argc, char **argv) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    
    cloud_in->width = 5;
    cloud_in->height = 1;
    
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    
    for(int i = 0; i < cloud_in->points.size(); i++)
    {
        cloud_in->points[i].x = 1024 * rand()/(RAND_MAX + 1.0F);; 
        cloud_in->points[i].y = 1024 * rand()/(RAND_MAX + 1.0F);; 
        cloud_in->points[i].z = 0; 
        
    }
    
    *cloud_out = *cloud_in;
    for(int i = 0; i < cloud_in->points.size(); i++)
    {
        cloud_out->points[i].x = cos(M_PI/6) * cloud_in->points[i].x - sin(M_PI/6) * cloud_in->points[i].y; 
        cloud_out->points[i].y = sin(M_PI/6) * cloud_in->points[i].x + cos(M_PI/6) * cloud_in->points[i].y; 
        cloud_out->points[i].z = 0; 
    }
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*unused_result);
    
    
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f tCorrect;
    tCorrect = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    
    std::cout << "yaw to target = " << yaw << std::endl;
    
    
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
