#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

ros::Publisher pub_cloud;

int count = 0;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "hybridslam_sensor_processor_cloud_saver");
    ros::NodeHandle nh;

    // Create ROS subscribers for sensor data
    ros::Subscriber sub_cloud = nh.subscribe ("feature_cloud", 1, cloud_cb);

    // Spin
    ros::spin ();
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    ROS_INFO("# feature cloud points: %d", cloud_msg->width * cloud_msg->height);

    // Save feature cloud
    pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);

    pcl_conversions::toPCL(*cloud_msg, *pcl_input);

    char buff[100];
    sprintf(buff, "pointcloud_%d.pcd", count);
    std::string filename(buff);

    count++;

    pcl::PCDWriter pcd_writer;

    pcd_writer.writeASCII(filename, *pcl_input);
}
