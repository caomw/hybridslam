#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>

#include <iostream>

bool print_msgs = false;
bool display_features = false;

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg);
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

ros::Publisher pub_cloud;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "hybridslam_sensor_processor");
    ros::NodeHandle nh;

    // Create ROS subscribers for sensor data
    ros::Subscriber sub_odom  = nh.subscribe ("odometry",     1, odom_cb);
    ros::Subscriber sub_cloud = nh.subscribe ("kinect_cloud", 1, cloud_cb);

    // Create a ROS publisher for processed sensor data
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("feature_cloud", 1);

    if(display_features)
        cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);

    // Spin
    ros::spin ();
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    int seq = odom_msg->header.seq;
    int time_sec = odom_msg->header.stamp.sec;
    int time_nsec = odom_msg->header.stamp.nsec;
    double x = odom_msg->pose.pose.position.x;
    double y = odom_msg->pose.pose.position.y;
    double qx = odom_msg->pose.pose.orientation.x;
    double qy = odom_msg->pose.pose.orientation.y;
    double qz = odom_msg->pose.pose.orientation.z;
    double qw = odom_msg->pose.pose.orientation.w;
    double th = (180/M_PI)*tf::getYaw(tf::Quaternion(qx,qy,qz,qw));
    double v = odom_msg->twist.twist.linear.x;
    double w = odom_msg->twist.twist.angular.z;

    if(print_msgs)
    {
        ROS_INFO("Odometry Data\n");
        ROS_INFO("seq: %d", seq);
        ROS_INFO("timestamp: %d.%d", time_sec, time_nsec);
        ROS_INFO("x: %lf", x);
        ROS_INFO("y: %lf", y);
        ROS_INFO("th: %lf (q: %lf %lf %lf %lf)", th, qx, qy, qz, qw);
        ROS_INFO("v: %lf", v);
        ROS_INFO("w: %lf", w);
        ROS_INFO("------------------------------------------------------");
    }

    // Should I be publishing this to the filter?
    // Update with odometry:
    // x = x + delta_trans*cos(th+delta_rot1)
    // y = y + delta_trans*sin(th+delta_rot1)
    // th = th + delta_rot1 + delta_rot2
    // Need to work out the math for covariance update
    /*static double x_old = 0;
    static double y_old = 0;
    static double th_old = 0;

    double dx = x-x_old;
    double dy = y-y_old;
    double dth = th-th_old;

    double delta_rot1 = atan2(dy, dx)-th_old; 
    double delta_trans = sqrt(dx*dx+dy*dy);
    double delta_rot2 = dth-delta_rot1;

    x_old = x;
    y_old = y;
    th_old = th;*/
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    int seq = cloud_msg->header.seq;
    int time_sec = cloud_msg->header.stamp.sec;
    int time_nsec = cloud_msg->header.stamp.nsec;
    std::string frame_id = cloud_msg->header.frame_id;
    int width = cloud_msg->width;
    int height = cloud_msg->height;
    bool is_bigendian = cloud_msg->is_bigendian;
    int point_step = cloud_msg->point_step;
    int row_step = cloud_msg->row_step;
    bool is_dense = cloud_msg->is_dense;

    if(print_msgs)
    {
        ROS_INFO("Point cloud\n");
        ROS_INFO("seq: %d", seq);
        ROS_INFO("timestamp: %d.%d", time_sec, time_nsec);
        ROS_INFO("frame_id: %s", frame_id.c_str());
        ROS_INFO("dimensions: %dx%d", width, height);
        ROS_INFO("is_bigendian: %d", is_bigendian);
        ROS_INFO("point_step: %d\n", point_step);
        ROS_INFO("row_step: %d\n", row_step);
        ROS_INFO("is_dense: %d\n", is_dense);
        ROS_INFO("------------------------------------------------------");
    }

    // Extract RGB image from cloud
    sensor_msgs::Image image_msg;
    cv_bridge::CvImagePtr cv_image_ptr;
    cv::Mat img;

    pcl::toROSMsg(*cloud_msg, image_msg);
    cv_image_ptr = cv_bridge::toCvCopy(image_msg);
    img = cv_image_ptr->image;

    // Detect the keypoints using SURF Detector
    std::vector<cv::KeyPoint> keypoints;
    int minHessian = 400;

    cv::SurfFeatureDetector detector(minHessian);
    detector.detect(img, keypoints);

    // Print out number of features
    ROS_INFO("# features: %ld", keypoints.size());

    if(display_features)
    {
        // Overlay keypoints on image
        cv::drawKeypoints(img, keypoints, img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
      
        // Show results in display window
        cv::imshow("Display window", img);
        cv::waitKey(10);
    }

    // Build feature cloud
    pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr pcl_output(new pcl::PCLPointCloud2);

    pcl::PointIndices::Ptr keypoint_indices (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;

    pcl_conversions::toPCL(*cloud_msg, *pcl_input);

    for(int i=0; i < keypoints.size(); i++)
    {
        int x = keypoints[i].pt.x;//1-640
        int y = keypoints[i].pt.y;//1-480

        keypoint_indices->indices.push_back(width*y+x);
    }

    extract.setInputCloud(pcl_input);
    extract.setIndices(keypoint_indices);
    extract.setNegative(false);
    extract.filter(*pcl_output);

    ROS_INFO("# feature cloud points: %d", pcl_output->width * pcl_output->height);

    // Publish the data
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
    
    pcl_conversions::fromPCL(*pcl_output, *output);

    pub_cloud.publish (output);
}
