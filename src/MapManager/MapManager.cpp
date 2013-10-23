#include <ros/ros.h>
#include <ros/package.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>

#include <ctime>
#include <iostream>

bool print_msgs = false;
bool display_features = false;

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& kcloud_msg, const sensor_msgs::PointCloud2::ConstPtr& fcloud_msg);

ros::Publisher pub_cloud;

class Pose
{
    public:
        float x;
        float y;
        float th;
};

class PointCloudRecord
{
    public:
        bool is_loaded;
        std::string kcloud_filename;
        std::string fcloud_filename;
        pcl::PCLPointCloud2::Ptr kcloud_ptr;
        pcl::PCLPointCloud2::Ptr fcloud_ptr;

    //load_cloud();
    //store_cloud();
};

Pose current_pose;
int loaded_clouds = 0;
int last_unloaded_cloud = -1;

std::vector<Pose> poses; // FLANN Matrix compatible, needs to be able to grow...
std::vector<PointCloudRecord> clouds; // Need to be able access using FLANN indices, need to maintain a separate record for keeping track of used clouds

char buffer[255];
char path[255];

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "hybridslam_map_manager");
    ros::NodeHandle nh;

    // Create ROS subscribers for sensor data
    ros::Subscriber sub_pose  = nh.subscribe ("estimated_pose", 1, pose_cb);

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_kcloud(nh, "camera/depth_registered/points"/*"kinect_cloud"*/, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_fcloud(nh, "feature_cloud", 1);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_kcloud, sub_fcloud);
    sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

    // Create data folder
    time_t rawtime;
    struct tm* timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 19, "%F-(%H:%M)", timeinfo);
    sprintf(path, "%s/data/experiments/%s", ros::package::getPath("hybridslam").c_str(), buffer);

    mkdir(path, 0755);

    // Spin
    ros::spin ();
}

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    int seq = pose_msg->header.seq;
    int time_sec = pose_msg->header.stamp.sec;
    int time_nsec = pose_msg->header.stamp.nsec;
    double x = pose_msg->pose.pose.position.x;
    double y = pose_msg->pose.pose.position.y;
    double qx = pose_msg->pose.pose.orientation.x;
    double qy = pose_msg->pose.pose.orientation.y;
    double qz = pose_msg->pose.pose.orientation.z;
    double qw = pose_msg->pose.pose.orientation.w;
    double th = (180/M_PI)*tf::getYaw(tf::Quaternion(qx,qy,qz,qw));

    if(print_msgs)
    {
        ROS_INFO("Pose Data\n");
        ROS_INFO("seq: %d", seq);
        ROS_INFO("timestamp: %d.%d", time_sec, time_nsec);
        ROS_INFO("x: %lf", x);
        ROS_INFO("y: %lf", y);
        ROS_INFO("th: %lf (q: %lf %lf %lf %lf)", th, qx, qy, qz, qw);
        ROS_INFO("------------------------------------------------------");
    }

    current_pose.x = x;
    current_pose.y = y;
    current_pose.th = th;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& kcloud_msg, const sensor_msgs::PointCloud2::ConstPtr& fcloud_msg)
{
    int kseq = kcloud_msg->header.seq;
    int ktime_sec = kcloud_msg->header.stamp.sec;
    int ktime_nsec = kcloud_msg->header.stamp.nsec;
    std::string kframe_id = kcloud_msg->header.frame_id;
    int kwidth = kcloud_msg->width;
    int kheight = kcloud_msg->height;
    bool kis_bigendian = kcloud_msg->is_bigendian;
    int kpoint_step = kcloud_msg->point_step;
    int krow_step = kcloud_msg->row_step;
    bool kis_dense = kcloud_msg->is_dense;

    if(print_msgs)
    {
        ROS_INFO("Feature point cloud\n");
        ROS_INFO("seq: %d", kseq);
        ROS_INFO("timestamp: %d.%d", ktime_sec, ktime_nsec);
        ROS_INFO("frame_id: %s", kframe_id.c_str());
        ROS_INFO("dimensions: %dx%d", kwidth, kheight);
        ROS_INFO("is_bigendian: %d", kis_bigendian);
        ROS_INFO("point_step: %d\n", kpoint_step);
        ROS_INFO("row_step: %d\n", krow_step);
        ROS_INFO("is_dense: %d\n", kis_dense);
        ROS_INFO("------------------------------------------------------");
    }

    int fseq = fcloud_msg->header.seq;
    int ftime_sec = fcloud_msg->header.stamp.sec;
    int ftime_nsec = fcloud_msg->header.stamp.nsec;
    std::string fframe_id = fcloud_msg->header.frame_id;
    int fwidth = fcloud_msg->width;
    int fheight = fcloud_msg->height;
    bool fis_bigendian = fcloud_msg->is_bigendian;
    int fpoint_step = fcloud_msg->point_step;
    int frow_step = fcloud_msg->row_step;
    bool fis_dense = fcloud_msg->is_dense;

    if(print_msgs)
    {
        ROS_INFO("Kinect point cloud\n");
        ROS_INFO("seq: %d", fseq);
        ROS_INFO("timestamp: %d.%d", ftime_sec, ftime_nsec);
        ROS_INFO("frame_id: %s", fframe_id.c_str());
        ROS_INFO("dimensions: %dx%d", fwidth, fheight);
        ROS_INFO("is_bigendian: %d", fis_bigendian);
        ROS_INFO("point_step: %d\n", fpoint_step);
        ROS_INFO("row_step: %d\n", frow_step);
        ROS_INFO("is_dense: %d\n", fis_dense);
        ROS_INFO("------------------------------------------------------");
    }

    pcl::PCLPointCloud2::Ptr kcloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr fcloud(new pcl::PCLPointCloud2);

    pcl_conversions::toPCL(*kcloud_msg, *kcloud);
    pcl_conversions::toPCL(*fcloud_msg, *fcloud);

    PointCloudRecord pcr;

    pcr.is_loaded = true;

    sprintf(buffer, "%s/kcloud-%d-%d.pcd", path, ktime_sec, ktime_nsec);
    pcr.kcloud_filename = std::string(buffer);

    sprintf(buffer, "%s/fcloud-%d-%d.pcd", path, ftime_sec, ftime_nsec);
    pcr.fcloud_filename = std::string(buffer);

    pcr.kcloud_ptr = kcloud;
    pcr.fcloud_ptr = fcloud;

    poses.push_back(current_pose);
    clouds.push_back(pcr);

    ROS_INFO("# poses: %ld", poses.size());
    ROS_INFO("# clouds: %ld", clouds.size());

    loaded_clouds++;

    if(loaded_clouds > 300)
    {
        // unload oldest cloud to disk?
        last_unloaded_cloud++;
        clouds[last_unloaded_cloud].is_loaded = false;

        pcl::PCDWriter pcd_writer;

        pcd_writer.writeBinary(clouds[last_unloaded_cloud].kcloud_filename, *clouds[last_unloaded_cloud].kcloud_ptr);

        ROS_INFO("Saved point cloud '%s'", clouds[last_unloaded_cloud].kcloud_filename.c_str());

        clouds[last_unloaded_cloud].kcloud_ptr.reset();

        pcd_writer.writeBinary(clouds[last_unloaded_cloud].fcloud_filename, *clouds[last_unloaded_cloud].fcloud_ptr);

        ROS_INFO("Saved point cloud '%s'", clouds[last_unloaded_cloud].fcloud_filename.c_str());

        clouds[last_unloaded_cloud].fcloud_ptr.reset();
    }
}
