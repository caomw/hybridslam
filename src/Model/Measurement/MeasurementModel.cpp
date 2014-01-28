/*
 * MeasurementModel.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: georgebrindeiro
 */

#include <Model/Measurement/MeasurementModel.h>

#include <cmath>
#include <cstdio>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

bool map_init = false;

pcl::PointCloud<pcl::PointXYZ> map;
pcl::PointCloud<pcl::PointXYZ> z;

bool MeasurementModel::preprocess(Vector x, const sensor_msgs::PointCloud2::ConstPtr& z_msg/*Vector z*/)
{
	n_ = x.size();
	m_ = z_msg->width;
	dz_.resize(m_);
	Sz_.resize(m_,m_);

	//printf("n=%d\n", n_);
	//printf("m=%d\n", m_);
	//printf("len(dz)=%ld\n", dz_.size());
	//printf("dim(Sz)=(%ld,%ld)\n", Sz_.rows(), Sz_.cols());

	// Retrieve k clouds closest to current pose estimate from MapManager (only hybrid version)
	// In non-hybrid version, we only have one global cloud

	printf("map size=%d\n", map.width);

	if(!map_init)
	{
		pcl::fromROSMsg(*z_msg, map); // this map will originally come from MapManager
		map_init = true;

		return false;
	}
	else
	{
		pcl::fromROSMsg(*z_msg, z);
	}

	// Determine which features are in clouds already and which are not
	// 1. Get cloud from set in current frame
	Vector offset(3);
	offset << x(0), x(1), 0;

	/*tf::Quaternion
	Quaternion rotation();
	rotation.
	Quaternion rotation(u.pose.pose.orientation.w, u.pose.pose.orientation.x, u.pose.pose.orientation.y, u.pose.pose.orientation.z);
	pcl::transformPointCloud(cloud, transformed_cloud, offset, rotation.conjugate());

	pcl::toROSMsg(transformed_cloud, z);*/

	// 2. Build cloud from measurement and transform to current frame using pose estimate
	// measurement is already cloud. just transform using current pose in x.

	// 3. RANSAC and identify correspondences
	// look for this in my examples

	// Augment the state vector by the number of new features
	/*if(new_feature)
	{
		augment_state(state_, z);
	}*/

	// Compute differences between matched points and store in dz_

	// How do I get measurement noise to store in Sz_? What is the best model? We could apply something simpler at first then ccny_rgbd's model

	// Measurement vector dimensions = sum of number of matches for each cloud
}

Vector MeasurementModel::dz()
{
	// Difference to expected measurement (z-h(x)). Comes from RANSAC matches to points in z vector
	return dz_;
}

Matrix MeasurementModel::Hx()
{
	// Is this an identity due to converted measurement?
	//return Matrix::Identity(m_,m_);
	return Matrix::Zero(m_,n_);
}

Matrix MeasurementModel::Sz()
{
	// Measurement noise of each match? Does this come from RANSAC?
	return Sz_;
}
