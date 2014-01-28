/**
 * @file   MotionModel.cpp
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Nov 12, 2013
 *
 * @ingroup models
 *
 * @attention Copyright (C) 2013
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <Model/Motion/MotionModel.h>

#include <cmath>
#include <cstdio>
#include <tf/transform_datatypes.h>

bool MotionModel::preprocess_new(Vector x, const nav_msgs::Odometry::ConstPtr& u_msg/*Vector u*/)
{
	Vector up(3);

	up(0) = u_msg->pose.pose.position.x;
	up(1) = u_msg->pose.pose.position.y;
	up(2) = u_msg->pose.pose.position.z;

	Quaternion uq;

	uq.x() = u_msg->pose.pose.orientation.x;
	uq.y() = u_msg->pose.pose.orientation.y;
	uq.z() = u_msg->pose.pose.orientation.z;
	uq.w() = u_msg->pose.pose.orientation.w;

	if(odom_init_ == false)
	{
		printf("first!\n");

		p_bar_new_ = up;
		q_bar_new_ = uq;

		odom_init_ = true;

		return false;
	}
	else
	{
		p_hat_old_(0) = x(0);
		p_hat_old_(1) = x(1);
		p_hat_old_(2) = x(2);
		q_hat_old_.x() = x(3);
		q_hat_old_.y() = x(4);
		q_hat_old_.z() = x(5);
		q_hat_old_.w() = x(6);

		p_bar_old_ = p_bar_new_;
		q_bar_old_ = q_bar_new_;

		p_bar_new_ = up;
		q_bar_new_ = uq;

		dp_bar_ = p_bar_new_ - p_bar_old_;
		dq_bar_ = q_bar_new_*q_bar_old_.conjugate();

		return true;
	}
}

Vector MotionModel::f_new()
{
	Vector p_hat_new(3);

	p_hat_new = p_hat_old_ + dp_bar_;

	Quaternion q_hat_new;

	q_hat_new = q_hat_old_*dq_bar_;

	Vector v = Vector(7);
	v << p_hat_new(0), p_hat_new(1), p_hat_new(2), q_hat_new.x(), q_hat_new.y(), q_hat_new.z(), q_hat_new.w();

	return v;
}

Matrix MotionModel::Fx_new()
{
	Matrix Fx = Matrix::Identity(3,3);

	double th_dr1 = th_hat_old_+dr1_;

	Fx(0,2) = -dtr_*sin(th_dr1);
	Fx(1,2) =  dtr_*cos(th_dr1);

	std::cout << "Fx\n" << Fx << std::endl;

	return Fx;
}

Matrix MotionModel::Fu_new()
{
	Matrix Fu = Matrix::Zero(3,3);

	double th_dr1 = th_hat_old_+dr1_;

	Fu(0,0) = -dtr_*sin(th_dr1);
	Fu(1,0) =  dtr_*cos(th_dr1);
	Fu(2,0) = 1;
	Fu(0,1) = cos(th_dr1);
	Fu(1,1) = sin(th_dr1);
	Fu(2,2) = 1;

	std::cout << "Fu\n" << Fu << std::endl;

	return Fu;
}

Matrix MotionModel::Su_new()
{
	Matrix Su = Matrix::Zero(3,3);

	double dr1_sq = dr1_*dr1_;
	double dtr_sq = dtr_*dtr_;
	double dr2_sq = dr2_*dr2_;

	Su(0,0) = a1_*dr1_sq + a2_*dtr_sq;
	Su(1,1) = a3_*dtr_sq + a4_*dr1_sq + a4_*dr2_sq;
	Su(2,2) = a1_*dr2_sq + a2_*dtr_sq;

	std::cout << "Su\n" << Su << std::endl;

	return Su;
}

bool MotionModel::preprocess(Vector x, const nav_msgs::Odometry::ConstPtr& u_msg/*Vector u*/)
{
	double ux = u_msg->pose.pose.position.x;
	double uy = u_msg->pose.pose.position.y;
	double qx = u_msg->pose.pose.orientation.x;
	double qy = u_msg->pose.pose.orientation.y;
	double qz = u_msg->pose.pose.orientation.z;
	double qw = u_msg->pose.pose.orientation.w;
	double uth = tf::getYaw(tf::Quaternion(qx,qy,qz,qw));

	Vector u(3);

	u(0) = ux;
	u(1) = uy;
	u(2) = uth;

	if(odom_init_ == false)
	{
		printf("first!\n");

		x_bar_new_  = u(0);
		y_bar_new_  = u(1);
		th_bar_new_ = u(2);

		odom_init_ = true;

		return false;
	}
	else
	{
		x_hat_old_  = x(0);
		y_hat_old_  = x(1);
		th_hat_old_ = x(2);

		x_bar_old_  = x_bar_new_;
		y_bar_old_  = y_bar_new_;
		th_bar_old_ = th_bar_new_;

		x_bar_new_  = u(0);
		y_bar_new_  = u(1);
		th_bar_new_ = u(2);

		dx_bar_  = x_bar_new_  - x_bar_old_;
		dy_bar_  = y_bar_new_  - y_bar_old_;
		dth_bar_ = th_bar_new_ - th_bar_old_;

		dr1_ = atan2(dy_bar_,dx_bar_) - th_bar_old_;
		dtr_ = sqrt(dx_bar_*dx_bar_+dy_bar_*dy_bar_);
		dr2_ = dth_bar_-dr1_;

		return true;
	}
}

Vector MotionModel::f()
{
	double th_dr1 = th_hat_old_ + dr1_;

	double x_hat_new  = x_hat_old_  + dtr_*cos(th_dr1);
	double y_hat_new  = y_hat_old_  + dtr_*sin(th_dr1);
	double th_hat_new = th_hat_old_ + dr1_ + dr2_;

	Vector v = Vector(3);
	v << x_hat_new, y_hat_new, th_hat_new;

	return v;
}

Matrix MotionModel::Fx()
{
	Matrix Fx = Matrix::Identity(3,3);

	double th_dr1 = th_hat_old_+dr1_;

	Fx(0,2) = -dtr_*sin(th_dr1);
	Fx(1,2) =  dtr_*cos(th_dr1);

	std::cout << "Fx\n" << Fx << std::endl;

	return Fx;
}

Matrix MotionModel::Fu()
{
	Matrix Fu = Matrix::Zero(3,3);

	double th_dr1 = th_hat_old_+dr1_;

	Fu(0,0) = -dtr_*sin(th_dr1);
	Fu(1,0) =  dtr_*cos(th_dr1);
	Fu(2,0) = 1;
	Fu(0,1) = cos(th_dr1);
	Fu(1,1) = sin(th_dr1);
	Fu(2,2) = 1;

	std::cout << "Fu\n" << Fu << std::endl;

	return Fu;
}

Matrix MotionModel::Su()
{
	Matrix Su = Matrix::Zero(3,3);

	double dr1_sq = dr1_*dr1_;
	double dtr_sq = dtr_*dtr_;
	double dr2_sq = dr2_*dr2_;

	Su(0,0) = a1_*dr1_sq + a2_*dtr_sq;
	Su(1,1) = a3_*dtr_sq + a4_*dr1_sq + a4_*dr2_sq;
	Su(2,2) = a1_*dr2_sq + a2_*dtr_sq;

	std::cout << "Su\n" << Su << std::endl;

	return Su;
}
