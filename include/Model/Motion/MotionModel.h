/**
 * @file   MotionModel.h
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Nov 12, 2013
 *
 * @ingroup models
 *
 * @attention Copyright (C) 2013
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <Types/Typedefs.h>

#include <nav_msgs/Odometry.h>

#include <cstdio>

class MotionModel
{
	public:
		MotionModel(double a1, double a2, double a3, double a4)
		{
			a1_ = a1;
			a2_ = a2;
			a3_ = a3;
			a4_ = a4;

			odom_init_ = false;
		}

		bool preprocess(Vector x, const nav_msgs::Odometry::ConstPtr& u_msg/*Vector u*/);

		Vector f();

		Matrix Fx();

		Matrix Fu();

		Matrix Su();

		bool preprocess_new(Vector x, const nav_msgs::Odometry::ConstPtr& u_msg/*Vector u*/);

		Vector f_new();

		Matrix Fx_new();

		Matrix Fu_new();

		Matrix Su_new();

	private:
		double a1_;
		double a2_;
		double a3_;
		double a4_;

		bool odom_init_;

		double x_hat_old_;
		double y_hat_old_;
		double th_hat_old_;

		double x_bar_new_;
		double y_bar_new_;
		double th_bar_new_;

		double x_bar_old_;
		double y_bar_old_;
		double th_bar_old_;

		double dx_bar_;
		double dy_bar_;
		double dth_bar_;

		double dr1_;
		double dtr_;
		double dr2_;

		Vector p_hat_old_;
		Quaternion q_hat_old_;

		Vector p_bar_new_;
		Quaternion q_bar_new_;

		Vector p_bar_old_;
		Quaternion q_bar_old_;

		Vector dp_bar_;
		Quaternion dq_bar_;
};
