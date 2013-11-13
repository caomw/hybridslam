/*
 * MotionModel.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: georgebrindeiro
 */

#include <Model/Motion/MotionModel.h>

#include <cmath>

bool MotionModel::preprocess(Vector x, Vector u)
{
	if(odom_init_ == false)
	{
		x_bar_old_  = u(0);
		y_bar_old_  = u(1);
		th_bar_old_ = u(2);

		odom_init_ = true;

		return false;
	}
	else
	{
		x_hat_old_  = x(0);
		y_hat_old_  = x(1);
		th_hat_old_ = x(2);

		x_bar_new_  = u(0);
		y_bar_new_  = u(1);
		th_bar_new_ = u(2);

		x_bar_old_  = u_(0);
		y_bar_old_  = u_(1);
		th_bar_old_ = u_(2);

		dx_bar_  = x_bar_new_  - x_bar_old_;
		dy_bar_  = y_bar_new_  - y_bar_old_;
		dth_bar_ = th_bar_new_ - th_bar_old_;

		dr1_ = atan2(dy_bar_,dx_bar_)-th_bar_old_;
		dtr_ = sqrt(dx_bar_*dx_bar_+dy_bar_*dy_bar_);
		dr2_ = dth_bar_-dr1_;

		return true;
	}
}

Vector MotionModel::f()
{
	double th_dr1 = th_hat_old_+dr1_;

	double x_hat_new  = x_hat_old_  + dtr_*cos(th_dr1);
	double y_hat_new  = y_hat_old_  + dtr_*sin(th_dr1);
	double th_hat_new = th_hat_old_ + dr1_ + dr2_;

	return Vector(x_hat_new, y_hat_new, th_hat_new);
}

Matrix MotionModel::Fx()
{
	Matrix Fx = Matrix::Identity(3,3);

	double th_dr1 = th_hat_old_+dr1_;

	Fx(0,2) = -dtr_*sin(th_dr1);
	Fx(1,2) =  dtr_*cos(th_dr1);

	return Fx;
}

Matrix MotionModel::Fu()
{
	Matrix Fu = Matrix::Zeros(3,3);

	double th_dr1 = th_hat_old_+dr1_;

	Fu(0,0) = -dtr*sin(th_dr1);
	Fu(1,0) =  dtr*cos(th_dr1);
	Fu(2,0) = 1;
	Fu(0,1) = cos(th_dr1);
	Fu(1,1) = sin(th_dr1);
	Fu(2,2) = 1;

	return Fu;
}

Matrix MotionModel::Su()
{
	Matrix Su = Matrix::Zeros(3,3);

	double dr1_sq = dr1_*dr1_;
	double dtr_sq = dtr_*dtr_;
	double dr2_sq = dr2_*dr2_;

	Su(0,0) = a1_*dr1_sq + a2_*dtr_sq;
	Su(1,1) = a3_*dtr_sq + a4_*dr1_sq + a4_*dr2_sq;
	Su(2,2) = a1_*dr2_sq + a2_*dtr_sq;

	return Su;
}
