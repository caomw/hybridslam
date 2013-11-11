#include <Filter/Filter.h>

// vector contains poses... that's it. prediction updates current pose
Filter::prediction_step(Vector u)
{
	Vector x_old = state_.x();
	Matrix Sx_old = state_.Sx();

	Matrix Fx = motion_model_.Fx(x,u,dt);
	Matrix Fu = motion_model_.Fu(x,u,dt);
	Matrix Su = motion_model_.Su(u,dt);

	// EKF
	Vector x_new = Fx*x_old + Fu*u;
	Matrix Sx_new = Fx*Sx_old*Fx.transpose + Gu*Su*Gu.transpose();

	state_.update(x_new, Sx_new);
}

// measurements are points in some coordinate frame. expected is from past regions... correspondences through ransac.
// no matching? just transform according to poses and find the difference
Filter::correction_step(Vector z)
{
	Vector x_old = state_.x();
	Matrix Sx_old = state_.Sx();

	// EKF
	if(new_feature)
	{
		augment_state(state_, z);
	}

	Matrix Hx = measurement_model_.Hx(x,z);
	Matrix Sz = measurement_model_.Sz(x,z);
	Vector z_expected = measurement_model_.z_expected(x,z_index);

	Matrix K = Sx_old*Hx.transpose()*(Hx*Sx_old*Hx.transpose()+Sz).inverse();
	Vector x_new = x_old + K*(z-z_expected);
	Matrix Sx_new = (I-K*Hx)*Sx_old;

	state_.update(x_new, Sx_new);
}
