#include <Types/Typedefs.h>

class MotionModel
{
	public:
		MotionModel(double a1, double a2, double a3, double a4)
		{
			a1_ = a1;
			a2_ = a2;
			a3_ = a3;
			a4_ = a4;

			odom_init = false;
		}

		bool preprocess(Vector x, Vector u);

		Vector f();

		Matrix Fx();

		Matrix Fu();

		Matrix Su();

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
};
