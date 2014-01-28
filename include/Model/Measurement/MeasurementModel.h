#include <Types/Typedefs.h>

#include <sensor_msgs/PointCloud2.h>

class MeasurementModel
{
	public:
		MeasurementModel()
		{
			m_ = 0;
		}

		bool preprocess(Vector x, const sensor_msgs::PointCloud2::ConstPtr& z_msg/*Vector z*/);

		Vector dz();

		Matrix Hx();

		Matrix Sz();

	private:

		int m_;
		int n_;

		Vector dz_;

		Matrix Sz_;
};
