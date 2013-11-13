#include <Types/Typedefs.h>

class MeasurementModel
{
	public:
		MeasurementModel()
		{
		}

		bool preprocess(Vector x, Vector u);

		Vector h();

		Matrix Hx();

		Matrix Sz();

	private:

};
