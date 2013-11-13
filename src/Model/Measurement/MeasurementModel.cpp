/*
 * MeasurementModel.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: georgebrindeiro
 */

#include <Model/Motion/MeasurementModel.h>

#include <cmath>

bool MeasurementModel::preprocess(Vector x, Vector z)
{
	// RANSAC stuff should happen here, probably
}

Vector MeasurementModel::h()
{
	// Expected measurement. Comes from RANSAC matches to points in z vector
}

Matrix MeasurementModel::Hx()
{
	// Is this an identity due to converted measurement?
}

Matrix MeasurementModel::Sz()
{
	// Measurement noise of each match? Does this come from RANSAC?
}
