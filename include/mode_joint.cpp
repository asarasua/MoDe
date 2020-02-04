#include "mode_joint.h"

int MoDe::MoDeJoint::computeDescriptors(MoDePoint jointPos, float *aFilter, float *bFilter, float* aLpd1, float* bLpd1, float* aLpd2, float * bLpd2)
{
	//Position
	addValue(MoDe::DESC_POSITION, jointPos);

	//Filtered position
	addValue(MoDe::DESC_POSITION_FILTERED, applyFilter(getDescriptor(MoDe::DESC_POSITION).getData(), getDescriptor(MoDe::DESC_POSITION_FILTERED).getData(), aFilter, bFilter));

	//Velocity
	addValue(MoDe::DESC_VELOCITY, applyFilter(getDescriptor(MoDe::DESC_POSITION).getData(), getDescriptor(MoDe::DESC_VELOCITY).getData(), aLpd1, bLpd1));

	//Acceleration
	addValue(MoDe::DESC_ACCELERATION, applyFilter(getDescriptor(MoDe::DESC_POSITION).getData(), getDescriptor(MoDe::DESC_ACCELERATION).getData(), aLpd2, bLpd2));

	//Acceleration along trajectory
	MoDePoint acc = getDescriptor(MoDe::DESC_ACCELERATION).getCurrent();
	MoDePoint vel = getDescriptor(MoDe::DESC_VELOCITY).getCurrent();
	addValue(MoDe::DESC_ACCELERATION_TRAJECTORY, acc.dot(vel) / vel.length());
	return 0;
}

void MoDe::MoDeJoint::computeRelativeJointDescriptors(MoDePoint torsoPos, const float & h)
{
	MoDe::MoDePoint jointPos = getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent();
	vector<double> relPosToTorso(3);
	relPosToTorso[0] = (jointPos.x - torsoPos.x) / (h * 1.8);
	relPosToTorso[1] = (jointPos.y - torsoPos.y) / (h * 1.8);
	relPosToTorso[2] = -((jointPos.z - torsoPos.z) / h) / 1.4;
	addValue(MoDe::DESC_RELATIVEPOSTOTORSO, relPosToTorso);
}

MoDe::MoDePoint MoDe::MoDeJoint::applyFilter(vector<MoDePoint> x, vector<MoDePoint> y, float * a, float * b)
{
	reverse(x.begin(), x.end());
	reverse(y.begin(), y.end());
	return b[0] * x[0] + b[1] * x[1] + b[2] * x[2] + b[3] * x[3] + b[4] * x[4] - (a[1] * y[0] + a[2] * y[1] + a[3] * y[2] + a[4] * y[3]);
}

map<int, vector<MoDe::MoDeExtreme>> MoDe::MoDeJoint::getNewExtrema() const
{
	map<int, vector<MoDeExtreme>> newExtrema;
	if (getDescriptor(MoDe::DESC_POSITION).getNewExtremes().size())
		newExtrema[MoDe::DESC_POSITION] = getDescriptor(MoDe::DESC_POSITION).getNewExtremes();
	if (getDescriptor(MoDe::DESC_POSITION_FILTERED).getNewExtremes().size())
		newExtrema[MoDe::DESC_POSITION_FILTERED] = getDescriptor(MoDe::DESC_POSITION_FILTERED).getNewExtremes();
	if (getDescriptor(MoDe::DESC_VELOCITY).getNewExtremes().size())
		newExtrema[MoDe::DESC_VELOCITY] = getDescriptor(MoDe::DESC_VELOCITY).getNewExtremes();
	if (getDescriptor(MoDe::DESC_ACCELERATION).getNewExtremes().size())
		newExtrema[MoDe::DESC_ACCELERATION] = getDescriptor(MoDe::DESC_ACCELERATION).getNewExtremes();
	if (getUniDescriptor(MoDe::DESC_ACCELERATION_TRAJECTORY).getNewExtremes().size())
		newExtrema[MoDe::DESC_ACCELERATION_TRAJECTORY] = getUniDescriptor(MoDe::DESC_ACCELERATION_TRAJECTORY).getNewExtremes();

	return newExtrema;
}