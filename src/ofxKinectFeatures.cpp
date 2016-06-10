#include "ofxKinectFeatures.h"

ofxKinectFeatures::ofxKinectFeatures(int head, int torso)
{
	featExtractor.setup(head, torso);
}

void ofxKinectFeatures::setup(int head, int torso)
{
	featExtractor.setup(head, torso);
}

void ofxKinectFeatures::update(map<int, ofPoint> joints)
{
	map<int, MocapPoint> jointsMap;
	for (auto joint : joints)
		jointsMap[joint.first] = toMocapPoint(joint.second);

	featExtractor.update(jointsMap);

	for (auto joint : joints) {
		checkMaxAndMin(getVelocityHistory(joint.first, 5), joint.first, FEAT_VELOCITY);
		checkMaxAndMin(getAccelerationHistory(joint.first, 5), joint.first, FEAT_ACCELERATION);
		checkMaxAndMin(getAccelerationTrajectoryHistory(joint.first, 5), joint.first, FEAT_ACCELERATION_TRAJECTORY);
		checkMaxAndMin(getRelativePositionToTorsoHistory(joint.first, 5), joint.first, FEAT_RELATIVEPOSTOTORSO);
	}

	checkMaxAndMin(getQomHistory(5), NO_JOINT, FEAT_QOM);
	checkMaxAndMin(getCIHistory(5), NO_JOINT, FEAT_CI);
}

void ofxKinectFeatures::setFilterLevel(int filterLevel)
{
	featExtractor.setFilterLevel(filterLevel);
}

void ofxKinectFeatures::setDepth(int depth)
{
	featExtractor.setDepth(depth);
}

int ofxKinectFeatures::getDepth()
{
	return featExtractor.getDepth();
}

ofPoint ofxKinectFeatures::getPosition(int j)
{
	return toOfPoint(featExtractor.getPosition(j));
}

vector<ofPoint> ofxKinectFeatures::getPositionHistory(int j)
{
	return toOfPointVector(featExtractor.getPositionHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getPositionHistory(int j, int frames)
{
	return toOfPointVector(featExtractor.getPositionHistory(j, frames));
}

ofPoint ofxKinectFeatures::getPositionFiltered(int j)
{
	return toOfPoint(featExtractor.getPositionFiltered(j));
}

vector<ofPoint> ofxKinectFeatures::getPositionFilteredHistory(int j)
{
	return toOfPointVector(featExtractor.getPositionFilteredHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getPositionFilteredHistory(int j, int frames)
{
	return toOfPointVector(featExtractor.getPositionFilteredHistory(j, frames));
}

ofPoint ofxKinectFeatures::getVelocity(int j)
{
	return toOfPoint(featExtractor.getVelocity(j));
}

vector<ofPoint> ofxKinectFeatures::getVelocityHistory(int j)
{
	return toOfPointVector(featExtractor.getVelocityHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getVelocityHistory(int j, int frames)
{
	return toOfPointVector(featExtractor.getVelocityHistory(j, frames));
}

float ofxKinectFeatures::getVelocityMagnitude(int j)
{
	return featExtractor.getVelocityMagnitude(j);
}

ofPoint ofxKinectFeatures::getVelocityMean(int j, int frames)
{
	return toOfPoint(featExtractor.getVelocityMean(j, frames));
}

float ofxKinectFeatures::getVelocityMagnitudeMean(int j, int frames)
{
	return featExtractor.getVelocityMagnitudeMean(j, frames);
}

ofPoint ofxKinectFeatures::getAcceleration(int j)
{
	return toOfPoint(featExtractor.getAcceleration(j));
}

vector<ofPoint> ofxKinectFeatures::getAccelerationHistory(int j)
{
	return toOfPointVector(featExtractor.getAccelerationHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getAccelerationHistory(int j, int frames)
{
	return vector<ofPoint>();
}

float ofxKinectFeatures::getAccelerationMagnitude(int j)
{
	return featExtractor.getAccelerationMagnitude(j);
}

ofPoint ofxKinectFeatures::getAccelerationMean(int j, int frames)
{
	return toOfPoint(featExtractor.getAccelerationMean(j, frames));
}

float ofxKinectFeatures::getAccelerationMagnitudeMean(int j, int frames)
{
	return featExtractor.getAccelerationMagnitudeMean(j, frames);
}

float ofxKinectFeatures::getAccelerationTrajectory(int j)
{
	return featExtractor.getAccelerationTrajectory(j);
}

vector<float> ofxKinectFeatures::getAccelerationTrajectoryHistory(int j)
{
	return featExtractor.getAccelerationTrajectoryHistory(j);
}

vector<float> ofxKinectFeatures::getAccelerationTrajectoryHistory(int j, int frames)
{
	return featExtractor.getAccelerationTrajectoryHistory(j, frames);
}

float ofxKinectFeatures::getAccelerationTrajectoryMean(int j, int frames)
{
	return featExtractor.getAccelerationTrajectoryMean(j, frames);
}

ofPoint ofxKinectFeatures::getRelativePositionToTorso(int j)
{
	return toOfPoint(featExtractor.getRelativePositionToTorso(j));
}

vector<ofPoint> ofxKinectFeatures::getRelativePositionToTorsoHistory(int j)
{
	return toOfPointVector(featExtractor.getRelativePositionToTorsoHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getRelativePositionToTorsoHistory(int j, int frames)
{
	return toOfPointVector(featExtractor.getRelativePositionToTorsoHistory(j, frames));
}

float ofxKinectFeatures::getAngle(int j1, int j2, int j3)
{
	return featExtractor.getAngle(j1, j2, j3);
}

float ofxKinectFeatures::getQom()
{
	return featExtractor.getQom();
}

vector<float> ofxKinectFeatures::getQomHistory()
{
	return featExtractor.getQomHistory();
}

vector<float> ofxKinectFeatures::getQomHistory(int frames)
{
	return featExtractor.getQomHistory(frames);
}

float ofxKinectFeatures::getCI()
{
	return featExtractor.getCI();
}

vector<float> ofxKinectFeatures::getCIHistory()
{
	return featExtractor.getCIHistory();
}

vector<float> ofxKinectFeatures::getCIHistory(int frames)
{
	return featExtractor.getCIHistory(frames);
}

bool ofxKinectFeatures::isNewDataAvailable()
{
	return featExtractor.isNewDataAvailable();
}

void ofxKinectFeatures::checkMaxAndMin(vector<ofPoint> descriptorHistory, unsigned int jointId, unsigned int feature) {
	vector<float> x_vec, y_vec, z_vec;
	//for (vector<MocapPoint>::iterator it = descriptorHistory.begin(); it != descriptorHistory.end(); it++) {
	for (auto it : descriptorHistory) {
		x_vec.push_back(it.x);
		y_vec.push_back(it.y);
		z_vec.push_back(it.z);
	}

	//x
	if (distance(x_vec.begin(), max_element(x_vec.begin(), x_vec.end())) == 2) {
		static MocapMaxEvent newMaxEvent;
		newMaxEvent.joint = jointId;
		newMaxEvent.axis = MOCAP_X;
		newMaxEvent.feature = feature;
		newMaxEvent.value = descriptorHistory[1].x;
		ofNotifyEvent(MocapMaxEvent::events, newMaxEvent);
	}
	else if (distance(x_vec.begin(), min_element(x_vec.begin(), x_vec.end())) == 2) {
		static MocapMinEvent newMinEvent;
		newMinEvent.joint = jointId;
		newMinEvent.axis = MOCAP_X;
		newMinEvent.feature = feature;
		newMinEvent.value = descriptorHistory[1].x;
		ofNotifyEvent(MocapMinEvent::events, newMinEvent);
	}
	//y
	if (distance(y_vec.begin(), max_element(y_vec.begin(), y_vec.end())) == 2) {
		static MocapMaxEvent newMaxEvent;
		newMaxEvent.joint = jointId;
		newMaxEvent.axis = MOCAP_Y;
		newMaxEvent.feature = feature;
		newMaxEvent.value = descriptorHistory[1].y;
		ofNotifyEvent(MocapMaxEvent::events, newMaxEvent);
	}
	else if (distance(y_vec.begin(), min_element(y_vec.begin(), y_vec.end())) == 2) {
		static MocapMinEvent newMinEvent;
		newMinEvent.joint = jointId;
		newMinEvent.axis = MOCAP_Y;
		newMinEvent.feature = feature;
		newMinEvent.value = descriptorHistory[1].y;
		ofNotifyEvent(MocapMinEvent::events, newMinEvent);
	}
	//z
	if (distance(z_vec.begin(), max_element(z_vec.begin(), z_vec.end())) == 2) {
		static MocapMaxEvent newMaxEvent;
		newMaxEvent.joint = jointId;
		newMaxEvent.axis = MOCAP_Y;
		newMaxEvent.feature = feature;
		newMaxEvent.value = descriptorHistory[1].z;
		ofNotifyEvent(MocapMaxEvent::events, newMaxEvent);
	}
	else if (distance(z_vec.begin(), min_element(z_vec.begin(), z_vec.end())) == 2) {
		static MocapMinEvent newMinEvent;
		newMinEvent.joint = jointId;
		newMinEvent.axis = MOCAP_Z;
		newMinEvent.feature = feature;
		newMinEvent.value = descriptorHistory[1].z;
		ofNotifyEvent(MocapMinEvent::events, newMinEvent);
	}
}

void ofxKinectFeatures::checkMaxAndMin(vector<float> descriptorHistory, unsigned int jointId, unsigned int feature) {
	if (distance(descriptorHistory.begin(), max_element(descriptorHistory.begin(), descriptorHistory.end())) == 2) {
		static MocapMaxEvent newMaxEvent;
		newMaxEvent.joint = jointId;
		newMaxEvent.feature = feature;
		newMaxEvent.value = descriptorHistory[1];
		ofNotifyEvent(MocapMaxEvent::events, newMaxEvent);
	}
	else if (distance(descriptorHistory.begin(), min_element(descriptorHistory.begin(), descriptorHistory.end())) == 2) {
		static MocapMinEvent newMinEvent;
		newMinEvent.joint = jointId;
		newMinEvent.feature = feature;
		newMinEvent.value = descriptorHistory[1];
		ofNotifyEvent(MocapMinEvent::events, newMinEvent);
	}
}

ofPoint ofxKinectFeatures::toOfPoint(MocapPoint point)
{
	return ofPoint(point.x, point.y, point.y);
}

vector<ofPoint> ofxKinectFeatures::toOfPointVector(vector<MocapPoint> pointVector)
{
	vector<ofPoint> pVector;
	for (auto p : pointVector)
		pVector.push_back(toOfPoint(p));
	return pVector;
}

MocapPoint ofxKinectFeatures::toMocapPoint(ofPoint point)
{
	return MocapPoint(point.x, point.y, point.z);
}
