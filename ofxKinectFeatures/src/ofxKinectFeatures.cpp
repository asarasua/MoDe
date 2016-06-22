#include "ofxKinectFeatures.h"

//ofxKinectFeatures::ofxKinectFeatures(int head, int torso)
//{
//	featExtractor->setup(head, torso);
//}

void ofxKinectFeatures::setup(int head, int torso)
{
	featExtractor->setup(head, torso, 60);
}

void ofxKinectFeatures::update(map<int, ofPoint> joints)
{
	map<int, MocapPoint> jointsMap;
	for (auto joint : joints)
		jointsMap[joint.first] = toMocapPoint(joint.second);

	featExtractor->update(jointsMap);
}

void ofxKinectFeatures::newBeat(MocapBeat beat)
{
	static MocapEvent newEvent;
	newEvent.joint = beat.joint;
	newEvent.axis = beat.axis;
	newEvent.feature = beat.feature;
	newEvent.value = beat.value;
	newEvent.beatType = beat.beatType;

	ofNotifyEvent(MocapEvent::events, newEvent);
}

void ofxKinectFeatures::setFilterLevel(int filterLevel)
{
	featExtractor->setFilterLevel(filterLevel);
}

void ofxKinectFeatures::setDepth(int depth)
{
	featExtractor->setDepth(depth);
}

int ofxKinectFeatures::getDepth()
{
	return featExtractor->getDepth();
}

ofPoint ofxKinectFeatures::getPosition(int j)
{
	return toOfPoint(featExtractor->getPosition(j));
}

vector<ofPoint> ofxKinectFeatures::getPositionHistory(int j)
{
	return toOfPointVector(featExtractor->getPositionHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getPositionHistory(int j, int frames)
{
	return toOfPointVector(featExtractor->getPositionHistory(j, frames));
}

ofPoint ofxKinectFeatures::getPositionFiltered(int j)
{
	return toOfPoint(featExtractor->getPositionFiltered(j));
}

vector<ofPoint> ofxKinectFeatures::getPositionFilteredHistory(int j)
{
	return toOfPointVector(featExtractor->getPositionFilteredHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getPositionFilteredHistory(int j, int frames)
{
	return toOfPointVector(featExtractor->getPositionFilteredHistory(j, frames));
}

ofPoint ofxKinectFeatures::getVelocity(int j)
{
	return toOfPoint(featExtractor->getVelocity(j));
}

vector<ofPoint> ofxKinectFeatures::getVelocityHistory(int j)
{
	return toOfPointVector(featExtractor->getVelocityHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getVelocityHistory(int j, int frames)
{
	return toOfPointVector(featExtractor->getVelocityHistory(j, frames));
}

float ofxKinectFeatures::getVelocityMagnitude(int j)
{
	return featExtractor->getVelocityMagnitude(j);
}

ofPoint ofxKinectFeatures::getVelocityMean(int j, int frames)
{
	return toOfPoint(featExtractor->getVelocityMean(j, frames));
}

float ofxKinectFeatures::getVelocityMagnitudeMean(int j, int frames)
{
	return featExtractor->getVelocityMagnitudeMean(j, frames);
}

ofPoint ofxKinectFeatures::getAcceleration(int j)
{
	return toOfPoint(featExtractor->getAcceleration(j));
}

vector<ofPoint> ofxKinectFeatures::getAccelerationHistory(int j)
{
	return toOfPointVector(featExtractor->getAccelerationHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getAccelerationHistory(int j, int frames)
{
	return vector<ofPoint>();
}

float ofxKinectFeatures::getAccelerationMagnitude(int j)
{
	return featExtractor->getAccelerationMagnitude(j);
}

ofPoint ofxKinectFeatures::getAccelerationMean(int j, int frames)
{
	return toOfPoint(featExtractor->getAccelerationMean(j, frames));
}

float ofxKinectFeatures::getAccelerationMagnitudeMean(int j, int frames)
{
	return featExtractor->getAccelerationMagnitudeMean(j, frames);
}

float ofxKinectFeatures::getAccelerationTrajectory(int j)
{
	return featExtractor->getAccelerationTrajectory(j);
}

vector<float> ofxKinectFeatures::getAccelerationTrajectoryHistory(int j)
{
	return featExtractor->getAccelerationTrajectoryHistory(j);
}

vector<float> ofxKinectFeatures::getAccelerationTrajectoryHistory(int j, int frames)
{
	return featExtractor->getAccelerationTrajectoryHistory(j, frames);
}

float ofxKinectFeatures::getAccelerationTrajectoryMean(int j, int frames)
{
	return featExtractor->getAccelerationTrajectoryMean(j, frames);
}

ofPoint ofxKinectFeatures::getRelativePositionToTorso(int j)
{
	return toOfPoint(featExtractor->getRelativePositionToTorso(j));
}

vector<ofPoint> ofxKinectFeatures::getRelativePositionToTorsoHistory(int j)
{
	return toOfPointVector(featExtractor->getRelativePositionToTorsoHistory(j));
}

vector<ofPoint> ofxKinectFeatures::getRelativePositionToTorsoHistory(int j, int frames)
{
	return toOfPointVector(featExtractor->getRelativePositionToTorsoHistory(j, frames));
}

float ofxKinectFeatures::getAngle(int j1, int j2, int j3)
{
	return featExtractor->getAngle(j1, j2, j3);
}

ofPoint ofxKinectFeatures::getAccelerationCrest(int j, int frames){
    return toOfPoint(featExtractor->getAccelerationCrest(j, frames));
}

ofPoint ofxKinectFeatures::getRms(int j, int frames){
    return toOfPoint(featExtractor->getRms(j, frames));
}

float ofxKinectFeatures::getQom()
{
	return featExtractor->getQom();
}

vector<float> ofxKinectFeatures::getQomHistory()
{
	return featExtractor->getQomHistory();
}

vector<float> ofxKinectFeatures::getQomHistory(int frames)
{
	return featExtractor->getQomHistory(frames);
}

float ofxKinectFeatures::getCI()
{
	return featExtractor->getCI();
}

vector<float> ofxKinectFeatures::getCIHistory()
{
	return featExtractor->getCIHistory();
}

vector<float> ofxKinectFeatures::getCIHistory(int frames)
{
	return featExtractor->getCIHistory(frames);
}

bool ofxKinectFeatures::isNewDataAvailable()
{
	return featExtractor->isNewDataAvailable();
}

ofPoint ofxKinectFeatures::toOfPoint(MocapPoint point)
{
	return ofPoint(point.x, point.y, point.z);
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

ofEvent<MocapEvent> MocapEvent::events;