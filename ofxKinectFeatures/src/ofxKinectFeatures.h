/*
KinectFeatures
Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya

This file is part of KinectFeatures, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>

KinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

KinectFeatures is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).

You should have received a copy of the GNU Lesser General Public License long within the KinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.

If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
*/

#ifndef ofxKinectFeatures_h
#define ofxKinectFeatures_h

#include "kinectFeatures.h"
#include "ofMain.h"

class MocapEvent : public ofEventArgs, public MocapExtreme {

public:
	MocapEvent() {}

	static ofEvent <MocapEvent> events;
};


class ofxKinectFeatures : public ExtremeListener {
public:
	ofxKinectFeatures() : ExtremeListener(new KinectFeatures()) {};
	ofxKinectFeatures(int head, int torso);

	void setup(int head, int torso);
	void update(map<int, ofPoint> joints);

	void newExtreme(MocapExtreme extreme); //method from ExtremeListener, called whenever a new extreme is found

	void setFilterLevel(int filterLevel);

	void setDepth(int depth);
	int getDepth();

	//DESCIPTOR GETTERS
	//JOINT DESCRIPTORS
	ofPoint getPosition(int j);
	vector<ofPoint> getPositionHistory(int j);
	vector<ofPoint> getPositionHistory(int j, int frames);

	ofPoint getPositionFiltered(int j);
	vector<ofPoint> getPositionFilteredHistory(int j);
	vector<ofPoint> getPositionFilteredHistory(int j, int frames);

	ofPoint getVelocity(int j);
	vector<ofPoint> getVelocityHistory(int j);
	vector<ofPoint> getVelocityHistory(int j, int frames);
	float getVelocityMagnitude(int j);
	ofPoint getVelocityMean(int j, int frames = 30);
	float getVelocityMagnitudeMean(int j, int frames = 30);

	ofPoint getAcceleration(int j);
	vector<ofPoint> getAccelerationHistory(int j);
	vector<ofPoint> getAccelerationHistory(int j, int frames);
	float getAccelerationMagnitude(int j);
	ofPoint getAccelerationMean(int j, int frames = 30);
	float getAccelerationMagnitudeMean(int j, int frames = 30);

	float getAccelerationTrajectory(int j);
	vector<float> getAccelerationTrajectoryHistory(int j);
	vector<float> getAccelerationTrajectoryHistory(int j, int frames);
	float getAccelerationTrajectoryMean(int j, int frames = 30);

	ofPoint getRelativePositionToTorso(int j);
	vector<ofPoint> getRelativePositionToTorsoHistory(int j);
	vector<ofPoint> getRelativePositionToTorsoHistory(int j, int frames);

	//SPECIAL DESCRIPTORS
	float getAngle(int j1, int j2, int j3);
    ofPoint getAccelerationCrest(int j, int frames);
    ofPoint getRms(int j, int frames);

	//OVERALL DESCRIPTORS
	float getQom();
	vector<float> getQomHistory();
	vector<float> getQomHistory(int frames);
	float getCI();
	vector<float> getCIHistory();
	vector<float> getCIHistory(int frames);
	//    float getSymmetry();
	//    float getYMaxHands();

	bool isNewDataAvailable();

private:
	ofPoint toOfPoint(MocapPoint point);
	vector<ofPoint> toOfPointVector(vector <MocapPoint> pointVector);
	MocapPoint toMocapPoint(ofPoint point);
};

#endif
