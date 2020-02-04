/*
 MoDeExtractor
 Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of MoDeExtractor, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>
 
 MoDeExtractor is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 MoDeExtractor is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the MoDeExtractor SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
 */

#include "mode_extractor.h"
#include <algorithm>    // std::find_if


//Real-time filters for MoCap by Skogstad et al ( http://www.uio.no/english/research/groups/fourms/projects/sma/subprojects/mocapfilters/ )

float lpf_soft_a[] = {1,-1.2982434912,1.4634092217,-0.7106501488,0.2028836637};
float lpf_soft_b[] = {0.1851439645,0.1383283833,0.1746892243,0.1046627716,0.0464383730};
float lpf_med_a[] = {1,-1.7462227354,1.7354077932,-0.8232679111,0.1793463694};
float lpf_med_b[] = {0.1526249789,0.0333481282,0.0777551903,0.0667145281,0.0138945068};
float lpf_hard_a[] = {1,-1.9185418203,1.5929378702,-0.5939699187,0.0814687111};
float lpf_hard_b[] = {0.1400982208,-0.0343775491,0.0454003083,0.0099732061,0.0008485135};

float lpd1_soft_a[] = {1,-0.2919477037,0.5104653639,-0.01557831719,0.000283848732};
float lpd1_soft_b[] = {0.2712475020,0.1323672597,-0.0487267360,-0.1783422292,-0.1765457966};
float lpd1_med_a[] = {1,-0.9870779094,0.7774863652,-0.2206843188,0.02813441289};
float lpd1_med_b[] = {0.1973679432,-0.0056567353,-0.0321850947,-0.1099445540,-0.0495815592};
float lpd1_hard_a[] = {1,-2.057494776,1.858705877,-0.801785135,0.131076358};
float lpd1_hard_b[] = {-0.1543174259,0.1742393427,-0.0178886989,-0.0022975713,0.0002643535};

float lpd2_soft_a[] = {1,-0.8274946715,0.8110775672,-0.3530877871,0.06598917583};
float lpd2_soft_b[] = {0.1099156485,-0.1289124440,-0.0372667405,0.0216082189,0.0346553170};
float lpd2_med_a[] = {1,-1.571029458,1.459212744,-0.7173743414,0.1488005975};
float lpd2_med_b[] = {-0.0795571277,0.1390709784,-0.0479192600,-0.0031459045,-0.0084486862};
float lpd2_hard_a[] = {1,-1.628286742,1.418759018,-0.6223424612,0.1085280231};
float lpd2_hard_b[] = {-0.0738989849,0.1351624829,-0.0512998379,-0.0072918334,-0.0026718267};

using namespace MoDe;

//____________________________________________________MoDeExtractor

MoDeExtractor::MoDeExtractor() : qom(30), ci(30){
    newValues_ = false;
    setDepth(30);
    aFilter = lpf_soft_a;
    bFilter = lpf_soft_b;
    aLpd1 = lpd1_soft_a;
    bLpd1 = lpd1_soft_b;
    aLpd2 = lpd2_soft_a;
    bLpd2 = lpd2_soft_b;
	head_ = NO_JOINT;
	torso_ = NO_JOINT;
}

MoDeExtractor::MoDeExtractor(int head, int torso, int depth) : qom(depth), ci(depth){
    newValues_ = false;
    setDepth(30);
    aFilter = lpf_soft_a;
    bFilter = lpf_soft_b;
    aLpd1 = lpd1_soft_a;
    bLpd1 = lpd1_soft_b;
    aLpd2 = lpd2_soft_a;
    bLpd2 = lpd2_soft_b;
    head_ = head;
    torso_ = torso;
}

void MoDeExtractor::setup(int head, int torso, int depth){
    head_ = head;
    torso_ = torso;
    setDepth(depth);
    qom.setDepth(depth);
    ci.setDepth(depth);
#ifdef OPENFRAMEWORKS
	myfile.open(ofToDataPath("foo.txt"));
#endif
}

void MoDeExtractor::setFilterLevel(int filterLevel){
    switch (filterLevel) {
        case FILTER_HARD:
            aFilter = lpf_hard_a;
            bFilter = lpf_hard_b;
            aLpd1 = lpd1_hard_a;
            bLpd1 = lpd1_hard_b;
            aLpd2 = lpd2_hard_a;
            bLpd2 = lpd2_hard_b;
            break;
        case FILTER_MED:
            aFilter = lpf_med_a;
            bFilter = lpf_med_b;
            aLpd1 = lpd1_med_a;
            bLpd1 = lpd1_med_b;
            aLpd2 = lpd2_med_a;
            bLpd2 = lpd2_med_b;
            break;
        case FILTER_SOFT:
            aFilter = lpf_soft_a;
            bFilter = lpf_soft_b;
            aLpd1 = lpd1_soft_a;
            bLpd1 = lpd1_soft_b;
            aLpd2 = lpd2_soft_a;
            bLpd2 = lpd2_soft_b;
        default:
            break;
    }
}

void MoDeExtractor::setDepth(int depth){
    depth_ = depth;
}

int MoDeExtractor::getDepth(){
    return depth_;
}

void MoDeExtractor::addExtremeListener(ExtremeListener * extremeListener)
{
	extremeListeners.push_back(extremeListener);
}

void MoDeExtractor::update(map<int, MoDePoint> joints){

	for (auto joint : joints) {
		vector<MoDeJoint>::iterator it = find_if(elements_.begin(), elements_.end(), MatchId(joint.first));
		if (it == elements_.end()){
			MoDeJoint newElement(joint.first, depth_);
			elements_.push_back(newElement);
		}
	}
    
    //Compute descriptors
	newValues_ = true;
    
    float meanVel = 0.0; //for qom
    //for CI
    float xMax = numeric_limits<float>::min();
	float yMax = numeric_limits<float>::min();
	float zMax = numeric_limits<float>::min();
    float xMin = numeric_limits<float>::max();
	float yMin = numeric_limits<float>::max();
	float zMin = numeric_limits<float>::max();

	for (auto joint : joints)
	{
		MoDeJoint* modeJoint = getElement(joint.first);
		modeJoint->computeDescriptors(joint.second, aFilter, bFilter, aLpd1, bLpd1, aLpd2, bLpd2);
		for (auto extrema : modeJoint->getNewExtrema())
		{
			notify(extrema.second, joint.first, extrema.first);
		}
        
        //qom
		meanVel += modeJoint->getDescriptor(MoDe::DESC_VELOCITY).getMagnitude();
        
        //ci
        if (modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().x > xMax) {
            xMax = modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().x;
        }
        if (modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().y > yMax) {
            yMax = modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().y;
        }
        if (modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().z > zMax) {
            zMax = modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().z;
        }
        if (modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().x < xMin) {
            xMin = modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().x;
        }
        if (modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().y < yMin) {
            yMin = modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().y;
        }
        if (modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().z < zMin) {
            zMin = modeJoint->getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().z;
        }
    }
    
    // Add position to history
    if (meanVels_.size() <= depth_) {
        meanVels_.insert(meanVels_.begin(), meanVel/joints.size());
    }
    
    // remove positions from history
    if (meanVels_.size() > depth_) {
        meanVels_.pop_back();
    }
    
    qom.push(accumulate(meanVels_.begin(), meanVels_.end(), 0.0) / (meanVels_.size()));

    notify(qom.getNewExtremes(), NO_JOINT, DESC_QOM);
    
	if (head_ != NO_JOINT && torso_ != NO_JOINT)
	{
		MoDePoint headPos(joints[head_]);
		MoDePoint torsoPos(joints[torso_]);

		float h = headPos.distance(torsoPos);
		ci.push((-4.0 + ((abs(xMax - xMin) + abs(yMax - yMin) + abs(zMax - zMin)) / h)) / 6.0);

		notify(ci.getNewExtremes(), NO_JOINT, DESC_CI);

		for (auto joint : joints) {
			MoDeJoint modeJoint = getJoint(joint.first);
			modeJoint.computeRelativeJointDescriptors(joint.second, h);
		}
	}
    
    //TODO solve this!!
//    symmetry_ = 1.0 - (0.5 * (abs(sqrt(getDistanceToTorso(JOINT_RIGHT_HAND))-sqrt(getDistanceToTorso(JOINT_LEFT_HAND))) + abs(sqrt(getDistanceToTorso(JOINT_RIGHT_ELBOW))-sqrt(getDistanceToTorso(JOINT_LEFT_ELBOW)))) / h);
    //symmetry_ = 0.0;
}

MoDeJoint MoDeExtractor::getJoint(int jointId){
    vector<MoDeJoint>::iterator it = find_if(elements_.begin(), elements_.end(), MatchId(jointId));
    if (it != elements_.end()){
        return *it;
    } else {
        return MoDeJoint(0, 1);
    }
}

MoDeJoint* MoDeExtractor::getElement(int jointId){
    vector<MoDeJoint>::iterator it = find_if(elements_.begin(), elements_.end(), MatchId(jointId));
    if (it != elements_.end()){
        return &(*it);
    } else {
        return NULL;
    }
}

void MoDeExtractor::notify(vector<MoDeExtreme> newExtremes, int jointId, int featId)
{
    for (auto newExtreme : newExtremes) {
        newExtreme.joint = jointId;
        newExtreme.feature = featId;        
        for (auto& extremeListener : extremeListeners){
            extremeListener->newExtreme(newExtreme);
        }
#if OPENFRAMEWORKS
        static ofxMoDeEvent newEvent;
        newEvent.joint = newExtreme.joint;
        newEvent.axis = newExtreme.axis;
        newEvent.feature = newExtreme.feature;
        newEvent.value = newExtreme.value;
        newEvent.extremeType = newExtreme.extremeType;
        
        ofNotifyEvent(ofxMoDeEvent::events, newEvent);
#endif
    }
}

template <typename T>
vector<T> MoDeExtractor::createVector(T element){
    vector<T> v (depth_);
    fill(v.begin(), v.begin()+depth_, element);
    return v;
}

float MoDeExtractor::getAngle(int j1, int j2, int j3){
    float d12 = getJoint(j1).getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().distance(getJoint(j2).getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent());
    float d13 = getJoint(j1).getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().distance(getJoint(j3).getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent());
    float d23 = getJoint(j2).getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent().distance(getJoint(j3).getDescriptor(MoDe::DESC_POSITION_FILTERED).getCurrent());
    return (180/PI) * acos(( -(d13*d13) + d23*d23 + d12*d12 ) / (2*d23*d12 ) ); //cos rule
}

float MoDeExtractor::getQom(){
    if (qom.getData().size()) {
        return qom.getData().back();
    } else {
        return 0.0;
    }
}

vector<float> MoDeExtractor::getQomHistory(){
    return qom.getData();
}

vector<float> MoDeExtractor::getQomHistory(int frames){
    vector<float> qomH = qom.getData();
    vector<float> history(qomH.end()-frames, qomH.end());
    return history;
}

float MoDeExtractor::getCI(){
    if (ci.getData().size()) {
        return ci.getData().back();
    } else {
        return 0.0;
    }
}

vector<float> MoDeExtractor::getCIHistory(){
    return ci.getData();
}

vector<float> MoDeExtractor::getCIHistory(int frames){
    vector<float> ciH = ci.getData();
    vector<float> history(ciH.end()-frames, ciH.end());
    return history;
}

/*float MoDeExtractor::getSymmetry(){
    return symmetry_;
}

float MoDeExtractor::getYMaxHands(){
    return yMaxHands_;
}*/

bool MoDeExtractor::isNewDataAvailable(){
    return newValues_;
}

#if OPENFRAMEWORKS
ofEvent<ofxMoDeEvent> ofxMoDeEvent::events;
#endif