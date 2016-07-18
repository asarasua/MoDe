/*
 KinectFeatures
 Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of KinectFeatures, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>
 
 KinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 KinectFeatures is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the KinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
 */

#include "KinectFeatures.h"
#include <algorithm>    // std::find_if

#include <iostream>

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

//____________________________________________________KinectFeatures

KinectFeatures::KinectFeatures() : qom(30), ci(30){
    newValues_ = false;
    setDepth(30);
    aFilter = lpf_soft_a;
    bFilter = lpf_soft_b;
    aLpd1 = lpd1_soft_a;
    bLpd1 = lpd1_soft_b;
    aLpd2 = lpd2_soft_a;
    bLpd2 = lpd2_soft_b;
    //TODO
    
}

KinectFeatures::KinectFeatures(int head, int torso, int depth) : qom(depth), ci(depth){
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

void KinectFeatures::setup(int head, int torso, int depth){
    head_ = head;
    torso_ = torso;
    setDepth(depth);
    qom.setDepth(depth);
    ci.setDepth(depth);
}

void KinectFeatures::setFilterLevel(int filterLevel){
    switch (filterLevel) {
        case filter::HARD:
            aFilter = lpf_hard_a;
            bFilter = lpf_hard_b;
            aLpd1 = lpd1_hard_a;
            bLpd1 = lpd1_hard_b;
            aLpd2 = lpd2_hard_a;
            bLpd2 = lpd2_hard_b;
            break;
        case filter::MED:
            aFilter = lpf_med_a;
            bFilter = lpf_med_b;
            aLpd1 = lpd1_med_a;
            bLpd1 = lpd1_med_b;
            aLpd2 = lpd2_med_a;
            bLpd2 = lpd2_med_b;
            break;
        case filter::SOFT:
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

void KinectFeatures::setDepth(int depth){
    depth_ = depth;
}

int KinectFeatures::getDepth(){
    return depth_;
}

void KinectFeatures::addExtremeListener(ExtremeListener * extremeListener)
{
	extremeListeners.push_back(extremeListener);
}

void KinectFeatures::update(map<int, MocapPoint> joints){
    //Initialize elements
    if (elements_.empty()) {
		for (auto joint : joints) {
			MocapElement newElement(joint.first, depth_);
			elements_.push_back(newElement);
		}
    }
    
    //Compute descriptors
	newValues_ = true;
    
    //TODO solve this!!
    MocapPoint headPos(joints[head_]);
	MocapPoint torsoPos(joints[torso_]);
    
	float h = headPos.distance(torsoPos);
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
		int j = joint.first;
		MocapPoint jointPos(joint.second);
        computeJointDescriptors(j, jointPos, h);
        
        //qom
        meanVel += getJoint(j).velocity.getMagnitude();
        
        //ci
        if (getJoint(j).positionFiltered.getCurrent().x > xMax) {
            xMax = getJoint(j).positionFiltered.getCurrent().x;
        }
        if (getJoint(j).positionFiltered.getCurrent().y > yMax) {
            yMax = getJoint(j).positionFiltered.getCurrent().y;
        }
        if (getJoint(j).positionFiltered.getCurrent().z > zMax) {
            zMax = getJoint(j).positionFiltered.getCurrent().z;
        }
        if (getJoint(j).positionFiltered.getCurrent().x < xMin) {
            xMin = getJoint(j).positionFiltered.getCurrent().x;
        }
        if (getJoint(j).positionFiltered.getCurrent().y < yMin) {
            yMin = getJoint(j).positionFiltered.getCurrent().y;
        }
        if (getJoint(j).positionFiltered.getCurrent().z < zMin) {
            zMin = getJoint(j).positionFiltered.getCurrent().z;
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

    notify(qom.getNewExtremes(), NO_JOINT, FEAT_QOM);
    
    
    ci.push(( -4.0 + (( abs(xMax-xMin) + abs(yMax-yMin) + abs(zMax-zMin) ) / h) ) / 6.0);
    
	notify(ci.getNewExtremes(), NO_JOINT, FEAT_CI);
    
    //TODO solve this!!
//    symmetry_ = 1.0 - (0.5 * (abs(sqrt(getDistanceToTorso(JOINT_RIGHT_HAND))-sqrt(getDistanceToTorso(JOINT_LEFT_HAND))) + abs(sqrt(getDistanceToTorso(JOINT_RIGHT_ELBOW))-sqrt(getDistanceToTorso(JOINT_LEFT_ELBOW)))) / h);
    //symmetry_ = 0.0;
 
    //TODO solve this!!
    //yMaxHands_ = max(getRelativePositionToTorso(JOINT_RIGHT_HAND)[1], getRelativePositionToTorso(JOINT_LEFT_HAND)[1]);
    //yMaxHands_ = 0.0;
        
        
//    } else {
//        newValues_
//        = false;
//    }
}

void KinectFeatures::computeJointDescriptors(int jointId, MocapPoint jointPos, const float &h){
    MocapElement* mocapElement = getElement(jointId);
    
    //Position
    mocapElement->position.push(jointPos);
    notify(mocapElement->position.getNewExtremes(), jointId, FEAT_POSITION);
    
    //Filtered position
    mocapElement->positionFiltered.push(applyFilter(mocapElement->position.getData(), mocapElement->positionFiltered.getData(), aFilter, bFilter));
    notify(mocapElement->positionFiltered.getNewExtremes(), jointId, FEAT_POSITION_FILTERED);
    
    //Velocity
    mocapElement->velocity.push(applyFilter(mocapElement->position.getData(), mocapElement->velocity.getData(), aLpd1, bLpd1));
	notify(mocapElement->velocity.getNewExtremes(), jointId, FEAT_VELOCITY);
    
    //Acceleration
    mocapElement->acceleration.push(applyFilter(mocapElement->position.getData(), mocapElement->acceleration.getData(), aLpd2, bLpd2));
    notify(mocapElement->acceleration.getNewExtremes(), jointId, FEAT_ACCELERATION);
    
    //Acceleration along trajectory
    MocapPoint acc = mocapElement->acceleration.getData().back();
	MocapPoint vel = mocapElement->velocity.getData().back();
    mocapElement->accelerationTrajectory.push(acc.dot(vel) / vel.length());
    notify(mocapElement->accelerationTrajectory.getNewExtremes(), jointId, FEAT_ACCELERATION_TRAJECTORY);
    
    //Relative position to torso
    //TODO solve this!!
    vector<float> relPosToTorso (3);
    relPosToTorso[0] = (jointPos.x - getJoint(torso_).positionFiltered.getCurrent().x) / (h * 1.8);
    relPosToTorso[1] = (jointPos.y - getJoint(torso_).positionFiltered.getCurrent().y) / (h * 1.8);
    relPosToTorso[2] = -((jointPos.z - getJoint(torso_).positionFiltered.getCurrent().z) / h) / 1.4;
    mocapElement->relativePositionToTorso.push(relPosToTorso);
    notify(mocapElement->relativePositionToTorso.getNewExtremes(), jointId, FEAT_RELATIVEPOSTOTORSO);
}

const MocapElement KinectFeatures::getJoint(int jointId){
    vector<MocapElement>::iterator it = find_if(elements_.begin(), elements_.end(), MatchId(jointId));
    if (it != elements_.end()){
        return *it;
    } else {
        return MocapElement(0, 1);
    }
}

MocapElement* KinectFeatures::getElement(int jointId){
    vector<MocapElement>::iterator it = find_if(elements_.begin(), elements_.end(), MatchId(jointId));
    if (it != elements_.end()){
        return &(*it);
    } else {
        return NULL;
    }
}

MocapPoint KinectFeatures::applyFilter(vector<MocapPoint> x, vector<MocapPoint> y, float *a, float *b){
    reverse(x.begin(), x.end());
    reverse(y.begin(), y.end());
	return b[0] * x[0] + b[1] * x[1] + b[2] * x[2] + b[3] * x[3] + b[4] * x[4] - (a[1] * y[0] + a[2] * y[1] + a[3] * y[2] + a[4] * y[3]);
}

void KinectFeatures::notify(vector<MocapExtreme> newExtremes, int jointId, int featId)
{
    for (auto newExtreme : newExtremes) {
        newExtreme.joint = jointId;
        newExtreme.feature = featId;        
        for (auto& extremeListener : extremeListeners){
            extremeListener->newExtreme(newExtreme);
        }
#if OPENFRAMEWORKS
        static MocapEvent newEvent;
        newEvent.joint = newExtreme.joint;
        newEvent.axis = newExtreme.axis;
        newEvent.feature = newExtreme.feature;
        newEvent.value = newExtreme.value;
        newEvent.extremeType = newExtreme.extremeType;
        
        ofNotifyEvent(MocapEvent::events, newEvent);
#endif
    }
    
	
}

template <typename T>
vector<T> KinectFeatures::createVector(T element){
    vector<T> v (depth_);
    fill(v.begin(), v.begin()+depth_, element);
    return v;
}

float KinectFeatures::getAngle(int j1, int j2, int j3){
    float d12 = getJoint(j1).positionFiltered.getCurrent().distance(getJoint(j2).positionFiltered.getCurrent());
    float d13 = getJoint(j1).positionFiltered.getCurrent().distance(getJoint(j3).positionFiltered.getCurrent());
    float d23 = getJoint(j2).positionFiltered.getCurrent().distance(getJoint(j3).positionFiltered.getCurrent());
    return (180/PI) * acos(( -(d13*d13) + d23*d23 + d12*d12 ) / (2*d23*d12 ) ); //cos rule
}

MocapPoint KinectFeatures::getAccelerationCrest(int j){
    if (getElement(j)) {
        return getElement(j)->acceleration.getCrest();
    } else {
        return MocapPoint(0.0);
    }
//    
//    if (getElement(j)) {
//        //cout << ":::" << endl;
//        vector<MocapPoint> acc = getAccelerationHistory(j, frames);
//        vector<double> x_vec, y_vec, z_vec, x_max, y_max, z_max;
//
//        for (auto it : acc) {
//            x_vec.push_back(it.x);
//            y_vec.push_back(it.y);
//            //cout << it.y << ", ";
//            z_vec.push_back(it.z);
//        }
//        
//    //    double sq_sum_x = inner_product(x_vec.begin(), x_vec.end(), x_vec.begin(), 0.0);
//    //    double sq_sum_y = inner_product(y_vec.begin(), y_vec.end(), y_vec.begin(), 0.0);
//    //    double sq_sum_z = inner_product(z_vec.begin(), z_vec.end(), z_vec.begin(), 0.0);
//    //    
//    //    double mean_x = accumulate(x_vec.begin(), x_vec.end(), 0.0) / acc.size();
//    //    double mean_y = accumulate(y_vec.begin(), y_vec.end(), 0.0) / acc.size();
//    //    double mean_z = accumulate(z_vec.begin(), z_vec.end(), 0.0) / acc.size();
//    //    
//    //    double std_x = sqrt(sq_sum_x / acc.size() - mean_x * mean_x);
//    //    double std_y = sqrt(sq_sum_y / acc.size() - mean_y * mean_y);
//    //    double std_z = sqrt(sq_sum_z / acc.size() - mean_z * mean_z);
//        
//        //Find maxima
//        MocapPoint sigma = getElement(j)->acceleration.getStdev();
//        //MocapPoint threshold = getElement(j)->acceleration.getRms() * (1.0 + 3.0 * ( 1.0 / ( sigma + 1.0 )) );
//        //cout << endl << "th: " << threshold.y << " (" << getElement(j)->acceleration.getRms().y << ", " << sigma.y << ", " << (1.0 + 3.0 * ( 1.0 / ( sigma.y + 1.0 )) ) << ")" << endl;
//        //cout << endl << "peaks: " << endl;
//        for (int i = 0; i < acc.size()-4; i++) {
//            if (distance(x_vec.begin()+i, max_element(x_vec.begin()+i, x_vec.begin()+i+4)) == 2){
//                x_max.push_back(x_vec[i + 2]);
//            }
//            if (distance(y_vec.begin()+i, max_element(y_vec.begin()+i, y_vec.begin()+i+4)) == 2){
//                //&& y_vec[i + 2] > threshold.y){
//                y_max.push_back(y_vec[i + 2]);
//                //cout << y_vec[i + 2] << endl;
//            }
//            if (distance(z_vec.begin()+i, max_element(z_vec.begin()+i, z_vec.begin()+i+4)) == 2){
//                z_max.push_back(z_vec[i + 2]);
//            }
//        }
//        
//        MocapPoint crest(0.0, 0.0, 0.0);
//        if (x_max.size()) crest.x = ( accumulate( x_max.begin(), x_max.end(), 0.0)/x_max.size() ) / getElement(j)->acceleration.getStdev().x; //denom is RMS
//        if (y_max.size()) crest.y = ( accumulate( y_max.begin(), y_max.end(), 0.0)/y_max.size() ) / getElement(j)->acceleration.getStdev().y;
//        if (z_max.size()) crest.z = ( accumulate( z_max.begin(), z_max.end(), 0.0)/z_max.size() ) / getElement(j)->acceleration.getStdev().z;
//        
//        //cout << "======" << endl;
//        
//        return crest;
//    } else {
//        return MocapPoint(0.0,0.0,0.0);
//    }    
}

MocapPoint KinectFeatures::getRms(int j, int frames){
    if (getElement(j)) {
        return getElement(j)->acceleration.getRms();
    } else {
        return MocapPoint(0.0,0.0,0.0);
    }
}

float KinectFeatures::getQom(){
    if (qom.getData().size()) {
        return qom.getData().back();
    } else {
        return 0.0;
    }
}

vector<float> KinectFeatures::getQomHistory(){
    return qom.getData();
}

vector<float> KinectFeatures::getQomHistory(int frames){
    vector<float> qomH = qom.getData();
    vector<float> history(qomH.end()-frames, qomH.end());
    return history;
}

float KinectFeatures::getCI(){
    if (ci.getData().size()) {
        return ci.getData().back();
    } else {
        return 0.0;
    }
}

vector<float> KinectFeatures::getCIHistory(){
    return ci.getData();
}

vector<float> KinectFeatures::getCIHistory(int frames){
    vector<float> ciH = ci.getData();
    vector<float> history(ciH.end()-frames, ciH.end());
    return history;
}

/*float KinectFeatures::getSymmetry(){
    return symmetry_;
}

float KinectFeatures::getYMaxHands(){
    return yMaxHands_;
}*/

bool KinectFeatures::isNewDataAvailable(){
    return newValues_;
}

#if OPENFRAMEWORKS
ofEvent<MocapEvent> MocapEvent::events;
#endif