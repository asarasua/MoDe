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

KinectFeatures::KinectFeatures(){
    newValues_ = false;
    setDepth(30);
    aFilter = lpf_soft_a;
    bFilter = lpf_soft_b;
    aLpd1 = lpd1_soft_a;
    bLpd1 = lpd1_soft_b;
    aLpd2 = lpd2_soft_a;
    bLpd2 = lpd2_soft_b;
    //TODO
    
    for (int i = 0; i < depth_; i++) {
        qom_.push_back(0.0);
        ci_.push_back(0.0);
    }
    
}

KinectFeatures::KinectFeatures(int head, int torso){
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
    
    for (int i = 0; i < depth_; i++) {
        qom_.push_back(0.0);
        ci_.push_back(0.0);
    }
}

void KinectFeatures::setup(int head, int torso){
    head_ = head;
    torso_ = torso;
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
        meanVel += getVelocityMagnitude(j);
        
        //ci
        if (getPositionFiltered(j).x > xMax) {
            xMax = getPositionFiltered(j).x;
        }
        if (getPositionFiltered(j).y > yMax) {
            yMax = getPositionFiltered(j).y;
        }
        if (getPositionFiltered(j).z > zMax) {
            zMax = getPositionFiltered(j).z;
        }
        if (getPositionFiltered(j).x < xMin) {
            xMin = getPositionFiltered(j).x;
        }
        if (getPositionFiltered(j).y < yMin) {
            yMin = getPositionFiltered(j).y;
        }
        if (getPositionFiltered(j).z < zMin) {
            zMin = getPositionFiltered(j).z;
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
    
    //qom_ = accumulate(meanVels_.begin(), meanVels_.end(), 0.0) / (meanVels_.size());
    // QOM insert
    if (qom_.size() <= depth_) {
        qom_.insert(qom_.begin(), accumulate(meanVels_.begin(), meanVels_.end(), 0.0) / (meanVels_.size()));
    }
    // QOM delete
    if (qom_.size() > depth_) {
        qom_.pop_back();
    }
    
    checkMaxAndMin(getQomHistory(5), NO_JOINT, FEAT_QOM);
    
    //ci_ = ( -4.0 + (( abs(xMax-xMin) + abs(yMax-yMin) + abs(zMax-zMin) ) / h) ) / 6.0;
    if (ci_.size() <= depth_) {
        ci_.insert(ci_.begin(), ( -4.0 + (( abs(xMax-xMin) + abs(yMax-yMin) + abs(zMax-zMin) ) / h) ) / 6.0);
    }
    // QOM delete
    if (ci_.size() > depth_) {
        ci_.pop_back();
    }
    
    checkMaxAndMin(getCIHistory(5), NO_JOINT, FEAT_CI);
    
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
    mocapElement->setPosition(jointPos);
    
    //Filtered position
    mocapElement->setPositionFiltered(applyFilter(mocapElement->getPosition(), mocapElement->getPositionFiltered(), aFilter, bFilter));
    
    //Velocity
    mocapElement->setVelocity(applyFilter(mocapElement->getPosition(), mocapElement->getVelocity(), aLpd1, bLpd1));
    checkMaxAndMin(getVelocityHistory(jointId, 5), jointId, FEAT_VELOCITY);
    
    //Acceleration
    mocapElement->setAcceleration(applyFilter(mocapElement->getPosition(), mocapElement->getAcceleration(), aLpd2, bLpd2));
    checkMaxAndMin(getAccelerationHistory(jointId, 5), jointId, FEAT_ACCELERATION);
    
    //Acceleration along trajectory
    MocapPoint acc = mocapElement->getAcceleration()[0];
	MocapPoint vel = mocapElement->getVelocity()[0];
    mocapElement->setAccelerationTrajectory(acc.dot(vel) / vel.length());

    checkMaxAndMin(getAccelerationTrajectoryHistory(jointId, 5), jointId, FEAT_ACCELERATION_TRAJECTORY);
    
    //Distance to torso
	MocapPoint torsoPos(getPositionFiltered(torso_));
	MocapPoint filtPos(getPositionFiltered(jointId));
    mocapElement->setDistanceToTorso(torsoPos.squareDistance(filtPos));
    checkMaxAndMin(getDistanceToTorsoHistory(jointId, 5), jointId, FEAT_DISTANCETOTORSO);
    
    //Relative position to torso
    //TODO solve this!!
    vector<float> relPosToTorso (3);
    relPosToTorso[0] = (jointPos.x - getPositionFiltered(torso_).x) / (h * 1.8);
    relPosToTorso[1] = (jointPos.y - getPositionFiltered(torso_).y) / (h * 1.8);
    relPosToTorso[2] = -((jointPos.z - getPositionFiltered(torso_).z) / h) / 1.4;
    mocapElement->setRelativePositionToTorso(relPosToTorso);
    checkMaxAndMin(getRelativePositionToTorsoHistory(jointId, 5), jointId, FEAT_RELATIVEPOSTOTORSO);
    
    //TODO: auto hand
    //beatTracker.update(getAccTrVector(JOINT_RIGHT_HAND), get3DFiltPosVector(JOINT_RIGHT_HAND)[coord::Y]);
}

MocapElement* KinectFeatures::getElement(int _id){
    vector<MocapElement>::iterator it = find_if(elements_.begin(), elements_.end(), MatchId(_id));
    if (it != elements_.end()){
        return &(*it);
    } else {
        return NULL;
    }
}

MocapPoint KinectFeatures::applyFilter(vector<MocapPoint> x, vector<MocapPoint> y, float *a, float *b){
	return b[0] * x[0] + b[1] * x[1] + b[2] * x[2] + b[3] * x[3] + b[4] * x[4] - (a[1] * y[0] + a[2] * y[1] + a[3] * y[2] + a[4] * y[3]);
}


void KinectFeatures::checkMaxAndMin(vector<MocapPoint> descriptorHistory, unsigned int jointId, unsigned int feature){
    vector<float> x_vec, y_vec, z_vec;
    //for (vector<MocapPoint>::iterator it = descriptorHistory.begin(); it != descriptorHistory.end(); it++) {
	for (auto it : descriptorHistory){
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
    } else if (distance(x_vec.begin(), min_element(x_vec.begin(), x_vec.end())) == 2) {
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
    } else if (distance(y_vec.begin(), min_element(y_vec.begin(), y_vec.end())) == 2) {
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
    } else if (distance(z_vec.begin(), min_element(z_vec.begin(), z_vec.end())) == 2) {
        static MocapMinEvent newMinEvent;
        newMinEvent.joint = jointId;
        newMinEvent.axis = MOCAP_Z;
        newMinEvent.feature = feature;
        newMinEvent.value = descriptorHistory[1].z;
        ofNotifyEvent(MocapMinEvent::events, newMinEvent);
    }
}

void KinectFeatures::checkMaxAndMin(vector<float> descriptorHistory, unsigned int jointId, unsigned int feature){
    if (distance(descriptorHistory.begin(), max_element(descriptorHistory.begin(), descriptorHistory.end())) == 2) {
        static MocapMaxEvent newMaxEvent;
        newMaxEvent.joint = jointId;
        newMaxEvent.feature = feature;
        newMaxEvent.value = descriptorHistory[1];
        ofNotifyEvent(MocapMaxEvent::events, newMaxEvent);
    } else if (distance(descriptorHistory.begin(), min_element(descriptorHistory.begin(), descriptorHistory.end())) == 2) {
        static MocapMinEvent newMinEvent;
        newMinEvent.joint = jointId;
        newMinEvent.feature = feature;
        newMinEvent.value = descriptorHistory[1];
        ofNotifyEvent(MocapMinEvent::events, newMinEvent);
    }
}

template <typename T>
vector<T> KinectFeatures::createVector(T element){
    vector<T> v (depth_);
    fill(v.begin(), v.begin()+depth_, element);
    return v;
}

//Descriptors getters

MocapPoint KinectFeatures::getPosition(int j){
    if (getElement(j)) {
        return getElement(j)->getPosition()[0];
    } else {
        return MocapPoint(0,0,0);
    }
}

vector<MocapPoint> KinectFeatures::getPositionHistory(int j){
    if (getElement(j)) {
        return getElement(j)->getPosition();
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

vector<MocapPoint> KinectFeatures::getPositionHistory(int j, int frames){
    if (getElement(j)) {
        vector<MocapPoint> fullHistory = getElement(j)->getPosition();
        vector<MocapPoint> history(fullHistory.begin(), fullHistory.begin() + frames);
        return history;
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

MocapPoint KinectFeatures::getPositionFiltered(int j){
    if (getElement(j)) {
        return getElement(j)->getPositionFiltered()[0];
    } else {
        return MocapPoint(0,0,0);
    }
}

vector<MocapPoint> KinectFeatures::getPositionFilteredHistory(int j){
    if (getElement(j)) {
        return getElement(j)->getPositionFiltered();
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

vector<MocapPoint> KinectFeatures::getPositionFilteredHistory(int j, int frames){
    if (getElement(j)) {
        vector<MocapPoint> fullHistory = getElement(j)->getPositionFiltered();
        vector<MocapPoint> history(fullHistory.begin(), fullHistory.begin() + frames);
        return history;
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

MocapPoint KinectFeatures::getVelocity(int j){
    if (getElement(j)) {
        return getElement(j)->getVelocity()[0];
    } else {
        return MocapPoint(0.0,0.0,0.0);
    }
}

vector<MocapPoint> KinectFeatures::getVelocityHistory(int j){
    if (getElement(j)) {
        return getElement(j)->getVelocity();
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

vector<MocapPoint> KinectFeatures::getVelocityHistory(int j, int frames){
    if (getElement(j)) {
        vector<MocapPoint> fullHistory = getElement(j)->getVelocity();
        vector<MocapPoint> history(fullHistory.begin(), fullHistory.begin() + frames);
        return history;
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

float KinectFeatures::getVelocityMagnitude(int j){
    if (getElement(j)) {
        return getElement(j)->getVelocity()[0].length();
    } else {
        return 0.0;
    }
}

MocapPoint KinectFeatures::getVelocityMean(int j, int frames){
    if (getElement(j)) {
        float sumx = 0, sumy = 0, sumz = 0.0;
        for (int i = 0; i < frames && i < getElement(j)->getVelocity().size(); i++) {
            sumx += getElement(j)->getVelocity()[i].x;
            sumy += getElement(j)->getVelocity()[i].y;
            sumz += getElement(j)->getVelocity()[i].z;
        }
        if (frames <= getElement(j)->getVelocity().size()) {
            return MocapPoint(sumx / frames, sumy / frames, sumz / frames);
        } else {
            return MocapPoint(sumx / getElement(j)->getVelocity().size(), sumy / getElement(j)->getVelocity().size(), sumz / getElement(j)->getVelocity().size());
        }
    } else {
        return MocapPoint(0.0,0.0,0.0);
    }
}

float KinectFeatures::getVelocityMagnitudeMean(int j, int frames){
    if (getElement(j)) {
        float sum = 0.0;
        for (int i = 0; i < frames && i < getElement(j)->getVelocity().size(); i++) {
            sum += getElement(j)->getVelocity()[i].length();
        }
        if (frames <= getElement(j)->getVelocity().size()) {
            return sum / frames;
        } else {
            return sum / getElement(j)->getVelocity().size();
        }
    } else {
        return 0.0;
    }
}

MocapPoint KinectFeatures::getAcceleration(int j){
    if (getElement(j)) {
        return getElement(j)->getAcceleration()[0];
    } else {
        return MocapPoint(0.0,0.0,0.0);
    }
}

vector<MocapPoint> KinectFeatures::getAccelerationHistory(int j){
    if (getElement(j)) {
        return getElement(j)->getAcceleration();
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

vector<MocapPoint> KinectFeatures::getAccelerationHistory(int j, int frames){
    if (getElement(j)) {
        vector<MocapPoint> fullHistory = getElement(j)->getAcceleration();
        vector<MocapPoint> history(fullHistory.begin(), fullHistory.begin() + frames);
        return history;
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

float KinectFeatures::getAccelerationMagnitude(int j){
    if (getElement(j)) {
        return getElement(j)->getAcceleration()[0].length();
    } else {
        return 0.0;
    }
}

MocapPoint KinectFeatures::getAccelerationMean(int j, int frames){
    if (getElement(j)) {
        float sumx = 0, sumy = 0, sumz = 0.0;
        for (int i = 0; i < frames && i < getElement(j)->getVelocity().size(); i++) {
            sumx += getElement(j)->getAcceleration()[i].x;
            sumy += getElement(j)->getAcceleration()[i].y;
            sumz += getElement(j)->getAcceleration()[i].z;
        }
        if (frames <= getElement(j)->getVelocity().size()) {
            return MocapPoint(sumx / frames, sumy / frames, sumz / frames);
        } else {
            return MocapPoint(sumx / getElement(j)->getAcceleration().size(), sumy / getElement(j)->getAcceleration().size(), sumz / getElement(j)->getAcceleration().size());
        }
    } else {
        return MocapPoint(0.0,0.0,0.0);
    }
}

float KinectFeatures::getAccelerationMagnitudeMean(int j, int frames){
    if (getElement(j)) {
        float sum = 0.0;
        for (int i = 0; i < frames && i < getElement(j)->getAcceleration().size(); i++) {
            sum += getElement(j)->getAcceleration()[i].length();
        }
        if (frames <= getElement(j)->getAcceleration().size()) {
            return sum / frames;
        } else {
            return sum / getElement(j)->getAcceleration().size();
        }
    } else {
        return 0.0;
    }
}

float KinectFeatures::getAccelerationTrajectory(int j){
    if (getElement(j)){
        return getElement(j)->getAccelerationTrajectory()[0];
    } else {
        return 0.0;
    }
}

vector<float> KinectFeatures::getAccelerationTrajectoryHistory(int j){
    if (getElement(j)) {
        return getElement(j)->getAccelerationTrajectory();
    } else {
        return createVector(0.0f);
    }
}

vector<float> KinectFeatures::getAccelerationTrajectoryHistory(int j, int frames){
    if (getElement(j)) {
        vector<float> fullHistory = getElement(j)->getAccelerationTrajectory();
        vector<float> history(fullHistory.begin(), fullHistory.begin() + frames);
        return history;
    } else {
        return createVector(0.0f);
    }
}

float KinectFeatures::getAccelerationTrajectoryMean(int j, int frames){
    if (getElement(j)) {
        float sum = 0.0;
        for (int i = 0; i < frames && i < getElement(j)->getAccelerationTrajectory().size(); i++) {
            sum += getElement(j)->getAccelerationTrajectory()[i];
        }
        if (frames <= getElement(j)->getAccelerationTrajectory().size()) {
            return sum / frames;
        } else {
            return sum / getElement(j)->getAccelerationTrajectory().size();
        }
    } else {
        return 0.0;
    }
}

float KinectFeatures::getDistanceToTorso(int j){
    if (getElement(j)){
        return getElement(j)->getDistanceToTorso()[0];
    } else {
        return 0.0;
    }
}

vector<float> KinectFeatures::getDistanceToTorsoHistory(int j){
    if (getElement(j)) {
        return getElement(j)->getDistanceToTorso();
    } else {
        return createVector(0.0f);
    }
}

vector<float> KinectFeatures::getDistanceToTorsoHistory(int j, int frames){
    if (getElement(j)) {
        vector<float> fullHistory = getElement(j)->getDistanceToTorso();
        vector<float> history(fullHistory.begin(), fullHistory.begin() + frames);
        return history;
    } else {
        return createVector(0.0f);
    }
}

MocapPoint KinectFeatures::getRelativePositionToTorso(int j){
    if (getElement(j)) {
        return getElement(j)->getRelativePositionToTorso()[0];
    } else {
        return MocapPoint(0.0,0.0,0.0);
    }
}

vector<MocapPoint> KinectFeatures::getRelativePositionToTorsoHistory(int j){
    if (getElement(j)) {
        return getElement(j)->getRelativePositionToTorso();
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

vector<MocapPoint> KinectFeatures::getRelativePositionToTorsoHistory(int j, int frames){
    if (getElement(j)) {
        vector<MocapPoint> fullHistory = getElement(j)->getRelativePositionToTorso();
        vector<MocapPoint> history(fullHistory.begin(), fullHistory.begin() + frames);
        return history;
    } else {
        return createVector(MocapPoint(0.0,0.0,0.0));
    }
}

float KinectFeatures::getAngle(int j1, int j2, int j3){
    float d12 = getPositionFiltered(j1).distance(getPositionFiltered(j2));
    float d13 = getPositionFiltered(j1).distance(getPositionFiltered(j3));
    float d23 = getPositionFiltered(j2).distance(getPositionFiltered(j3));
    return RAD_TO_DEG * acos(( -(d13*d13) + d23*d23 + d12*d12 ) / (2*d23*d12 ) ); //cos rule
}

float KinectFeatures::getQom(){
    return qom_[0];
}

vector<float> KinectFeatures::getQomHistory(){
    return qom_;
}

vector<float> KinectFeatures::getQomHistory(int frames){
    vector<float> history(qom_.begin(), qom_.begin() + frames);
    return history;
}

float KinectFeatures::getCI(){
    return ci_[0];
}

vector<float> KinectFeatures::getCIHistory(){
    return ci_;
}

vector<float> KinectFeatures::getCIHistory(int frames){
    vector<float> history(ci_.begin(), ci_.begin() + frames);
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