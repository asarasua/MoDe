/*
 
 ofxKinectFeatures
 Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 Copyright (c) 2014 Álvaro Sarasúa <http://alvarosarasua.wordpress.com>
 
 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following
 conditions:
 
 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 
 */

#include "ofxKinectFeatures.h"

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

//____________________________________________________ofxKinectFeatures

ofxKinectFeatures::ofxKinectFeatures(){
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

void ofxKinectFeatures::setKinect(ofxOpenNI* kinect){
    kinect_ = kinect;
}

void ofxKinectFeatures::setFilterLevel(int filterLevel){
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

void ofxKinectFeatures::setDepth(int depth){
    depth_ = depth;
}

int ofxKinectFeatures::getDepth(){
    return depth_;
}

void ofxKinectFeatures::update(){
    if (kinect_->getNumTrackedUsers()) {
        ofxOpenNIUser user = kinect_->getTrackedUser(0);
        
        //Initialize elements
        if (elements_.empty()) {
            for (int i = 0; i < user.getNumJoints(); i++) {
                ofxOpenNIJoint joint = user.getJoint((Joint)i);
                ofxMocapElement newElement(depth_);
                newElement.setElementId(joint.getType());
                elements_.push_back(newElement);
            }
        }
        
        //Compute descriptors
        //Hard-coded way to check if skeleton (bSkeleton doesn't work) and new data (isNewDataAvailable doesn't work)
        if (user.getJoint((Joint)0).getWorldPosition() != ofPoint(0,0,0) &&
            user.getJoint((Joint)0).getWorldPosition() != getElement((Joint)0)->getPosition()[0] ) {
            newValues_ = true;
            
            ofPoint headPos = user.getJoint(JOINT_HEAD).getWorldPosition();
            ofPoint torsoPos = user.getJoint(JOINT_TORSO).getWorldPosition();
            float h = headPos.distance(torsoPos);
            float meanVel = 0.0; //for qom
            //for CI
            float xMax, yMax, zMax = numeric_limits<float>::min();
            float xMin, yMin, zMin = numeric_limits<float>::max();
            
            for (int i = 0; i < user.getNumJoints(); i++) {
                ofxOpenNIJoint joint = user.getJoint((Joint)i);
                Joint j = (Joint) i;
                computeJointDescriptors(joint, h);
                
                //qom
                meanVel += getVelocityMagnitude(j);
                //meanVel += get3DVel((Joint)i).length();
                
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
                meanVels_.insert(meanVels_.begin(), meanVel/user.getNumJoints());
            }
            
            // remove positions from history
            if (meanVels_.size() > depth_) {
                meanVels_.pop_back();
            }
            
            qom_ = accumulate(meanVels_.begin(), meanVels_.end(), 0.0) / (meanVels_.size());
            
            ci_ = ( -4.0 + (( abs(xMax-xMin) + abs(yMax-yMin) + abs(zMax-zMin) ) / h) ) / 6.0;
            
            symmetry_ = 1.0 - (0.5 * (abs(sqrt(getDistanceToTorso(JOINT_RIGHT_HAND))-sqrt(getDistanceToTorso(JOINT_LEFT_HAND))) + abs(sqrt(getDistanceToTorso(JOINT_RIGHT_ELBOW))-sqrt(getDistanceToTorso(JOINT_LEFT_ELBOW)))) / h);
            
            yMaxHands_ = max(getRelativePositionToTorso(JOINT_RIGHT_HAND).y, getRelativePositionToTorso(JOINT_LEFT_HAND).y);
            
            
        } else {
            newValues_
            = false;
        }
        
    }
}

void ofxKinectFeatures::computeJointDescriptors(ofxOpenNIJoint joint, const float &h){
    ofPoint position = joint.getWorldPosition();
    Joint j = joint.getType();
    
    ofxMocapElement* mocapElement = getElement(j);
    
    //Position
    mocapElement->setPosition(position);
    
    //Filtered position
    mocapElement->setPositionFiltered(applyFilter(mocapElement->getPosition(), mocapElement->getPositionFiltered(), aFilter, bFilter));
    
    //Velocity
    mocapElement->setVelocity(applyFilter(mocapElement->getPosition(), mocapElement->getVelocity(), aLpd1, bLpd1));
    
    //Acceleration
    mocapElement->setAcceleration(applyFilter(mocapElement->getPosition(), mocapElement->getAcceleration(), aLpd2, bLpd2));
    
    //Acceleration along trajectory
    ofPoint acc = mocapElement->getAcceleration()[0];
    ofPoint vel = mocapElement->getVelocity()[0];
    mocapElement->setAccelerationTrajectory(acc.dot(vel) / vel.length());
    
    //Distance to torso
    mocapElement->setDistanceToTorso(getPositionFiltered(j).distanceSquared(getPositionFiltered(JOINT_TORSO)));
    
    //Relative position to torso
    ofPoint relPosToTorso;
    relPosToTorso.x = (position.x - getPositionFiltered(JOINT_TORSO).x) / (h * 1.8);
    relPosToTorso.y = (position.y - getPositionFiltered(JOINT_TORSO).y) / (h * 1.8);
    relPosToTorso.z = -((position.z - getPositionFiltered(JOINT_TORSO).z) / h) / 1.4;
    mocapElement->setRelativePositionToTorso(relPosToTorso);
    
    //TODO: auto hand
    //beatTracker.update(getAccTrVector(JOINT_RIGHT_HAND), get3DFiltPosVector(JOINT_RIGHT_HAND)[coord::Y]);
}

ofxMocapElement* ofxKinectFeatures::getElement(Joint _id){
    vector<ofxMocapElement>::iterator it = find_if(elements_.begin(), elements_.end(), MatchId(_id));
    if (it != elements_.end()){
        return &(*it);
    } else {
        return false;
    }
}

ofPoint ofxKinectFeatures::applyFilter(vector<ofPoint> x, vector<ofPoint> y, float *a, float *b){
    return b[0]*x[0] + b[1]*x[1] + b[2]*x[2] + b[3]*x[3] + b[4]*x[4] - (a[1]*y[0] + a[2]*y[1] + a[3]*y[2] + a[4]*y[3]);
}


template <typename T>
vector<T> ofxKinectFeatures::createVector(T element){
    vector<T> v (depth_);
    fill(v.begin(), v.begin()+depth_, element);
    return v;
}

//Descriptors getters

ofPoint ofxKinectFeatures::getPosition(Joint j){
    if (elements_.empty()) {
        return ofPoint(0,0,0);
    } else {
        return getElement(j)->getPosition()[0];
    }
}

vector<ofPoint> ofxKinectFeatures::getPositionHistory(Joint j){
     if (elements_.empty()) {
         return createVector(ofPoint(0.0,0.0,0.0));
     } else {
         return getElement(j)->getPosition();
     }
}

ofPoint ofxKinectFeatures::getPositionFiltered(Joint j){
    if (elements_.empty()) {
        return ofPoint(0,0,0);
    } else {
        return getElement(j)->getPositionFiltered()[0];
    }
}

vector<ofPoint> ofxKinectFeatures::getPositionFilteredHistory(Joint j){
    if (elements_.empty()) {
        return createVector(ofPoint(0.0,0.0,0.0));
    } else {
        return getElement(j)->getPositionFiltered();
    }
}

ofPoint ofxKinectFeatures::getVelocity(Joint j){
    return getElement(j)->getVelocity()[0];
}

vector<ofPoint> ofxKinectFeatures::getVelocityHistory(Joint j){
    if (elements_.empty()) {
        return createVector(ofPoint(0.0,0.0,0.0));
    } else {
        return getElement(j)->getVelocity();
    }
}

float ofxKinectFeatures::getVelocityMagnitude(Joint j){
    return getElement(j)->getVelocity()[0].length();
}

float ofxKinectFeatures::getVelocityMean(Joint j, int frames){
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

ofPoint ofxKinectFeatures::getAcceleration(Joint j){
    return getElement(j)->getAcceleration()[0];
}

vector<ofPoint> ofxKinectFeatures::getAccelerationHistory(Joint j){
    if (elements_.empty()) {
        return createVector(ofPoint(0.0,0.0,0.0));
    } else {
        return getElement(j)->getAcceleration();
    }
}

float ofxKinectFeatures::getAccelerationMagnitude(Joint j){
    return getElement(j)->getAcceleration()[0].length();
}

float ofxKinectFeatures::getAccelerationMean(Joint j, int frames){
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

float ofxKinectFeatures::getAccelerationTrajectory(Joint j){
    if (elements_.empty()){
        return 0.0;
    } else {
        return getElement(j)->getAccelerationTrajectory()[0];
    }
}

vector<float> ofxKinectFeatures::getAccelerationTrajectoryHistory(Joint j){
    if (elements_.empty()) {
        return createVector(0.0f);
    } else {
        return getElement(j)->getAccelerationTrajectory();
    }
}

float ofxKinectFeatures::getAccelerationTrajectoryMean(Joint j, int frames){
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

float ofxKinectFeatures::getDistanceToTorso(Joint j){
    if (elements_.empty()){
        return 0.0;
    } else {
        return getElement(j)->getDistanceToTorso()[0];
    }
}

vector<float> ofxKinectFeatures::getDistanceToTorsoHistory(Joint j){
    if (elements_.empty()) {
        return createVector(0.0f);
    } else {
        return getElement(j)->getDistanceToTorso();
    }
}

ofPoint ofxKinectFeatures::getRelativePositionToTorso(Joint j){
    if (elements_.empty()){
        return ofPoint(0.0,0.0,0.0);
    } else {
        return getElement(j)->getRelativePositionToTorso()[0];
    }
}

vector<ofPoint> ofxKinectFeatures::getRelativePositionToTorsoHistory(Joint j){
    if (elements_.empty()) {
        return createVector(ofPoint(0.0,0.0,0.0));
    } else {
        return getElement(j)->getRelativePositionToTorso();
    }
}

float ofxKinectFeatures::getQom(){
    return qom_;
}

float ofxKinectFeatures::getCI(){
    return ci_;
}

float ofxKinectFeatures::getSymmetry(){
    return symmetry_;
}

float ofxKinectFeatures::getYMaxHands(){
    return yMaxHands_;
}

bool ofxKinectFeatures::isNewDataAvailable(){
    return newValues_;
}