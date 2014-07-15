/*
 
 ofxKinectFeatures
 Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 Copyright (c) 2014 Álvaro Sarasúa <alvaro.sarasua@upf.edu>
 
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
    newValues = false;
    setDepth(30);
    aFilter = lpf_soft_a;
    bFilter = lpf_soft_b;
    aLpd1 = lpd1_soft_a;
    bLpd1 = lpd1_soft_b;
    aLpd2 = lpd2_soft_a;
    bLpd2 = lpd2_soft_b;
    //TODO
}

void ofxKinectFeatures::setKinect(ofxOpenNI* device){
    kinect = device;
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

void ofxKinectFeatures::setDepth(int newDepth){
    depth = newDepth;
}

int ofxKinectFeatures::getDepth(){
    return depth;
}

void ofxKinectFeatures::update(){
    if (kinect->getNumTrackedUsers()) {
        ofxOpenNIUser user = kinect->getTrackedUser(0);
        
        //Initialize elements
        if (elements.empty()) {
            for (int i = 0; i < user.getNumJoints(); i++) {
                ofxOpenNIJoint joint = user.getJoint((Joint)i);
                ofxMocapElement newElement(depth);
                newElement.setElementId(joint.getType());
                elements.push_back(newElement);
            }
        }
        
        //Compute descriptors
        //Hard-coded way to check if skeleton (bSkeleton doesn't work) and new data (isNewDataAvailable doesn't work)
        if (user.getJoint((Joint)0).getWorldPosition() != ofPoint(0,0,0) &&
            user.getJoint((Joint)0).getWorldPosition() != getElement((Joint)0)->getPos()[0] ) {
            newValues = true;
            
            //Joint descriptors
            ofPoint headPos = user.getJoint(JOINT_HEAD).getWorldPosition();
            ofPoint torsoPos = user.getJoint(JOINT_TORSO).getWorldPosition();
            float h = headPos.distance(torsoPos);
            
            float meanVel = 0.0; //for qom
            
            //for CI
            float xMax, yMax, zMax = numeric_limits<float>::min();
            float xMin, yMin, zMin = numeric_limits<float>::max();
            
            for (int i = 0; i < user.getNumJoints(); i++) {
                ofxOpenNIJoint joint = user.getJoint((Joint)i);
                computeJointDescriptors(joint, h);
                
                //qom
                meanVel += get3DVel((Joint)i).length();
                
                //ci
                if ( getFiltPos((Joint) i, 0) > xMax ){
                    xMax = getFiltPos((Joint) i, coord::X);
                }
                if ( getFiltPos((Joint) i, 1) > yMax ){
                    yMax = getFiltPos((Joint) i, coord::Y);
                }
                if ( getFiltPos((Joint) i, 2) > zMax ){
                    zMax = getFiltPos((Joint) i, coord::Z);
                }
                if ( getFiltPos((Joint) i, 0) < xMin ){
                    xMax = getFiltPos((Joint) i, coord::X);
                }
                if ( getFiltPos((Joint) i, 1) < yMin ){
                    yMin = getFiltPos((Joint) i, coord::Y);
                }
                if ( getFiltPos((Joint) i, 2) < zMin ){
                    zMin = getFiltPos((Joint) i, coord::Z);
                }
            }
            
            // Add position to history
            if (meanVels.size() <= depth) {
                meanVels.insert(meanVels.begin(), meanVel/user.getNumJoints());
            }
            
            // remove positions from history
            if (meanVels.size() > depth) {
                meanVels.pop_back();
            }
            
            qom = accumulate(meanVels.begin(), meanVels.end(), 0.0) / (meanVels.size());
            
            ci = ( -4.0 + (( abs(xMax-xMin) + abs(yMax-yMin) + abs(zMax-zMin) ) / h) ) / 6.0;
            
            symmetry = 1.0 - (0.5 * (abs(getDistToTorso(JOINT_RIGHT_HAND)-getDistToTorso(JOINT_LEFT_HAND)) + abs(getDistToTorso(JOINT_RIGHT_ELBOW)-getDistToTorso(JOINT_LEFT_ELBOW))) / h);
            
            yMaxHands = max(getRelPosToTorso(JOINT_RIGHT_HAND, 1), getRelPosToTorso(JOINT_LEFT_HAND, 1));
            
            
        } else {
            newValues = false;
        }
        
        computeOverallDescriptors();
    }
}

void ofxKinectFeatures::computeJointDescriptors(ofxOpenNIJoint joint, const float &h){
    ofPoint position = joint.getWorldPosition();
    Joint j = joint.getType();
    
    ofxMocapElement* mocapElement = getElement(j);
    
    //Position
    mocapElement->setPos(position);
    
    //Filtered position
    mocapElement->setFiltPos(applyFilter(mocapElement->getPos(), mocapElement->getFiltPos(), aFilter, bFilter));
    
    //Velocity
    mocapElement->setVel(applyFilter(mocapElement->getPos(), mocapElement->getVel(), aLpd1, bLpd1));
    
    //Acceleration
    mocapElement->setAcc(applyFilter(mocapElement->getPos(), mocapElement->getAcc(), aLpd2, bLpd2));
    
    //Acceleration along trajectory
    ofPoint acc = mocapElement->getAcc()[0];
    ofPoint vel = mocapElement->getVel()[0];
    mocapElement->setAccTr(acc.dot(vel) / vel.length());    
    
    //Distance to torso
    mocapElement->setDistToTorso(get3DFiltPos(j).distanceSquared(get3DFiltPos(JOINT_TORSO)));
    
    //Relative position to torso
    ofPoint relPosToTorso;
    relPosToTorso.x = (position.x - getFiltPos(JOINT_TORSO, coord::X)) / (h * 1.8);
    relPosToTorso.y = (position.y - getFiltPos(JOINT_TORSO, coord::Y)) / (h * 1.8);
    relPosToTorso.z = -((position.z - getFiltPos(JOINT_TORSO, coord::Z)) / h) / 1.4;
    mocapElement->setRelPosToTorso(relPosToTorso);
    
    //TODO: auto hand
    //beatTracker.update(getAccTrVector(JOINT_RIGHT_HAND), get3DFiltPosVector(JOINT_RIGHT_HAND)[coord::Y]);
}

void ofxKinectFeatures::computeOverallDescriptors(){
    
}

ofxMocapElement* ofxKinectFeatures::getElement(Joint _id){
    vector<ofxMocapElement>::iterator it = find_if(elements.begin(), elements.end(), MatchId(_id));
    if (it != elements.end()){
        return &(*it);
    } else {
        return false;
    }
}

ofPoint ofxKinectFeatures::applyFilter(vector<ofPoint> x, vector<ofPoint> y, float *a, float *b){
    return b[0]*x[0] + b[1]*x[1] + b[2]*x[2] + b[3]*x[3] + b[4]*x[4] - (a[1]*y[0] + a[2]*y[1] + a[3]*y[2] + a[4]*y[3]);
}

//Feature getters
float ofxKinectFeatures::getPos(Joint j, unsigned int axis){
    if (elements.empty()){
        return 0.0;
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        if (axis == 0) {
            return mocapElement->getPos()[0].x;
        } else if (axis == 1){
            return mocapElement->getPos()[0].y;
        } else {
            return mocapElement->getPos()[0].z;
        }
    }
}

ofPoint ofxKinectFeatures::get3DPos(Joint j){
    if (elements.empty()) {
        return ofPoint(0,0,0);
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        return mocapElement->getPos()[0];
    }
}

float ofxKinectFeatures::getFiltPos(Joint j, unsigned int axis){
    if (elements.empty()){
        return 0.0;
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        if (axis == 0) {
            return mocapElement->getFiltPos()[0].x;
        } else if (axis == 1){
            return mocapElement->getFiltPos()[0].y;
        } else {
            return mocapElement->getFiltPos()[0].z;
        }
    }
}

ofPoint ofxKinectFeatures::get3DFiltPos(Joint j){
    if (elements.empty()) {
        return ofPoint(0,0,0);
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        return mocapElement->getFiltPos()[0];
    }
}

vector<ofPoint> ofxKinectFeatures::get3DFiltPosVector(Joint j){
    //TODO automatic?
    vector<ofPoint> filtPos (5);
    if (elements.empty()) {
        fill(filtPos.begin(), filtPos.begin()+5, ofPoint(0,0,0));
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        for (int i = 0; i < 5; i++) {
            filtPos[i] = mocapElement->getFiltPos()[i];
        }
    }
    
    return filtPos;
}

float ofxKinectFeatures::getVel(Joint j, unsigned int axis){
    if (elements.empty()){
        return 0.0;
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        if (axis == 0) {
            return mocapElement->getVel()[0].x;
        } else if (axis == 1){
            return mocapElement->getVel()[0].y;
        } else {
            return mocapElement->getVel()[0].z;
        }
    }
}

ofPoint ofxKinectFeatures::get3DVel(Joint j){
    ofxMocapElement* mocapElement = getElement(j);
    return mocapElement->getVel()[0];
}

float ofxKinectFeatures::getAcc(Joint j, unsigned int axis){
    if (elements.empty()){
        return 0.0;
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        if (axis == 0) {
            return mocapElement->getAcc()[0].x;
        } else if (axis == 1){
            return mocapElement->getAcc()[0].y;
        } else {
            return mocapElement->getAcc()[0].z;
        }
    }
}

vector<float> ofxKinectFeatures::getAccVector(Joint j, unsigned int axis){
    //TODO automatic?
    vector<float> acc (5);
    if (elements.empty()) {
        fill(acc.begin(), acc.begin()+5, 0.0);
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        if (axis == 0) {
            for (int i = 0; i < 5; i++) {
                acc[i] = mocapElement->getAcc()[i].x;
            }
        } else if (axis == 1){
            for (int i = 0; i < 5; i++) {
                acc[i] = mocapElement->getAcc()[i].y;
            }
        } else {
            for (int i = 0; i < 5; i++) {
                acc[i] = mocapElement->getAcc()[i].z;
            }
        }
    }
    
    return acc;
}

float ofxKinectFeatures::getAccTr(Joint j){
    if (elements.empty()){
        return 0.0;
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        return mocapElement->getAccTr()[0];
    }
}

vector<float> ofxKinectFeatures::getAccTrVector(Joint j){
    //TODO automatic?
    vector<float> accTr (5);
    if (elements.empty()) {
        fill(accTr.begin(), accTr.begin()+5, 0.0);
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        for (int i = 0; i < 5; i++) {
            accTr[i] = mocapElement->getAccTr()[i];
        }
    }
    return accTr;
}

float ofxKinectFeatures::getDistToTorso(Joint j){
    ofxMocapElement* mocapElement = getElement(j);
    return mocapElement->getDistToTorso();
}

float ofxKinectFeatures::getRelPosToTorso(Joint j, unsigned int axis){
    if (elements.empty()){
        return 0.0;
    } else {
        ofxMocapElement* mocapElement = getElement(j);
        if (axis == 0) {
            return mocapElement->getRelPosToTorso().x;
        } else if (axis == 1){
            return mocapElement->getRelPosToTorso().y;
        } else {
            return mocapElement->getRelPosToTorso().z;
        }
    }
}

float ofxKinectFeatures::getQom(){
    return qom;
}

float ofxKinectFeatures::getCI(){
    return ci;
}

float ofxKinectFeatures::getSymmetry(){
    return symmetry;
}

float ofxKinectFeatures::getYMaxHands(){
    return yMaxHands;
}

bool ofxKinectFeatures::isNewDataAvailable(){
    return newValues;
}