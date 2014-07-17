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
#include "ofxMocapElement.h"

ofxMocapElement::ofxMocapElement(){
    ofPoint zeros = ofPoint(0.0,0.0,0.0);
    for (int i = i; i < historyDepth; i++) {
        setPosition(zeros);
        setPositionFiltered(zeros);
        setVelocity(zeros);
        setAcceleration(zeros);
        setRelativePositionToTorso(zeros);
    }
}

ofxMocapElement::ofxMocapElement(int depth){
    historyDepth = depth;
    ofPoint zeros = ofPoint(0.0,0.0,0.0);
    for (int i = 1; i < historyDepth; i++) {
        setPosition(zeros);
        setPositionFiltered(zeros);
        setVelocity(zeros);
        setAcceleration(zeros);
        setRelativePositionToTorso(zeros);
    }
}

unsigned int ofxMocapElement::getElementId(){
    return elementId;
}

void ofxMocapElement::setElementId(Joint newId){
    elementId = newId;
}


void ofxMocapElement::setHistoryDepth(int depth){
    historyDepth = depth;
}

//Getters & setters

vector<ofPoint> ofxMocapElement::getPosition(){
    return position_;
}

void ofxMocapElement::setPosition(ofPoint position){
    // Add position to history
	if (position_.size() <= historyDepth) {
        position_.insert(position_.begin(), position);
	}
    
	// remove positions from history
	if (position_.size() > historyDepth) {
		position_.pop_back();
	}
}

vector<ofPoint> ofxMocapElement::getPositionFiltered(){
    return positionFiltered_;
}

void ofxMocapElement::setPositionFiltered(ofPoint positionFiltered){
    // Add position to history
	if (positionFiltered_.size() <= historyDepth) {
        positionFiltered_.insert(positionFiltered_.begin(), positionFiltered);
	}
    
	// remove positions from history
	if (positionFiltered_.size() > historyDepth) {
		positionFiltered_.pop_back();
	}
}

vector<ofPoint> ofxMocapElement::getVelocity(){
    return velocity_;
}

void ofxMocapElement::setVelocity(ofPoint velocity){
    // Add position to history
	if (velocity_.size() <= historyDepth) {
        velocity_.insert(velocity_.begin(), velocity);
	}
    
	// remove positions from history
	if (velocity_.size() > historyDepth) {
		velocity_.pop_back();
	}
}

vector<ofPoint> ofxMocapElement::getAcceleration(){
    return acceleration_;
}

void ofxMocapElement::setAcceleration(ofPoint acceleration){
    // Add position to history
	if (acceleration_.size() <= historyDepth) {
        acceleration_.insert(acceleration_.begin(), acceleration);
	}
    
	// remove positions from history
	if (acceleration_.size() > historyDepth) {
		acceleration_.pop_back();
	}
}

vector<float> ofxMocapElement::getAccelerationTrajectory(){
    return accelerationTrajectory_;
}

void ofxMocapElement::setAccelerationTrajectory(float accelerationTrajectory){
    if (accelerationTrajectory_.size() <= historyDepth) {
        accelerationTrajectory_.insert(accelerationTrajectory_.begin(), accelerationTrajectory);
	}
    
	// remove positions from history
	if (accelerationTrajectory_.size() > historyDepth) {
		accelerationTrajectory_.pop_back();
	}
}

vector<float> ofxMocapElement::getDistanceToTorso(){
    return distanceToTorso_;
}

void ofxMocapElement::setDistanceToTorso(float distanceToTorso){
    if (distanceToTorso_.size() <= historyDepth) {
        distanceToTorso_.insert(distanceToTorso_.begin(), distanceToTorso);
	}
    
	// remove positions from history
	if (distanceToTorso_.size() > historyDepth) {
		distanceToTorso_.pop_back();
	}
}

vector<ofPoint> ofxMocapElement::getRelativePositionToTorso(){
    return relativePositionToTorso_;
}

void ofxMocapElement::setRelativePositionToTorso(ofPoint relativePositionToTorso){
    if (relativePositionToTorso_.size() <= historyDepth) {
        relativePositionToTorso_.insert(relativePositionToTorso_.begin(), relativePositionToTorso);
	}
    
	// remove positions from history
	if (relativePositionToTorso_.size() > historyDepth) {
		relativePositionToTorso_.pop_back();
	}
}

