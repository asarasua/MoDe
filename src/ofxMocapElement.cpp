//
//  ofxMocapElement.cpp
//  openNiFeatures
//
//  Created by Álvaro Sarasúa Berodia on 10/09/13.
//
//

#include "ofxMocapElement.h"

ofxMocapElement::ofxMocapElement(){
    ofPoint zeros = ofPoint(0.0,0.0,0.0);
    for (int i = i; i < historyDepth; i++) {
        setPos(zeros);
        setFiltPos(zeros);
        setVel(zeros);
        setAcc(zeros);
    }
    setRelPosToTorso(zeros);
}

ofxMocapElement::ofxMocapElement(int depth){
    historyDepth = depth;
    ofPoint zeros = ofPoint(0.0,0.0,0.0);
    for (int i = 1; i < historyDepth; i++) {
        setPos(zeros);
        setFiltPos(zeros);
        setVel(zeros);
        setAcc(zeros);
    }
    setRelPosToTorso(zeros);
}

vector<ofPoint> ofxMocapElement::getPos(){
    return pos;
}

void ofxMocapElement::setPos(ofPoint currentPosition){
    // Add position to history
	if (pos.size() <= historyDepth) {
        pos.insert(pos.begin(), currentPosition);
	}
    
	// remove positions from history
	if (pos.size() > historyDepth) {
		pos.pop_back();
	}
}

vector<ofPoint> ofxMocapElement::getFiltPos(){
    return pos;
}

void ofxMocapElement::setFiltPos(ofPoint currentFiltPos){
    // Add position to history
	if (filtPos.size() <= historyDepth) {
        filtPos.insert(filtPos.begin(), currentFiltPos);
	}
    
	// remove positions from history
	if (filtPos.size() > historyDepth) {
		filtPos.pop_back();
	}
}

vector<ofPoint> ofxMocapElement::getVel(){
    return vel;
}

void ofxMocapElement::setVel(ofPoint currentVel){
    // Add position to history
	if (vel.size() <= historyDepth) {
        vel.insert(vel.begin(), currentVel);
	}
    
	// remove positions from history
	if (vel.size() > historyDepth) {
		vel.pop_back();
	}
}

vector<ofPoint> ofxMocapElement::getAcc(){
    return acc;
}

void ofxMocapElement::setAcc(ofPoint currentAcc){
    // Add position to history
	if (acc.size() <= historyDepth) {
        acc.insert(acc.begin(), currentAcc);
	}
    
	// remove positions from history
	if (acc.size() > historyDepth) {
		acc.pop_back();
	}
}

vector<float> ofxMocapElement::getAccTr(){
    return accTr;
}

void ofxMocapElement::setAccTr(float currentAccTr){
    if (accTr.size() <= historyDepth) {
        accTr.insert(accTr.begin(), currentAccTr);
	}
    
	// remove positions from history
	if (accTr.size() > historyDepth) {
		accTr.pop_back();
	}
}

float ofxMocapElement::getDistToTorso(){
    return distToTorso;
}

void ofxMocapElement::setDistToTorso(float currentDistToTorso){
    distToTorso = currentDistToTorso;
}

ofPoint ofxMocapElement::getRelPosToTorso(){
    return relPosToTorso;
}

void ofxMocapElement::setRelPosToTorso(ofPoint currentRelPosToTorso){
    relPosToTorso = currentRelPosToTorso;
}

//string ofxMocapElement::getElementName(){
//    return elementName;
//}
//
//void ofxMocapElement::setElementName(string name){
//    elementName = name;
//}

unsigned int ofxMocapElement::getElementId(){
    return elementId;
}

void ofxMocapElement::setElementId(Joint newId){
    elementId = newId;
}


void ofxMocapElement::setHistoryDepth(int depth){
    historyDepth = depth;
}

