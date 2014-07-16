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

