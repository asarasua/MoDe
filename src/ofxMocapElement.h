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

#ifndef openNiFeatures_ofxMocapElement_h
#define openNiFeatures_ofxMocapElement_h

#include "ofMain.h"
#include "ofxOpenNI.h"

class ofxMocapElement{
public:
    ofxMocapElement();
    ofxMocapElement(int depth);
    
    unsigned int getElementId();
    void setElementId(Joint newId);
    
    void setHistoryDepth(int depth);    
    
    vector<ofPoint> getPosition();
    void setPosition(ofPoint position);
    
    vector<ofPoint> getPositionFiltered();
    void setPositionFiltered(ofPoint positionFiltered);
    
    vector<ofPoint> getVelocity();
    void setVelocity(ofPoint velocity);
    
    vector<ofPoint> getAcceleration();
    void setAcceleration(ofPoint acceleration);
    
    vector<float> getAccelerationTrajectory();
    void setAccelerationTrajectory(float accelerationTrajectory);
    
    vector<float> getDistanceToTorso();
    void setDistanceToTorso(float distanceToTorso);
    
    vector<ofPoint> getRelativePositionToTorso();
    void setRelativePositionToTorso(ofPoint relativePositionToTorso);

private:
    int historyDepth;
    
    //string elementName;
    Joint elementId;
    
    vector<ofPoint> position_;
    vector<ofPoint> positionFiltered_;
    vector<ofPoint> velocity_;
    vector<ofPoint> acceleration_;
    vector<float> accelerationTrajectory_;
    vector<float> distanceToTorso_;
    vector<ofPoint> relativePositionToTorso_;
};

#endif
