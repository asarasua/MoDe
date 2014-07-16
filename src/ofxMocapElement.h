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
    
    string getElementName();
    void setElementName(string name);
    
    unsigned int getElementId();
    void setElementId(Joint newId);
    
    void setHistoryDepth(int depth);    
    
    vector<ofPoint> getPos();
    void setPos(ofPoint currentPosition);
    
    vector<ofPoint> getFiltPos();
    void setFiltPos(ofPoint currentFiltPos);
    
    vector<ofPoint> getVel();
    void setVel(ofPoint currentVel);
    
    vector<ofPoint> getAcc();
    void setAcc(ofPoint currentAcc);
    
    vector<float> getAccTr();
    void setAccTr(float currentAccTr);
    
    float getDistToTorso();
    void setDistToTorso(float currentDistToTorso);
    
    ofPoint getRelPosToTorso();
    void setRelPosToTorso(ofPoint currentRelPosToTorso);

private:
    int historyDepth;
    
    //string elementName;
    Joint elementId;
    
    vector<ofPoint> pos;
    vector<ofPoint> filtPos;
    vector<ofPoint> vel;
    vector<ofPoint> acc;
    vector<float> accTr;
    float distToTorso;
    ofPoint relPosToTorso;
};

#endif
