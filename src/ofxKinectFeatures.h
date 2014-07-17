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

#ifndef openNiWorm_ofxKinectFeatures_h
#define openNiWorm_ofxKinectFeatures_h

#include "ofMain.h"
#include "ofxMocapElement.h"
#include <numeric>

namespace filter
{
    enum
    {
        SOFT = 0,
        MED = 1,
        HARD = 2
    };
} // namespace filter

class ofxKinectFeatures {
public:
    ofxKinectFeatures();
    
    void update();
    ofxMocapElement* getElement(Joint _id);
    
    void setKinect(ofxOpenNI* kinect);
    void setFilterLevel(int filterLevel);
    
    void setDepth(int depth);
    int getDepth();
    
    //DESCIPTOR GETTERS
    //JOINT DESCRIPTORS
    ofPoint getPosition(Joint j);
    vector<ofPoint> getPositionHistory(Joint j);
    
    ofPoint getPositionFiltered(Joint j);
    vector<ofPoint> getPositionFilteredHistory(Joint j);
    
    ofPoint getVelocity(Joint j);
    vector<ofPoint> getVelocityHistory(Joint j);
    float getVelocityMagnitude(Joint j);
    
    ofPoint getAcceleration(Joint j);
    vector<ofPoint> getAccelerationHistory(Joint j);
    float getAccelerationMagnitude(Joint j);
    
    float getAccelerationTrajectory(Joint j);
    vector<float> getAccelerationTrajectoryHistory(Joint j);
    
    float getDistanceToTorso(Joint j);
    vector<float> getDistanceToTorsoHistory(Joint j);
    
    ofPoint getRelativePositionToTorso(Joint j);
    vector<ofPoint> getRelativePositionToTorsoHistory(Joint j);
    
    //OVERALL DESCRIPTORS
    float getQom();
    float getCI();
    float getSymmetry();
    float getYMaxHands();
    
    bool isNewDataAvailable();
    
private:
    template <typename T>
    vector<T> createVector (T element);
    
    float *aFilter;
    float *bFilter;
    float *aLpd1;
    float *bLpd1;
    float *aLpd2;
    float *bLpd2;
    
    //overall descriptors
    float qom_, ci_, symmetry_, yMaxHands_;
    vector<float> meanVels_;
    
    bool newValues_;
    
    ofxOpenNI* kinect_;
    vector<ofxMocapElement> elements_;
    
    int depth_;
    
    void computeJointDescriptors(ofxOpenNIJoint joint, const float &h);
    ofPoint applyFilter (vector<ofPoint> x, vector<ofPoint> y, float *a, float *b);
    
    //Functor to look for mocap elements matching a Joint
    struct MatchId
    {
        MatchId(const Joint& j) : j_(j) {}
        bool operator()(ofxMocapElement& obj) const
        {
            return obj.getElementId() == j_;
        }
    private:
        const Joint& j_;
    };
};

#endif
