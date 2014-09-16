/*
 ofxKinectFeatures
 Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of ofxKinectFeatures, created and maintained by Álvaro Sarasúa <http://alvarosarasua.wordpress.com>
 
 ofxKinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 ofxKinectFeatures is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the ofxKinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
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
    float getVelocityMean(Joint j, int frames = 30);
    
    ofPoint getAcceleration(Joint j);
    vector<ofPoint> getAccelerationHistory(Joint j);
    float getAccelerationMagnitude(Joint j);
    float getAccelerationMean(Joint j, int frames = 30);
    
    float getAccelerationTrajectory(Joint j);
    vector<float> getAccelerationTrajectoryHistory(Joint j);
    float getAccelerationTrajectoryMean(Joint j, int frames = 30);
    
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
