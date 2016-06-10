/*
 KinectFeatures
 Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of KinectFeatures, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>
 
 KinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 KinectFeatures is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the KinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
 */

#ifndef openNiWorm_KinectFeatures_h
#define openNiWorm_KinectFeatures_h

#include "MocapElement.h"
#include <map>
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

class KinectFeatures {
public:
    KinectFeatures();
    KinectFeatures(int head, int torso);
    
    void setup(int head, int torso);
    void update(map<int, MocapPoint> joints);
    
    void setFilterLevel(int filterLevel);
    
    void setDepth(int depth);
    int getDepth();
    
    //DESCIPTOR GETTERS
    //JOINT DESCRIPTORS
    MocapPoint getPosition(int j);
    vector<MocapPoint> getPositionHistory(int j);
    vector<MocapPoint> getPositionHistory(int j, int frames);
    
	MocapPoint getPositionFiltered(int j);
    vector<MocapPoint> getPositionFilteredHistory(int j);
    vector<MocapPoint> getPositionFilteredHistory(int j, int frames);
    
	MocapPoint getVelocity(int j);
    vector<MocapPoint> getVelocityHistory(int j);
    vector<MocapPoint> getVelocityHistory(int j, int frames);
    float getVelocityMagnitude(int j);
    MocapPoint getVelocityMean(int j, int frames = 30);
    float getVelocityMagnitudeMean(int j, int frames = 30);
    
	MocapPoint getAcceleration(int j);
    vector<MocapPoint> getAccelerationHistory(int j);
    vector<MocapPoint> getAccelerationHistory(int j, int frames);
    float getAccelerationMagnitude(int j);
    MocapPoint getAccelerationMean(int j, int frames = 30);
    float getAccelerationMagnitudeMean(int j, int frames = 30);
    
    float getAccelerationTrajectory(int j);
    vector<float> getAccelerationTrajectoryHistory(int j);
    vector<float> getAccelerationTrajectoryHistory(int j, int frames);
    float getAccelerationTrajectoryMean(int j, int frames = 30);
    
	MocapPoint getRelativePositionToTorso(int j);
    vector<MocapPoint> getRelativePositionToTorsoHistory(int j);
    vector<MocapPoint> getRelativePositionToTorsoHistory(int j, int frames);
    
    //SPECIAL DESCRIPTORS
    float getAngle(int j1, int j2, int j3);
    
    //OVERALL DESCRIPTORS
    float getQom();
    vector<float> getQomHistory();
    vector<float> getQomHistory(int frames);
    float getCI();
    vector<float> getCIHistory();
    vector<float> getCIHistory(int frames);
//    float getSymmetry();
//    float getYMaxHands();
    
    bool isNewDataAvailable();
    
private:
    int head_, torso_;

	MocapElement* getElement(int _id);
    
    template <typename T>
    vector<T> createVector (T element);
    
    float *aFilter;
    float *bFilter;
    float *aLpd1;
    float *bLpd1;
    float *aLpd2;
    float *bLpd2;
    
    //overall descriptors
    vector<float> qom_, ci_;//, symmetry_, yMaxHands_;
    vector<float> meanVels_;
    
    bool newValues_;
    
    vector<MocapElement> elements_;
    
    int depth_;
    
    void computeJointDescriptors(int jointId, MocapPoint jointPos, const float &h);
    MocapPoint applyFilter (vector<MocapPoint> x, vector<MocapPoint> y, float *a, float *b);
   
	//Functor to look for mocap elements matching a Joint
    struct MatchId
    {
        MatchId(const int& j) : j_(j) {}
        bool operator()(MocapElement& obj) const
        {
            return obj.getElementId() == j_;
        }
    private:
        const int& j_;
    };
};

#endif
