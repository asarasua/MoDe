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

enum
{
    FEAT_POSITION,
    FEAT_POSITION_FILTERED,
	FEAT_VELOCITY,
	FEAT_VELOCITY_MAG,
	FEAT_VELOCITY_MEAN,
	FEAT_ACCELERATION,
	FEAT_ACCELERATION_MAG,
	FEAT_ACCELERATION_MEAN,
	FEAT_ACCELERATION_TRAJECTORY,
	FEAT_ACCELERATION_TRAJECTORY_MEAN,
	FEAT_RELATIVEPOSTOTORSO,
	FEAT_QOM,
	FEAT_CI
};

#if OPENFRAMEWORKS
class MocapEvent : public ofEventArgs, public MocapExtreme {
    
public:
    MocapEvent() {}
    static ofEvent <MocapEvent> events;
};
#endif

class KinectFeatures {
	vector <class ExtremeListener *> extremeListeners;
public:
    KinectFeatures();
    KinectFeatures(int head, int torso, int depth);
    
    void setup(int head, int torso, int depth);
    void update(map<int, MocapPoint> joints);
#if OPENFRAMEWORKS
    void update(map<int, ofPoint> joints){
        map<int, MocapPoint> jointsMap;
        for (auto joint : joints)
            jointsMap[joint.first] = MocapPoint(joint.second.x, joint.second.y, joint.second.z);
        
        update(jointsMap);
    }
#endif
    
    void setFilterLevel(int filterLevel);
    
    void setDepth(int depth);
    int getDepth();

	void addExtremeListener(ExtremeListener* extremeListener);
    
    //SPECIAL DESCRIPTORS
    float getAngle(int j1, int j2, int j3);
    MocapPoint getAccelerationCrest(int j);
    MocapPoint getRms(int j, int frames);
    
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
    
    const MocapElement getJoint(int jointId);
    
private:
    int head_, torso_;

	MocapElement* getElement(int jointId);
    
    template <typename T>
    vector<T> createVector (T element);
    
    float *aFilter;
    float *bFilter;
    float *aLpd1;
    float *bLpd1;
    float *aLpd2;
    float *bLpd2;
    
    //overall descriptors
    MocapDescriptor<float> qom, ci;//, symmetry_, yMaxHands_;
    vector<float> meanVels_;
    
    bool newValues_;
    
    vector<MocapElement> elements_;
    
    int depth_;
    
    void computeJointDescriptors(int jointId, MocapPoint jointPos, const float &h);
    MocapPoint applyFilter (vector<MocapPoint> x, vector<MocapPoint> y, float *a, float *b);
	void notify(vector<MocapExtreme> newExtremes, int jointId, int featId);
   
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

class ExtremeListener {
public:
	ExtremeListener(KinectFeatures * featExt) {
		featExtractor = featExt;
		featExtractor->addExtremeListener(this);
	}
	virtual void newExtreme(MocapExtreme extreme) = 0;
protected:
	KinectFeatures *featExtractor;
};

#endif
