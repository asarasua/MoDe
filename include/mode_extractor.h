/*
 MoDeExtractor
 Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of MoDeExtractor, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>
 
 MoDeExtractor is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 MoDeExtractor is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the MoDeExtractor SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
 */

#ifndef __APPLE__
#ifdef MODE_AS_DLL
#define MODE_DLLEXPORT __declspec(dllexport)
#else
#define MODE_DLLEXPORT
#endif
#endif

#ifndef MoDeExtractor_h
#define MoDeExtractor_h

#include "mode_joint.h"
#include <map>
#include <numeric>
#include <fstream>



namespace MoDe {
    
    #define NO_JOINT 999
    
    enum
    {
        FILTER_SOFT,
        FILTER_MED,
        FILTER_HARD
    };

    class MoDeExtractor {
        vector <class ExtremeListener *> extremeListeners;
    public:
        MoDeExtractor();
        MoDeExtractor(int head, int torso, int depth);
        
        void setup(int head, int torso, int depth);
        void update(map<int, MoDePoint> joints);
    #if OPENFRAMEWORKS
        void update(map<int, ofPoint> joints){
            map<int, MoDePoint> jointsMap;
            for (auto joint : joints)
                jointsMap[joint.first] = MoDePoint(joint.second.x, joint.second.y, joint.second.z);
            
            update(jointsMap);
        }
    #endif
        
        void setFilterLevel(int filterLevel);
        
        void setDepth(int depth);
        int getDepth();

        void addExtremeListener(ExtremeListener* extremeListener);
        
        //SPECIAL DESCRIPTORS
        float getAngle(int j1, int j2, int j3);
        
        //OVERALL DESCRIPTORS
        float getQom();
        vector<float> getQomHistory();
        vector<float> getQomHistory(int frames);
        float getCI();
        vector<float> getCIHistory();
        vector<float> getCIHistory(int frames);
        
        bool isNewDataAvailable();
        
        MoDeJoint getJoint(int jointId);
        
    private:
		ofstream myfile;
        int head_, torso_;

        MoDeJoint* getElement(int jointId);
        
        template <typename T>
        vector<T> createVector (T element);
        
        float *aFilter;
        float *bFilter;
        float *aLpd1;
        float *bLpd1;
        float *aLpd2;
        float *bLpd2;
        
        //overall descriptors
        MoDeDescriptor<float> qom, ci;//, symmetry_, yMaxHands_;
        vector<float> meanVels_;
        
        bool newValues_;
        
        vector<MoDeJoint> elements_;
        
        int depth_;
        void notify(vector<MoDeExtreme> newExtremes, int jointId, int featId);
       
        //Functor to look for mocap elements matching a Joint
        struct MatchId
        {
            MatchId(const int& j) : j_(j) {}
            bool operator()(MoDeJoint& obj) const
            {
                return obj.getElementId() == j_;
            }
        private:
            const int& j_;
        };
    };

    class ExtremeListener {
    public:
		ExtremeListener() {};
        ExtremeListener(MoDeExtractor * featExt) {
            featExtractor = featExt;
            featExtractor->addExtremeListener(this);
        }
        virtual void newExtreme(MoDeExtreme extreme) = 0;
    protected:
        MoDeExtractor *featExtractor;
    };

    #if OPENFRAMEWORKS
    typedef MoDeExtractor ofxMoDe;
    class ofxMoDeEvent : public ofEventArgs, public MoDeExtreme {
        
    public:
        ofxMoDeEvent() {}
        static ofEvent <ofxMoDeEvent> events;
    };
    #endif
    
}

#endif
