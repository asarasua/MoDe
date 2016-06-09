/*
 ofxKinectFeatures
 Copyright © 2014 Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of ofxKinectFeatures, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>
 
 ofxKinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 ofxKinectFeatures is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the ofxKinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
 */

#ifndef openNiFeatures_ofxMocapElement_h
#define openNiFeatures_ofxMocapElement_h

#include "mocapPoint.h"

class ofxMocapElement{
public:
    ofxMocapElement();
    ofxMocapElement(int elementId, int depth);
    
    unsigned int getElementId();
    void setElementId(int newId);
    
    void setHistoryDepth(int depth);    
    
    vector<MocapPoint> getPosition();
    void setPosition(MocapPoint position);
    
    vector<MocapPoint> getPositionFiltered();
    void setPositionFiltered(MocapPoint positionFiltered);
    
    vector<MocapPoint> getVelocity();
    void setVelocity(MocapPoint velocity);
    
    vector<MocapPoint> getAcceleration();
    void setAcceleration(MocapPoint acceleration);
    
    vector<float> getAccelerationTrajectory();
    void setAccelerationTrajectory(float accelerationTrajectory);
    
    vector<float> getDistanceToTorso();
    void setDistanceToTorso(float distanceToTorso);
    
    vector<MocapPoint> getRelativePositionToTorso();
    void setRelativePositionToTorso(MocapPoint relativePositionToTorso);

private:
    int historyDepth_;
    int elementId_;
    
    vector<MocapPoint> position_;
    vector<MocapPoint> positionFiltered_;
    vector<MocapPoint> velocity_;
    vector<MocapPoint> acceleration_;
    vector<float> accelerationTrajectory_;
    vector<float> distanceToTorso_;
    vector<MocapPoint> relativePositionToTorso_;
};

#endif
