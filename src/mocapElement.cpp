/*
 ofxKinectFeatures
 Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of ofxKinectFeatures, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>
 
 ofxKinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 ofxKinectFeatures is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the ofxKinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
 */
#include "mocapElement.h"

MocapElement::MocapElement() : position(0), positionFiltered(0), velocity(0), acceleration(0), accelerationTrajectory(0), relativePositionToTorso(0) {}

MocapElement::MocapElement(int elementId, int depth) : position(depth), positionFiltered(depth), velocity(depth), acceleration(depth), accelerationTrajectory(depth), relativePositionToTorso(depth){
    elementId_ = elementId;
    historyDepth_ = depth;
}

unsigned int MocapElement::getElementId(){
    return elementId_;
}

void MocapElement::setElementId(int newId){
    elementId_ = newId;
}

