/*
 ofxKinectFeatures
 Copyright © 2014 Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of ofxKinectFeatures, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>
 
 ofxKinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 ofxKinectFeatures is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the ofxKinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
 */

#ifndef openNiFeatures_MocapElement_h
#define openNiFeatures_MocapElement_h

#include "mocapPoint.h"
#include <iostream>


#define MOCAP_X 0
#define MOCAP_Y 1
#define MOCAP_Z 2
#define EXTREME_TYPE_MIN 0
#define EXTREME_TYPE_MAX 1
#define NO_JOINT 999

class MocapExtreme {
public:
    unsigned int axis, joint, feature, extremeType, framesPassed;
    float value;
    MocapExtreme() : framesPassed(0) {};
};


template <typename T> class MocapDescriptor {
private:
    vector<T> data;
    vector<MocapExtreme> extrema;
    T sum;
    T ssq;
    T upThreshold, lowThreshold;
    
    //TODO find better way to do this!!
    bool hasNegative(){
        for (auto d : data) {
            if (d < 0) return true;
        }
        return false;
    }
    
    double toDouble(double value) const {
        return value;
    }
    
    double toDouble(MocapPoint value) const {
        return value.x;
    }
    
    double _sqrt(double value) const {
        return sqrt(value);
    }
    
    MocapPoint _sqrt(MocapPoint point) const {
        return MocapPoint(std::sqrt(point.x), std::sqrt(point.y), std::sqrt(point.z));
    }
    
    double stdev (int cnt, double sum, double ssq) const {
        return sqrt((((double)(cnt))*ssq-pow((double)(sum),2)) / ((double)(cnt)*((double)(cnt)-1)));
    }
    
    MocapPoint stdev(int cnt, MocapPoint sum, MocapPoint ssq) const {
        return MocapPoint(stdev(cnt, sum.x, ssq.x), stdev(cnt, sum.y, ssq.y), stdev(cnt, sum.z, ssq.z)  );
    }
    
    void checkMaxAndMin(){
        //one dimensional case
        if (typeid(data[0]) == typeid(double)) {
            double uth = toDouble(upThreshold); //TODO find better way!
            double lth = toDouble(lowThreshold); //TODO find better way!
            vector<double> vec;
            
            for (int i = data.size() - 5; i < data.size(); i++) {
                vec.push_back(toDouble(data[i]));
            }
            
            if (distance(vec.begin(), max_element(vec.begin(), vec.end())) == 2
                && vec[2] > uth) {
                MocapExtreme max;
                max.value = vec[2];
                max.extremeType = EXTREME_TYPE_MAX;
                extrema.push_back(max);
            }
            else if (distance(vec.begin(), min_element(vec.begin(), vec.end())) == 2
                     && vec[2] < lth) {
                MocapExtreme min;
                min.value = vec[2];
                min.extremeType = EXTREME_TYPE_MIN;
                extrema.push_back(min);
            }
        }
        //3d case
        else {
            MocapPoint uth(upThreshold);
            MocapPoint lth(lowThreshold);
            vector<double> x_vec, y_vec, z_vec;
            for (int i = data.size() - 5; i < data.size(); i++) {
                MocapPoint d(data[i]);
                x_vec.push_back(d.x);
                y_vec.push_back(d.y);
                z_vec.push_back(d.z);
            }
            
            //x
            if (distance(x_vec.begin(), max_element(x_vec.begin(), x_vec.end())) == 2
                && x_vec[2] > uth.x) {
                MocapExtreme max;
                max.value = x_vec[2];
                max.axis = MOCAP_X;
                max.extremeType = EXTREME_TYPE_MAX;
                extrema.push_back(max);
            }
            else if (distance(x_vec.begin(), min_element(x_vec.begin(), x_vec.end())) == 2
                     && x_vec[2] < lth.x) {
                MocapExtreme min;
                min.value = x_vec[2];
                min.axis = MOCAP_X;
                min.extremeType = EXTREME_TYPE_MIN;
                extrema.push_back(min);
            }
            //y
            if (distance(y_vec.begin(), max_element(y_vec.begin(), y_vec.end())) == 2
                && y_vec[2] > uth.y) {
                MocapExtreme max;
                max.value = y_vec[2];
                max.axis = MOCAP_Y;
                max.extremeType = EXTREME_TYPE_MAX;
                extrema.push_back(max);
            }
            else if (distance(y_vec.begin(), min_element(y_vec.begin(), y_vec.end())) == 2
                     && y_vec[2] < lth.y) {
                MocapExtreme min;
                min.value = y_vec[2];
                min.axis = MOCAP_Y;
                min.extremeType = EXTREME_TYPE_MIN;
                extrema.push_back(min);
            }
            //z
            if (distance(z_vec.begin(), max_element(z_vec.begin(), z_vec.end())) == 2
                && z_vec[2] > uth.z) {
                MocapExtreme max;
                max.value = z_vec[2];
                max.axis = MOCAP_Z;
                max.extremeType = EXTREME_TYPE_MAX;
                extrema.push_back(max);
            }
            else if (distance(z_vec.begin(), min_element(z_vec.begin(), z_vec.end())) == 2
                     && z_vec[2] < lth.z) {
                MocapExtreme min;
                min.value = z_vec[2];
                min.axis = MOCAP_Z;
                min.extremeType = EXTREME_TYPE_MIN;
                extrema.push_back(min);
            }
        }
    }
    
public:
    MocapDescriptor(int depth) : sum(0), ssq(0), upThreshold(0), lowThreshold(0)  {
        data.resize(depth);
    }
    ~MocapDescriptor() {}
    
    void push(T newValue) {
        //delete extrema older than data size
        for (vector<MocapExtreme>::iterator extreme = extrema.begin(); extreme!=extrema.end();)
        {
            if(extreme->framesPassed >= data.size())
                extreme = extrema.erase(extreme);
            else {
                (*extreme).framesPassed++;
                ++extreme;
            }
        }
        
        if (data.size() == data.capacity()){
            T oldestValue = data.front();
            sum -= oldestValue;
            ssq -= oldestValue * oldestValue;
            data.erase(data.begin());
        }
        data.push_back(newValue);
        sum += newValue;
        ssq += newValue*newValue;
        
        upThreshold = getRms() * (1.0 + 3.0 * ( 1.0 / ( getStdev() + 1.0 )) );
        if (hasNegative()) {
            lowThreshold = -upThreshold;
        } else {
            lowThreshold = getRms() * (1.0 - 3.0 * ( 1.0 / ( getStdev() + 1.0 )) );
        }
        
        checkMaxAndMin();
        
    }
    double size() const {
        return data.size();
    }
    T getMean() const {
        return sum/size();
    }
    T getStdev() const {
        return stdev(size(), sum, ssq);
    }
    T getRms() const  {
        return _sqrt(ssq / size());
    }
    double getMagnitude() const {
        if (typeid(data[0]) == typeid(double)) {
            return toDouble(getCurrent());
        } else {
            return getCurrent().length();
        }
    }
    vector<T> getData() const {
        return data;
    }
    T getCurrent() const {
        return data[data.size()-1];
    }
    void setDepth(int depth){
        data.reserve(depth);
    }
    vector<MocapExtreme> getNewExtremes() const {
        vector<MocapExtreme> newExtrema;
        for (auto extreme : extrema){
            if (extreme.framesPassed == 0)
                newExtrema.push_back(extreme);
        }
        return newExtrema;
    }
    T getCrest() const {
        if (typeid(data[0]) == typeid(double)) {
            return 0;
        } else {
            MocapPoint mean(0);
            MocapPoint count(0);
            for (auto extreme: extrema) {
                if (extreme.axis == MOCAP_X) {
                    mean.x += extreme.value;
                    count.x ++;
                } else if (extreme.axis == MOCAP_Y) {
                    mean.y += extreme.value;
                    count.y ++;
                } else if (extreme.axis == MOCAP_Z) {
                    mean.z += extreme.value;
                    count.z ++;
                }
            }
            
            MocapPoint crest(0);
            MocapPoint rms(getRms());
            if (count.x) crest.x = ( mean.x / count.x ) / rms.x;
            if (count.y) crest.y = ( mean.y / count.y ) / rms.y;
            if (count.z) crest.z = ( mean.z / count.z ) / rms.z;
            
            return crest;
        }
    }
    
};


class MocapElement{
public:
    MocapElement() : position(0), positionFiltered(0), velocity(0), acceleration(0), accelerationTrajectory(0), relativePositionToTorso(0) {};
    MocapElement(int elementId, int depth) : position(depth), positionFiltered(depth), velocity(depth), acceleration(depth), accelerationTrajectory(depth), relativePositionToTorso(depth){
        elementId_ = elementId;
        historyDepth_ = depth;
    }
    
    unsigned int getElementId(){
        return elementId_;
    }
    
    void setElementId(int newId){
        elementId_ = newId;
    }
    
    MocapDescriptor<MocapPoint> position;
    MocapDescriptor<MocapPoint> positionFiltered;
    MocapDescriptor<MocapPoint> velocity;
    MocapDescriptor<MocapPoint> acceleration;    
    MocapDescriptor<float> accelerationTrajectory;
    MocapDescriptor<MocapPoint> relativePositionToTorso;

private:
    int historyDepth_;
    int elementId_;    
};

#endif
