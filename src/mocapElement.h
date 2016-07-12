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

#define _stdev(cnt, sum, ssq) sqrt((((double)(cnt))*ssq-pow((double)(sum),2)) / ((double)(cnt)*((double)(cnt)-1)))

#define MOCAP_X 0
#define MOCAP_Y 1
#define MOCAP_Z 2
#define BEAT_TYPE_MIN 0
#define BEAT_TYPE_MAX 1
#define NO_JOINT 999


struct Extreme {
    float value;
    int framesPassed, axis, extremeType;
    Extreme(): framesPassed(0){};
};

template <typename T> class MocapDescriptor {
private:
    vector<T> data;
    vector<Extreme> extrema;
    T sum;
    T ssq;
    T threshold;
    
    //TODO find better way to do this!!
    double toDouble(double value){
        return value;
    }
    
    double toDouble(MocapPoint value){
        return value.x;
    }
    
    double _sqrt(double value){
        return sqrt(value);
    }
    
    MocapPoint _sqrt(MocapPoint point){
        return MocapPoint(std::sqrt(point.x), std::sqrt(point.y), std::sqrt(point.z));
    }
    
    double stdev (int cnt, double sum, double ssq){
        return sqrt((((double)(cnt))*ssq-pow((double)(sum),2)) / ((double)(cnt)*((double)(cnt)-1)));
    }
    
    MocapPoint stdev(int cnt, MocapPoint sum, MocapPoint ssq){
        return MocapPoint(stdev(cnt, sum.x, ssq.x), stdev(cnt, sum.y, ssq.y), stdev(cnt, sum.z, ssq.z)  );
    }
    
    void checkMaxAndMin(){
        //one dimensional case
        if (typeid(data[0]) == typeid(double)) {
            double th = toDouble(threshold); //TODO find better way!
            vector<double> vec;
            for (int i = data.size()-6; i < data.size(); i++) {
                vec.push_back(toDouble(data[i]));
            }
            
            if (distance(vec.begin(), max_element(vec.begin(), vec.end())) == 2
                && vec[2] > th) {
                Extreme max;
                max.value = vec[2];
                max.extremeType = BEAT_TYPE_MAX;
                extrema.push_back(max);
            }
            else if (distance(vec.begin(), min_element(vec.begin(), vec.end())) == 2
                     && vec[2] < -th) { //TODO not quite sure about this
                Extreme min;
                min.value = vec[2];
                min.extremeType = BEAT_TYPE_MIN;
                extrema.push_back(min);
            }
        }
        //3d case
        else {
            MocapPoint th(threshold);
            vector<double> x_vec, y_vec, z_vec;
            //for (vector<MocapPoint>::iterator it = descriptorHistory.begin(); it != descriptorHistory.end(); it++) {
            for (int i = data.size()-6; i < data.size(); i++) {
                MocapPoint d(data[i]);
                x_vec.push_back(d.x);
                y_vec.push_back(d.y);
                z_vec.push_back(d.z);
            }
            
            //x
            if (distance(x_vec.begin(), max_element(x_vec.begin(), x_vec.end())) == 2
                && x_vec[2] > th.x) {
                Extreme max;
                max.value = x_vec[2];
                max.axis = MOCAP_X;
                max.extremeType = BEAT_TYPE_MAX;
                extrema.push_back(max);
            }
            else if (distance(x_vec.begin(), min_element(x_vec.begin(), x_vec.end())) == 2
                     && x_vec[2] < th.x) {
                Extreme min;
                min.value = x_vec[2];
                min.axis = MOCAP_X;
                min.extremeType = BEAT_TYPE_MIN;
                extrema.push_back(min);
            }
            //y
            if (distance(y_vec.begin(), max_element(y_vec.begin(), y_vec.end())) == 2
                && y_vec[2] > th.y) {
                Extreme max;
                max.value = y_vec[2];
                max.axis = MOCAP_Y;
                max.extremeType = BEAT_TYPE_MAX;
                extrema.push_back(max);
            }
            else if (distance(y_vec.begin(), min_element(y_vec.begin(), y_vec.end())) == 2
                     && y_vec[2] < th.y) {
                Extreme min;
                min.value = y_vec[2];
                min.axis = MOCAP_Y;
                min.extremeType = BEAT_TYPE_MIN;
                extrema.push_back(min);
            }
            //z
            if (distance(z_vec.begin(), max_element(z_vec.begin(), z_vec.end())) == 2
                && z_vec[2] > th.z) {
                Extreme max;
                max.value = z_vec[2];
                max.axis = MOCAP_Z;
                max.extremeType = BEAT_TYPE_MAX;
                extrema.push_back(max);
            }
            else if (distance(z_vec.begin(), min_element(z_vec.begin(), z_vec.end())) == 2
                     && z_vec[2] < th.z) {
                Extreme min;
                min.value = z_vec[2];
                min.axis = MOCAP_Z;
                min.extremeType = BEAT_TYPE_MIN;
                extrema.push_back(min);
            }
        }
    }
    
public:
    MocapDescriptor(int depth) : sum(0), ssq(0), threshold(0)  {
        data.resize(depth);
    }
    ~MocapDescriptor() {}
    
    void push(T newValue) {
        
        
        for (vector<Extreme>::iterator extreme = extrema.begin(); extreme!=extrema.end();)
        {
            if(extreme->framesPassed >= data.size())
                extreme = extrema.erase(extreme);
            else
                ++extreme;
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
        
        threshold = getRms() * (1.0 + 3.0 * ( 1.0 / ( getStdev() + 1.0 )) );
        
        checkMaxAndMin();
        
    }
    double size() {
        return data.size();
    }
    T getMean() {
        return sum/size();
    }
    T getStdev() {
        return stdev(size(), sum, ssq);
    }
    T getRms() {
        return _sqrt(ssq / size());
    }
    
    vector<T> getData() {
        return data;
    }
    void setDepth(int depth){
        data.reserve(depth);
    }
    
};


class MocapElement{
public:
    MocapElement();
    MocapElement(int elementId, int depth);
    
    unsigned int getElementId();
    void setElementId(int newId);
    
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
