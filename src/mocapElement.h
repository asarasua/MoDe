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

template <typename T> class MocapDescriptor {
private:
    vector<T> data;
    T sum;
    T ssq;
    T sum_c;
    T ssq_c;
//    int countNoOutliers
    
    MocapPoint _sqrt(double value){
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
    
    bool isOutlier(double value){
        return abs(value - getMean()) > getStdev();
    }
    
    bool isOutlier(MocapPoint point){
        MocapPoint std(getStdev());
        return abs(point.x - getMean().x) > std.x && abs(point.y - getMean().y) > std.y && abs(point.z - getMean().z) > std.z;
    }
    
public:
    MocapDescriptor(int depth) : sum(0), ssq(0), sum_c(0), ssq_c(0)  {
        data.resize(depth);
    }
    ~MocapDescriptor() {}
    
    void push(T newValue) {
        if (data.size() <= data.capacity()){
            T oldestValue = data.front();
            sum -= oldestValue;
            ssq -= oldestValue*oldestValue;
            data.erase(data.begin());
        }
        data.push_back(newValue);
        sum += newValue;
        ssq += newValue*newValue;
        if (!isOutlier(newValue)) {
            sum_c += newValue;
            ssq_c += newValue*newValue;
        } 
        
    }
    double size() {
        return data.size();
    }
    T getMean() {
        return sum/size();
    }
    T getMeanC() {
        return sum_c/size();
    }
    T getStdev() {
        return stdev(size(), sum, ssq);
    }
    T getStdevC() {
        return stdev(size(), sum_c, ssq_c);
    }
    T getRms() {
        return _sqrt(ssq / size());
    }
    
    vector<T> getData() {
        return data;
    }
    void setDepth(int depth){
        data.resize(depth);
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
