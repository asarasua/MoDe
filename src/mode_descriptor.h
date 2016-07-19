//
//  mode_descriptor.h
//  example-openni-of093
//
//  Created by Álvaro Sarasúa Berodia on 19/7/16.
//
//

#ifndef mode_descriptor_h
#define mode_descriptor_h

#include "mode_point.h"

namespace MoDe {
    
    enum {
        EXTREME_TYPE_MIN,
        EXTREME_TYPE_MAX
    };
    
    enum {
        MOCAP_X,
        MOCAP_Y,
        MOCAP_Z
    };
    
    class MoDeExtreme {
    public:
        unsigned int axis, joint, feature, extremeType, framesPassed;
        float value;
        MoDeExtreme() : framesPassed(0) {};
    };
    
    template <typename T> class MoDeDescriptor {
    private:
        vector<T> data;
        vector<MoDeExtreme> extrema;
        T sum;
        T ssq;
        T upThreshold, lowThreshold;
        
        bool hasNegative(){
            for (auto d : data) {
                if (d < 0) return true;
            }
            return false;
        }
        
        double toDouble(double value) const {
            return value;
        }
        
        double toDouble(MoDePoint value) const {
            return value.x;
        }
        
        double _sqrt(double value) const {
            return sqrt(value);
        }
        
        MoDePoint _sqrt(MoDePoint point) const {
            return MoDePoint(std::sqrt(point.x), std::sqrt(point.y), std::sqrt(point.z));
        }
        
        double stdev (int cnt, double sum, double ssq) const {
            return sqrt((((double)(cnt))*ssq-pow((double)(sum),2)) / ((double)(cnt)*((double)(cnt)-1)));
        }
        
        MoDePoint stdev(int cnt, MoDePoint sum, MoDePoint ssq) const {
            return MoDePoint(stdev(cnt, sum.x, ssq.x), stdev(cnt, sum.y, ssq.y), stdev(cnt, sum.z, ssq.z)  );
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
                    MoDeExtreme max;
                    max.value = vec[2];
                    max.extremeType = EXTREME_TYPE_MAX;
                    extrema.push_back(max);
                }
                else if (distance(vec.begin(), min_element(vec.begin(), vec.end())) == 2
                         && vec[2] < lth) {
                    MoDeExtreme min;
                    min.value = vec[2];
                    min.extremeType = EXTREME_TYPE_MIN;
                    extrema.push_back(min);
                }
            }
            //3d case
            else {
                MoDePoint uth(upThreshold);
                MoDePoint lth(lowThreshold);
                vector<double> x_vec, y_vec, z_vec;
                for (int i = data.size() - 5; i < data.size(); i++) {
                    MoDePoint d(data[i]);
                    x_vec.push_back(d.x);
                    y_vec.push_back(d.y);
                    z_vec.push_back(d.z);
                }
                
                //x
                if (distance(x_vec.begin(), max_element(x_vec.begin(), x_vec.end())) == 2
                    && x_vec[2] > uth.x) {
                    MoDeExtreme max;
                    max.value = x_vec[2];
                    max.axis = MOCAP_X;
                    max.extremeType = EXTREME_TYPE_MAX;
                    extrema.push_back(max);
                }
                else if (distance(x_vec.begin(), min_element(x_vec.begin(), x_vec.end())) == 2
                         && x_vec[2] < lth.x) {
                    MoDeExtreme min;
                    min.value = x_vec[2];
                    min.axis = MOCAP_X;
                    min.extremeType = EXTREME_TYPE_MIN;
                    extrema.push_back(min);
                }
                //y
                if (distance(y_vec.begin(), max_element(y_vec.begin(), y_vec.end())) == 2
                    && y_vec[2] > uth.y) {
                    MoDeExtreme max;
                    max.value = y_vec[2];
                    max.axis = MOCAP_Y;
                    max.extremeType = EXTREME_TYPE_MAX;
                    extrema.push_back(max);
                }
                else if (distance(y_vec.begin(), min_element(y_vec.begin(), y_vec.end())) == 2
                         && y_vec[2] < lth.y) {
                    MoDeExtreme min;
                    min.value = y_vec[2];
                    min.axis = MOCAP_Y;
                    min.extremeType = EXTREME_TYPE_MIN;
                    extrema.push_back(min);
                }
                //z
                if (distance(z_vec.begin(), max_element(z_vec.begin(), z_vec.end())) == 2
                    && z_vec[2] > uth.z) {
                    MoDeExtreme max;
                    max.value = z_vec[2];
                    max.axis = MOCAP_Z;
                    max.extremeType = EXTREME_TYPE_MAX;
                    extrema.push_back(max);
                }
                else if (distance(z_vec.begin(), min_element(z_vec.begin(), z_vec.end())) == 2
                         && z_vec[2] < lth.z) {
                    MoDeExtreme min;
                    min.value = z_vec[2];
                    min.axis = MOCAP_Z;
                    min.extremeType = EXTREME_TYPE_MIN;
                    extrema.push_back(min);
                }
            }
        }
        
    public:
        MoDeDescriptor(int depth) : sum(0), ssq(0), upThreshold(0), lowThreshold(0)  {
            data.resize(depth);
        }
        ~MoDeDescriptor() {}
        
        void push(T newValue) {
            //delete extrema older than data size
            for (vector<MoDeExtreme>::iterator extreme = extrema.begin(); extreme!=extrema.end();)
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
        vector<MoDeExtreme> getNewExtremes() const {
            vector<MoDeExtreme> newExtrema;
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
                MoDePoint mean(0);
                MoDePoint count(0);
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
                
                MoDePoint crest(0);
                MoDePoint rms(getRms());
                if (count.x) crest.x = ( mean.x / count.x ) / rms.x;
                if (count.y) crest.y = ( mean.y / count.y ) / rms.y;
                if (count.z) crest.z = ( mean.z / count.z ) / rms.z;
                
                return crest;
            }
        }
        
    };
}

#endif /* mode_descriptor_h */
