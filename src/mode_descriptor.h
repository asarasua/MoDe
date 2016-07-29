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
        AXIS_X,
        AXIS_Y,
        AXIS_Z,
		AXIS_NONE
    };
    
    class MoDeExtreme {
    public:
        unsigned int axis, joint, feature, extremeType, framesPassed;
        double value;
        MoDeExtreme() : framesPassed(0) {}; 
    };
    
    template <typename T> class MoDeDescriptor {
	private:
		vector<MoDeExtreme> outliers;
		double outlierInfluence;
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

		double crest(double rms) const {
			double count(0);
			double mean(0);
			for (auto extreme : extrema) {
				if (extreme.axis == AXIS_NONE) {
					mean += extreme.value;
					count++;
				}
			}

			double crest(0);
			if (count) crest = (mean / count) / rms;
			return crest;
		}

		MoDePoint crest(MoDePoint rms) const {
			MoDePoint mean(0);
			MoDePoint count(0);
			for (auto extreme : extrema) {
				if (extreme.axis == AXIS_X) {
					mean.x += extreme.value;
					count.x++;
				}
				else if (extreme.axis == AXIS_Y && extreme.extremeType == EXTREME_TYPE_MAX) {
					mean.y += extreme.value;
					count.y++;
				}
				else if (extreme.axis == AXIS_Z) {
					mean.z += extreme.value;
					count.z++;
				}
			}

			MoDePoint crest(0);
			if (count.x) crest.x = (mean.x / count.x) / rms.x;
			if (count.y) crest.y = (mean.y / count.y) / rms.y;
			if (count.z) crest.z = (mean.z / count.z) / rms.z;

			return crest;
		}
        
        double _sqrt(double value) const {
            return sqrt(value);
        }
        
        MoDePoint _sqrt(MoDePoint point) const {
            return MoDePoint(std::sqrt(point.x), std::sqrt(point.y), std::sqrt(point.z));
        }

		double stdev(int cnt, double sum, double ssq) const {
			double st = sqrt((((double)(cnt))*ssq - pow((double)(sum), 2)) / ((double)(cnt)*((double)(cnt)-1)));
			if (!isnan(st) && !isinf(st))
				return st;
			else return 0;
		}

		MoDePoint stdev(int cnt, MoDePoint sum, MoDePoint ssq) const {
			return MoDePoint(stdev(cnt, sum.x, ssq.x), stdev(cnt, sum.y, ssq.y), stdev(cnt, sum.z, ssq.z));
		}

		double mean_c(double sum) const {
			double sum_c(sum);
			for (auto o : outliers) {
				sum_c += o.value * (outlierInfluence - 1.0);
			}
			double mean_c = sum_c / (size() - outliers.size());
			return mean_c;
		}

		MoDePoint mean_c(MoDePoint sum) const {
			MoDePoint sum_c(sum);
			MoDePoint num_o(0);
			for (auto o : outliers) {
				if (o.axis == AXIS_X) {
					sum_c.x += o.value * (outlierInfluence - 1.0);
					num_o.x++;
				}
				else if (o.axis == AXIS_Y) {
					sum_c.y += o.value * (outlierInfluence - 1.0);
					num_o.y++;
				}
				else if (o.axis == AXIS_Z) {
					sum_c.z += o.value * (outlierInfluence - 1.0);
					num_o.z++;
				}
			}

			MoDePoint mean_c = sum_c / (size() - num_o);
			return mean_c;
		}

		double stdev_c(double sum, double ssq) const {
			if (outliers.size() == size())
			{
				return stdev(size(), sum, ssq);
			}
			else {
				double sum_c = sum;
				double ssq_c = ssq;
				for (auto o : outliers) {
					sum_c += o.value * (outlierInfluence - 1.0);
					ssq_c -= o.value * o.value;
					ssq_c += o.value * o.value * outlierInfluence * outlierInfluence;
				}

				return stdev(size() - extrema.size(), sum_c, ssq_c);
			}
		}

		MoDePoint stdev_c(MoDePoint sum, MoDePoint ssq) const {
			MoDePoint sum_c(sum);
			MoDePoint ssq_c(ssq);
			MoDePoint num_o(0);
			for (auto o : outliers) {
				if (o.axis == AXIS_X)
				{
					sum_c.x -= o.value;
					ssq_c.x -= o.value * o.value;
					ssq_c.x += o.value * o.value * outlierInfluence * outlierInfluence;
					num_o.x++;
				}
				else if (o.axis == AXIS_Y)
				{
					sum_c.y -= o.value;
					ssq_c.y -= o.value * o.value;
					ssq_c.y += o.value * o.value * outlierInfluence * outlierInfluence;
					num_o.y++;
				}
				else if (o.axis == AXIS_Z)
				{
					sum_c.z -= o.value;
					ssq_c.z -= o.value * o.value;
					ssq_c.z += o.value * o.value * outlierInfluence * outlierInfluence;
					num_o.z++;
				}
			}

			MoDePoint std;
			if (num_o.x != size())
				std.x = stdev(size() - num_o.x, sum_c.x, ssq_c.x);
			else
				std.x = stdev(size(), sum.x, ssq.x);
			if (num_o.y != size())
				std.y = stdev(size() - num_o.y, sum_c.y, ssq_c.y);
			else
				std.y = stdev(size(), sum.y, ssq.y);
			if (num_o.z != size())
				std.z = stdev(size() - num_o.z, sum_c.z, ssq_c.z);
			else
				std.z = stdev(size(), sum.z, ssq.z);

			return std;
		}

		void checkOutlier(double value) {
			if (value > upThreshold || value < lowThreshold)
			{
				MoDeExtreme outlier;
				outlier.value = value;
				outlier.axis = AXIS_NONE;
				outliers.push_back(outlier);
			}
		}

		void checkOutlier(MoDePoint value) {
			MoDePoint uth(upThreshold);
			MoDePoint lth(lowThreshold);
			//x
			if (value.x > uth.x || value.x < lth.x) {
				MoDeExtreme outlier;
				outlier.value = value.x;
				outlier.axis = AXIS_X;
				outliers.push_back(outlier);
			}
			//y
			if (value.y > uth.y || value.y < lth.y) {
				MoDeExtreme outlier;
				outlier.value = value.y;
				outlier.axis = AXIS_Y;
				outliers.push_back(outlier);
			}
			//z
			if (value.z > uth.z || value.z < lth.z) {
				MoDeExtreme outlier;
				outlier.value = value.z;
				outlier.axis = AXIS_Z;
				outliers.push_back(outlier);
			}
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
                
                if (vec[2] > uth && distance(vec.begin(), max_element(vec.begin(), vec.end())) == 2) {
                    MoDeExtreme max;
                    max.value = vec[2];
                    max.extremeType = EXTREME_TYPE_MAX;
					max.axis = AXIS_NONE;
					max.framesPassed = 2;
                    extrema.push_back(max);
                }
                else if (vec[2] < lth && distance(vec.begin(), min_element(vec.begin(), vec.end())) == 2) {
                    MoDeExtreme min;
                    min.value = vec[2];
                    min.extremeType = EXTREME_TYPE_MIN;
					min.axis = AXIS_NONE;
					min.framesPassed = 2;
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
                //TODO
                //x
                if (x_vec[2] > uth.x && distance(x_vec.begin(), max_element(x_vec.begin(), x_vec.end())) == 2) {
                    MoDeExtreme max;
                    max.value = x_vec[2];
                    max.axis = AXIS_X;
                    max.extremeType = EXTREME_TYPE_MAX;
					max.framesPassed = 2;
                    extrema.push_back(max);
                }
                else if (x_vec[2] < lth.x && distance(x_vec.begin(), min_element(x_vec.begin(), x_vec.end())) == 2) {
                    MoDeExtreme min;
                    min.value = x_vec[2];
                    min.axis = AXIS_X;
                    min.extremeType = EXTREME_TYPE_MIN;
					min.framesPassed = 2;
					extrema.push_back(min);
                }
                //y
                if (y_vec[2] > uth.y && distance(y_vec.begin(), max_element(y_vec.begin(), y_vec.end())) == 2) {
                    MoDeExtreme max;
                    max.value = y_vec[2];
                    max.axis = AXIS_Y;
                    max.extremeType = EXTREME_TYPE_MAX;
					max.framesPassed = 2;
                    extrema.push_back(max);
                }
                else if (y_vec[2] < lth.y && distance(y_vec.begin(), min_element(y_vec.begin(), y_vec.end())) == 2) {
                    MoDeExtreme min;
                    min.value = y_vec[2];
                    min.axis = AXIS_Y;
                    min.extremeType = EXTREME_TYPE_MIN;
					min.framesPassed = 2;
                    extrema.push_back(min);
                }
                //z
                if (z_vec[2] > uth.z && distance(z_vec.begin(), max_element(z_vec.begin(), z_vec.end())) == 2) {
                    MoDeExtreme max;
                    max.value = z_vec[2];
                    max.axis = AXIS_Z;
                    max.extremeType = EXTREME_TYPE_MAX;
					max.framesPassed = 2;
                    extrema.push_back(max);
                }
                else if (z_vec[2] < distance(z_vec.begin(), min_element(z_vec.begin(), z_vec.end())) == 2) {
                    MoDeExtreme min;
                    min.value = z_vec[2];
                    min.axis = AXIS_Z;
                    min.extremeType = EXTREME_TYPE_MIN;
					min.framesPassed = 2;
                    extrema.push_back(min);
                }
            }
        }
        
    public:
        MoDeDescriptor(int depth) : sum(0), ssq(0), upThreshold(0), lowThreshold(0), outlierInfluence(0.01)  {
            data.resize(depth);
        }
        ~MoDeDescriptor() {}
        
        void push(T newValue) {
            //delete extrema older than data size
            for (vector<MoDeExtreme>::iterator extreme = extrema.begin(); extreme!=extrema.end();)
            {
				(*extreme).framesPassed++;
                if(extreme->framesPassed == data.size())
                    extreme = extrema.erase(extreme);
                else {
                    ++extreme;
                }
            }
			//delete outliers older than data size
			for (vector<MoDeExtreme>::iterator outlier = outliers.begin(); outlier != outliers.end();)
			{
				(*outlier).framesPassed++;
				if (outlier->framesPassed == data.size())
					outlier = outliers.erase(outlier);
				else {
					++outlier;
				}
			}
            
            if (data.size() == data.capacity()){
                T oldestValue = data.front();
                sum -= oldestValue;
                ssq -= oldestValue * oldestValue;
                data.erase(data.begin());
            }
			checkOutlier(newValue);

            data.push_back(newValue);
            sum += newValue;
            ssq += newValue*newValue;            

			T factor = 1 / (1 + 10 * getStdev_C());

			//upThreshold = getMean_C() + 3.0 * getStdev_C();
			//upThreshold = getMean_C() + ( 5.0 * factor ) * getStdev_C();
			//lowThreshold = getMean_C() - 3.0 * getStdev_C();
			
            checkMaxAndMin();
            
        }
        double size() const {
            return data.size();
        }
        T getMean() const {
            return sum/size();
        }
		T getMean_C() const {
			return mean_c(sum);
		}

        T getStdev() const {
            return stdev(size(), sum, ssq);
        }

		T getStdev_C() const {
			return stdev_c(sum, ssq);
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
                if (extreme.framesPassed == 2)
                    newExtrema.push_back(extreme);
            }
            return newExtrema;
        }
        T getCrest() const {
			return crest(getRms());
        }

		T getUpperThreshold() {
			return upThreshold;
		}

		T getLowThreshold() {
			return lowThreshold;
		}

		void setUpperThreshold(T ut) {
			upThreshold = ut;
		}

		void setLowThreshold(T lt) {
			lowThreshold = lt;
		}
        
    };
}

#endif /* mode_descriptor_h */
