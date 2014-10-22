/*
 ofxKinectFeatures
 Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya
 
 This file is part of ofxKinectFeatures, created and maintained by Álvaro Sarasúa <http://alvarosarasua.wordpress.com>
 
 ofxKinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 
 ofxKinectFeatures is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).
 
 You should have received a copy of the GNU Lesser General Public License long within the ofxKinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.
 
 If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu
 */

#include "ofxKinectFeatures.h"

//Real-time filters for MoCap by Skogstad et al ( http://www.uio.no/english/research/groups/fourms/projects/sma/subprojects/mocapfilters/ )

float lpf_soft_a[] = {1,-1.2982434912,1.4634092217,-0.7106501488,0.2028836637};
float lpf_soft_b[] = {0.1851439645,0.1383283833,0.1746892243,0.1046627716,0.0464383730};
float lpf_med_a[] = {1,-1.7462227354,1.7354077932,-0.8232679111,0.1793463694};
float lpf_med_b[] = {0.1526249789,0.0333481282,0.0777551903,0.0667145281,0.0138945068};
float lpf_hard_a[] = {1,-1.9185418203,1.5929378702,-0.5939699187,0.0814687111};
float lpf_hard_b[] = {0.1400982208,-0.0343775491,0.0454003083,0.0099732061,0.0008485135};

float lpd1_soft_a[] = {1,-0.2919477037,0.5104653639,-0.01557831719,0.000283848732};
float lpd1_soft_b[] = {0.2712475020,0.1323672597,-0.0487267360,-0.1783422292,-0.1765457966};
float lpd1_med_a[] = {1,-0.9870779094,0.7774863652,-0.2206843188,0.02813441289};
float lpd1_med_b[] = {0.1973679432,-0.0056567353,-0.0321850947,-0.1099445540,-0.0495815592};
float lpd1_hard_a[] = {1,-2.057494776,1.858705877,-0.801785135,0.131076358};
float lpd1_hard_b[] = {-0.1543174259,0.1742393427,-0.0178886989,-0.0022975713,0.0002643535};

float lpd2_soft_a[] = {1,-0.8274946715,0.8110775672,-0.3530877871,0.06598917583};
float lpd2_soft_b[] = {0.1099156485,-0.1289124440,-0.0372667405,0.0216082189,0.0346553170};
float lpd2_med_a[] = {1,-1.571029458,1.459212744,-0.7173743414,0.1488005975};
float lpd2_med_b[] = {-0.0795571277,0.1390709784,-0.0479192600,-0.0031459045,-0.0084486862};
float lpd2_hard_a[] = {1,-1.628286742,1.418759018,-0.6223424612,0.1085280231};
float lpd2_hard_b[] = {-0.0738989849,0.1351624829,-0.0512998379,-0.0072918334,-0.0026718267};

//____________________________________________________ofxKinectFeatures

ofxKinectFeatures::ofxKinectFeatures(){
    newValues_ = false;
}

ofxKinectSkeleton* ofxKinectFeatures::getSkeleton(int skeletonId){
    vector<ofxKinectSkeleton>::iterator it = find_if(skeletons_.begin(), skeletons_.end(), MatchId(skeletonId));
    if (it != skeletons_.end()){
        return &(*it);
    } else {
        return false;
    }
}

bool ofxKinectFeatures::isNewDataAvailable(){
    return newValues_;
}