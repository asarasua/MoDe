/*
ofxKinectFeatures
Copyright © 2014  Music Technology Group - Universitat Pompeu Fabra / Escola Superior de Música de Catalunya

This file is part of ofxKinectFeatures, created and maintained by Álvaro Sarasúa <http://www.alvarosarasua.com>

This particular class is inspired and partially copied from openFrameworks ofVec3f (https://github.com/openframeworks/openFrameworks/blob/master/libs/openFrameworks/math/MocapPoint.h)

ofxKinectFeatures is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License (LGPL v3) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

ofxKinectFeatures is distributed in the hope that it will be useful, but WITHOUT  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License (LGPL v3).

You should have received a copy of the GNU Lesser General Public License long within the ofxKinectFeatures SW package.  If not, see <http://www.gnu.org/licenses/>.

If you are willing to get a (non FOSS) commercial license, please contact us at mtg@upf.edu

*/

using namespace std;

#pragma once

#include <cmath>
#include <vector>

class MocapPoint {
public:
	/// \cond INTERNAL
	static const int DIM = 3;
	/// \endcond

	/// \brief Stores the `X` component of the mocap point.
	float x;

	/// \brief Stores the `Y` component of the mocap point.
	float y;

	/// \brief Stores the `Z` component of the mocap point.
	float z;

	
	MocapPoint();

	/// \brief Construt a 3D vector with `x`, `y` and `z` specified
	MocapPoint(float x, float y, float z = 0);

	/// \brief Construct a 3D from a float vector
	MocapPoint(vector<float> floatVector);


	/// \brief Set 'x', 'y' and 'z' components of this vector with just one function call.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1;
	/// v1.set(40, 20, 70);
	/// ~~~~
	void set(float x, float y, float z);


	/// \brief Setting the values by using other 3 dimension vector MocapPoint.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1;
	/// MocapPoint v2;
	/// v1.set(40, 20, 70);
	/// v2.set(v1);
	/// ~~~~
	void set(const MocapPoint& vec);

	/// \}

	//---------------------
	/// \name Comparison 
	/// \{

	/// \brief Check for equality between two MocapPoint
	///
	/// Returns 'true' if each component is the same as the corresponding component in
	/// 'vec', ie if 'x == vec.x' and 'y == vec.y' and 'z == vec.z'; otherwise returns
	/// 'false'. But you should probably be using ['match'](#match) instead.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(40, 20, 10); 
	/// MocapPoint v2(50, 30, 10); 
	/// MocapPoint v3(40, 20, 10); 
	/// // ( v1 == v2 ) is false
	/// // ( v1 == v3 ) is true
	/// ~~~~
	bool operator==(const MocapPoint& vec) const;

	/// \brief Returns 'true' if any component is different to its corresponding component in
	/// 'vec', ie if 'x != vec.x' or 'y != vec.y' or 'z != vec.z'; otherwise returns
	/// 'false'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(40, 20, 10); 
	/// MocapPoint v2(50, 20, 40); 
	/// MocapPoint v3(40, 20, 10); 
	/// // ( v1 != v2 ) is true
	/// // ( v1 != v3 ) is false
	/// ~~~~
	bool operator!=(const MocapPoint& vec) const;

	/// \brief Let you check if two vectors are similar given a tolerance threshold
	/// 'tolerance' (default = 0.0001).	
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 70);
	/// MocapPoint v2 = MocapPoint(40.01, 19.999, 70.05);
	/// // v1.match(v2, 0.1) is true
	/// // v1.match(v2, 0.01) is false (because (70.5-70) > 0.01)
	/// ~~~~
	/// 
	bool match(const MocapPoint& vec, float tolerance = 0.0001f) const;

	//---------------------
	/// \name Operators
	/// \{

	/// Super easy vector addition. Returns a new vector
	/// ('x'+'vec.x','y'+'vec.y','z'+'vec.z').
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 10); 
	/// MocapPoint v2 = MocapPoint(25, 50, 10);
	/// MocapPoint v3 = v1 + v2; // v3 is (65, 70, 20)
	/// ~~~~
	MocapPoint  operator+(const MocapPoint& pnt) const;

	/// Returns a new vector with a float value 'f' added to 'x', 'y' and 'z'
	/// members.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// MocapPoint v2 = v1 + 10; // (12, 15, 11)
	/// ~~~~

	MocapPoint  operator+(const float f) const;

	/// Super easy addition assignment. Adds 'vec.x' to 'x', adds 'vec.y' to 'y' and
	/// adds 'vec.z' to 'z'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 10); 
	/// MocapPoint v2 = MocapPoint(25, 50, 10);
	/// v1 += v2; // v1 is (65, 70, 20)
	/// ~~~~
	MocapPoint& operator+=(const MocapPoint& pnt);

	/// Adds a float value 'f' to 'x', 'y' and 'z' members.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// v1 += 10; // (12, 15, 11)
	/// ~~~~
	MocapPoint& operator+=(const float f);

	/// Super easy vector subtraction. Returns a new vector
	/// ('x'-'vec.x','y'-'vec.y','z'-'vec.z').	
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 10); 
	/// MocapPoint v2 = MocapPoint(25, 50, 10);
	/// MocapPoint v3 = v1 - v2; // v3 is (15, -30, 0)
	/// ~~~~
	MocapPoint  operator-(const MocapPoint& vec) const;



	/// Returns a new vector with a float value 'f' subtracted from 'x', 'y' and 'z'
	/// members.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// MocapPoint v2 = v1 - 10; // (-8, -5, -9)
	/// ~~~~
	MocapPoint  operator-(const float f) const;

	/// Returns a new 'MocapPoint' that is the inverted version (mirrored in X, Y and Z)
	/// of this vector.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// MocapPoint v2 = -v1; // (-2, -5, -1)
	/// ~~~~
	/// 
	MocapPoint  operator-() const;

	/// Super easy subtraction assignment. Subtracts 'vec.x' from 'x', subtracts
	/// 'vec.y' from 'y' and subtracts 'vec.z' from 'z'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 10); 
	/// MocapPoint v2 = MocapPoint(25, 50, 10);
	/// v1 -= v2; // v1 is (15, -30, 0)
	/// ~~~~    
	MocapPoint& operator-=(const MocapPoint& vec);

	/// Subtract a float value 'f' from 'x', 'y', and 'z' members.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// v1 -= 10; // (-8, -5, -9)
	/// ~~~~
	MocapPoint& operator-=(const float f);

	/// Returns a new vector ('x'*'vec.x','y'*'vec.y','z'*'vec.z').
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 10); 
	/// MocapPoint v2 = MocapPoint(2, 4, 10);
	/// MocapPoint v3 = v1 * v2; // (80, 80, 100)
	/// ~~~~
	/// 
	/// Useful for scaling a 3D point by a non-uniform scale.
	/// 
	MocapPoint  operator*(const MocapPoint& vec) const;

	/// Return a new 'MocapPoint' that is this vector scaled by multiplying 'x', 'y', 'z'
	/// members by 'f'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// MocapPoint v2 = v1 * 4; // (8, 20, 4)
	/// ~~~~
	MocapPoint  operator*(const float f) const;

	/// Multiplies 'x' by 'vec.x', and multiplies 'y' by 'vec.y', and multiplies 'z'
	/// by 'vec.z'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 10); 
	/// MocapPoint v2 = MocapPoint(2, 4, 10);
	/// v1 *= v2; // v1 is now (80, 80, 100)
	/// ~~~~
	/// 
	/// Useful for scaling a 3D point by a non-uniform scale.
	MocapPoint& operator*=(const MocapPoint& vec);

	/// Scale this vector by multiplying 'x', 'y' and 'z' members by 'f'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// v1 *= 4; // (8, 20, 4)
	/// ~~~~
	MocapPoint& operator*=(const float f);

	/// Returns a new vector ('x'/'vec.x','y'/'vec.y','z'/'vec.z').
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 10); 
	/// MocapPoint v2 = MocapPoint(2, 4, 10);
	/// MocapPoint v3 = v1 / v2; // (20, 5, 1)
	/// ~~~~
	/// 
	/// Useful for scaling a 3D point by a non-uniform scale.
	MocapPoint  operator/(const MocapPoint& vec) const;

	/// Return a new 'MocapPoint' that is this vector scaled by dividing 'x', 'y'
	/// and 'z' members by 'f'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// MocapPoint v2 = v1 / 4; // (0.5, 1.25, 0.25)
	/// ~~~~
	MocapPoint  operator/(const float f) const;

	/// Divides 'x' by 'vec.x', divides 'y' by 'vec.y', and divides 'z' by 'vec.z'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1 = MocapPoint(40, 20, 10); 
	/// MocapPoint v2 = MocapPoint(2, 4, 10);
	/// v1 *= v2; // v1 is now (20, 5, 1)
	/// ~~~~
	/// 
	/// Useful for scaling a 3D point by a non-uniform scale.
	MocapPoint& operator/=(const MocapPoint& vec);

	/// Scale this vector by dividing 'x', 'y' and 'z' members by 'f'.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v1(2, 5, 1);
	/// v1 /= 4; // (0.5, 1.25, 0.25)
	/// ~~~~
	MocapPoint& operator/=(const float f);


	//---------------------
	/// \name Distance
	/// \{


	/// \brief Treats both this vector and 'pnt' as points in 3D space, and
	/// calculates and returns the distance between them.	
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint p1(3, 4, 2);
	/// MocapPoint p2(6, 8, 5);
	/// float distance = p1.distance( p2 ); // distance is 5.8310
	/// ~~~~
	/// 	
	/// 'distance' involves a square root calculation, which is one of the
	/// slowest things you can do in programming. If you don't need an exact
	/// number but rather just a rough idea of distance (for example when
	/// finding the shortest distance of a bunch of points to a reference
	/// point, where it doesn't matter exactly what the distances are, you
	/// just want the shortest), you can use squareDistance() instead.
	float distance(const MocapPoint& pnt) const;

	/// \brief Treats both this vector and 'pnt' as points in 3D space, and calculates and
	/// returns the squared distance between them.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint p1(3, 4, 2);
	/// MocapPoint p2(6, 8, 5);
	/// float distance = p1.distance( p2 ); // distance is 5.8310
	/// ~~~~
	/// 
	/// Use as a much faster alternative to distance() if you don't need
	/// to know an exact number but rather just a rough idea of distance (for example
	/// when finding the shortest distance of a bunch of points to a reference point,
	/// where it doesn't matter exactly what the distances are, you just want the
	/// shortest). It avoids the square root calculation that is ordinarily required
	/// to calculate a length.
	/// 
	float squareDistance(const MocapPoint& pnt) const;

	//---------------------
	/// \name Measurement
	/// \{


	/// Return the length (magnitude) of this vector.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v(3, 4, 1);
	/// float len = v.length(); // len is 5.0990
	/// ~~~~
	/// 
	/// `length' involves a square root calculation, which is one of the
	/// slowest things you can do in programming. If you don't need an exact
	/// number but rather just a rough idea of a length (for example when
	/// finding the shortest distance of a bunch of points to a reference
	/// point, where it doesn't matter exactly what the lengths are, you just
	/// want the shortest), you can use lengthSquared() instead.
	///    
	float length() const;

	/// \brief Return the squared length (squared magnitude) of this vector.
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint v(3, 4, 1);
	/// float len = v.length(); // len is 5.0990
	/// ~~~~
	/// 
	/// Use as a much faster alternative to length() if you don't need
	/// to know an accurate length but rather just a rough idea of a length (for
	/// example when finding the shortest distance of a bunch of points to a
	/// reference point, where it doesn't matter exactly what the lengths are, you
	/// just want the shortest). It avoids the square root calculation that is
	/// ordinarily required to calculate a length.
	float lengthSquared() const;


	/// \brief Calculate and return the dot product of this vector with 'vec'.
	/// 
	/// *Dot product* (less commonly known as *Euclidean inner product*)
	/// expresses the angular relationship between two vectors. In other
	/// words it is a measure of how *parallel* two vectors are. If they are
	/// completely perpendicular the dot product is 0; if they are completely
	/// parallel their dot product is either 1 if they are pointing in the
	/// same direction, or -1 if they are pointing in opposite directions.	
	/// 
	/// ![DOT](math/dotproduct.png)
	/// Image courtesy of Wikipedia
	/// 
	/// ~~~~{.cpp}
	/// MocapPoint a1(1, 0, 0);
	/// MocapPoint b1(0, 0, 1); // 90 degree angle to a1
	/// dot = a1.dot(b1); // dot is 0, ie cos(90)
	/// 
	/// MocapPoint a2(1, 0, 0); 
	/// MocapPoint b2(1, 1, 0); // 45 degree angle to a2
	/// b2.normalize(); // vectors should to be unit vectors (normalized)
	/// float dot = a2.dot(b2); // dot is 0.707, ie cos(45)
	/// 
	/// MocapPoint a3(0, 1, 0);
	/// MocapPoint b3(0, -1, 0); // 180 degree angle to a3
	/// dot = a3.dot(b3); // dot is -1, ie cos(180)
	/// ~~~~
	/// 
	float dot(const MocapPoint& vec) const;

};


/// \cond INTERNAL


// Non-Member operators
//
//
MocapPoint operator+(float f, const MocapPoint& vec);
MocapPoint operator-(float f, const MocapPoint& vec);
MocapPoint operator*(float f, const MocapPoint& vec);
MocapPoint operator/(float f, const MocapPoint& vec);


/////////////////
// Implementation
/////////////////


inline MocapPoint::MocapPoint() : x(0), y(0), z(0) {}
inline MocapPoint::MocapPoint(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
inline MocapPoint::MocapPoint(vector<float> floatVector) : x(floatVector[0]), y(floatVector[1]), z(floatVector[2]) {}


inline void MocapPoint::set(float _x, float _y, float _z) {
	x = _x;
	y = _y;
	z = _z;
}

inline void MocapPoint::set(const MocapPoint& vec) {
	x = vec.x;
	y = vec.y;
	z = vec.z;
}


// Check similarity/equality.
//
//
inline bool MocapPoint::operator==(const MocapPoint& vec) const {
	return (x == vec.x) && (y == vec.y) && (z == vec.z);
}

inline bool MocapPoint::operator!=(const MocapPoint& vec) const {
	return (x != vec.x) || (y != vec.y) || (z != vec.z);
}

inline bool MocapPoint::match(const MocapPoint& vec, float tolerance) const {
	return (fabs(x - vec.x) < tolerance)
		&& (fabs(y - vec.y) < tolerance)
		&& (fabs(z - vec.z) < tolerance);
}


inline MocapPoint MocapPoint::operator+(const MocapPoint& pnt) const {
	return MocapPoint(x + pnt.x, y + pnt.y, z + pnt.z);
}

inline MocapPoint& MocapPoint::operator+=(const MocapPoint& pnt) {
	x += pnt.x;
	y += pnt.y;
	z += pnt.z;
	return *this;
}

inline MocapPoint MocapPoint::operator-(const MocapPoint& vec) const {
	return MocapPoint(x - vec.x, y - vec.y, z - vec.z);
}

inline MocapPoint& MocapPoint::operator-=(const MocapPoint& vec) {
	x -= vec.x;
	y -= vec.y;
	z -= vec.z;
	return *this;
}

inline MocapPoint MocapPoint::operator*(const MocapPoint& vec) const {
	return MocapPoint(x*vec.x, y*vec.y, z*vec.z);
}

inline MocapPoint& MocapPoint::operator*=(const MocapPoint& vec) {
	x *= vec.x;
	y *= vec.y;
	z *= vec.z;
	return *this;
}

inline MocapPoint MocapPoint::operator/(const MocapPoint& vec) const {
	return MocapPoint(vec.x != 0 ? x / vec.x : x, vec.y != 0 ? y / vec.y : y, vec.z != 0 ? z / vec.z : z);
}

inline MocapPoint& MocapPoint::operator/=(const MocapPoint& vec) {
	vec.x != 0 ? x /= vec.x : x;
	vec.y != 0 ? y /= vec.y : y;
	vec.z != 0 ? z /= vec.z : z;
	return *this;
}

inline MocapPoint MocapPoint::operator-() const {
	return MocapPoint(-x, -y, -z);
}


//operator overloading for float
//
//
//inline void MocapPoint::operator=( const float f){
//	x = f;
//	y = f;
//	z = f;
//}

inline MocapPoint MocapPoint::operator+(const float f) const {
	return MocapPoint(x + f, y + f, z + f);
}

inline MocapPoint& MocapPoint::operator+=(const float f) {
	x += f;
	y += f;
	z += f;
	return *this;
}

inline MocapPoint MocapPoint::operator-(const float f) const {
	return MocapPoint(x - f, y - f, z - f);
}

inline MocapPoint& MocapPoint::operator-=(const float f) {
	x -= f;
	y -= f;
	z -= f;
	return *this;
}

inline MocapPoint MocapPoint::operator*(const float f) const {
	return MocapPoint(x*f, y*f, z*f);
}

inline MocapPoint& MocapPoint::operator*=(const float f) {
	x *= f;
	y *= f;
	z *= f;
	return *this;
}

inline MocapPoint MocapPoint::operator/(const float f) const {
	if (f == 0) return MocapPoint(x, y, z);

	return MocapPoint(x / f, y / f, z / f);
}

inline MocapPoint& MocapPoint::operator/=(const float f) {
	if (f == 0) return *this;

	x /= f;
	y /= f;
	z /= f;
	return *this;
}


// Distance between two points.
//
//
inline float MocapPoint::distance(const MocapPoint& pnt) const {
	float vx = x - pnt.x;
	float vy = y - pnt.y;
	float vz = z - pnt.z;
	return (float)sqrt(vx*vx + vy*vy + vz*vz);
}

inline float  MocapPoint::squareDistance(const MocapPoint& pnt) const {
	float vx = x - pnt.x;
	float vy = y - pnt.y;
	float vz = z - pnt.z;
	return vx*vx + vy*vy + vz*vz;
}


// Length
//
//
inline float MocapPoint::length() const {
	return (float)sqrt(x*x + y*y + z*z);
}

inline float MocapPoint::lengthSquared() const {
	return (float)(x*x + y*y + z*z);
}

/**
* Dot Product.
*/
inline float MocapPoint::dot(const MocapPoint& vec) const {
	return x*vec.x + y*vec.y + z*vec.z;
}

// Non-Member operators
//
//
inline MocapPoint operator+(float f, const MocapPoint& vec) {
	return MocapPoint(f + vec.x, f + vec.y, f + vec.z);
}

inline MocapPoint operator-(float f, const MocapPoint& vec) {
	return MocapPoint(f - vec.x, f - vec.y, f - vec.z);
}

inline MocapPoint operator*(float f, const MocapPoint& vec) {
	return MocapPoint(f*vec.x, f*vec.y, f*vec.z);
}

inline MocapPoint operator/(float f, const MocapPoint& vec) {
	return MocapPoint(f / vec.x, f / vec.y, f / vec.z);
}

/// \endcond