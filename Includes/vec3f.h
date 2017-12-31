#ifndef __VEC3F_H__
#define __VEC3F_H__
//Vectors calcultaions
//Adapté de vec2f.h
//C'etait des floats au depart, d'ou le f

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream>

class vec3f {
public:
	double x, y, z;

	inline vec3f() :x(0), y(0), z(0) {}
	inline vec3f(double x, double y, double z) : x(x), y(y), z(z) {}
	inline vec3f(const vec3f& v) : x(v.x), y(v.y), z(v.z) {}

	inline vec3f& operator=(const vec3f& v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}
	
// 	vec3f operator+(vec3f& v) {
// 		return vec3f(x + v.x, y + v.y, z + v.z);
// 	}
	inline vec3f operator+(vec3f v) {
		return vec3f(x + v.x, y + v.y, z + v.z);
	}
	inline vec3f operator-(vec3f& v) {
		return vec3f(x - v.x, y - v.y, z - v.z);
	}
	
	inline vec3f& operator+=(vec3f& v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	inline vec3f& operator-=(vec3f& v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}


	inline vec3f operator+(double s) {
		return vec3f(x + s, y + s, z + s);
	}
	inline vec3f operator-(double s) {
		return vec3f(x - s, y - s, z - s);
	}
	inline vec3f operator*(double s) {
		return vec3f(x * s, y * s, z * s);
	}
	inline vec3f operator/(double s) {
		return vec3f(x / s, y / s, z / s);
	}


	inline vec3f& operator+=(double s) {
		x += s;
		y += s;
		z += s;
		return *this;
	}
	inline vec3f& operator-=(double s) {
		x -= s;
		y -= s;
		z -= s;
		return *this;
	}
	inline vec3f& operator*=(double s) {
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}
	inline vec3f& operator/=(double s) {
		x /= s;
		y /= s;
		z /= s;
		return *this;
	}

	inline void set(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

// 	inline void rotate(double deg) {
// 		double theta = deg / 180.0 * M_PI;
// 		double c = cos(theta);
// 		double s = sin(theta);
// 		double tx = x * c - y * s;
// 		double ty = x * s + y * c;
// 		x = tx;
// 		y = ty;
// 	}

	inline vec3f& normalize() {
		if (length() == 0) return *this;
		*this *= (1.0 / length());
		return *this;
	}

	inline vec3f normal() {
		vec3f v1(x,y,z);
		if (v1.length() == 0) return v1;
		v1 *= (1.0 / v1.length());
		return v1;
	}

	inline double dist(vec3f v) const {
		vec3f d(v.x - x, v.y - y, v.z - z);
		return d.length();
	}
	inline double length() const {
		return std::sqrt(x * x + y * y + z * z);
	}

	inline double sqrlength() const {
		return (x * x + y * y + z * z);
	}

	inline vec3f truncate(double length) {
		vec3f normal = this->normal();
		normal *= length;
		return normal;
	}

//	inline vec3f ortho() const {
//		return vec3f(y, -x);
//	}

	inline double dot(vec3f v) {
		return x * v.x + y * v.y + z * v.z;
	}

	inline void reverse() {
		x = -x;
		y = -y;
		z = -z;
	}

	std::string print() {
		std::stringstream output;
		output << "x = " << x << ", y = " << y << ", z = " << z;
		return output.str();
	}
// 	inline static double cross(vec3f v1, vec3f v2) {
// 		return (v1.x * v2.y) - (v1.y * v2.x);
// 	}
};

#endif