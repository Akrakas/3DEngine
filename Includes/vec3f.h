#ifndef __VEC3F_H__
#define __VEC3F_H__
//Vectors calcultaions
//Adapté de vec2f.h
//C'etait des floats au depart, d'ou le f

#include <cmath>
#include <cstdio>
#include <cstdlib>

class vec3f {
public:
	double x, y, z;

	vec3f() :x(0), y(0), z(0) {}
	vec3f(double x, double y, double z) : x(x), y(y), z(z) {}
	vec3f(const vec3f& v) : x(v.x), y(v.y), z(v.z) {}

	vec3f& operator=(const vec3f& v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}
	
// 	vec3f operator+(vec3f& v) {
// 		return vec3f(x + v.x, y + v.y, z + v.z);
// 	}
	vec3f operator+(vec3f v) {
		return vec3f(x + v.x, y + v.y, z + v.z);
	}
	vec3f operator-(vec3f& v) {
		return vec3f(x - v.x, y - v.y, z - v.z);
	}

	vec3f& operator+=(vec3f& v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	vec3f& operator-=(vec3f& v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}


	vec3f operator+(double s) {
		return vec3f(x + s, y + s, z + s);
	}
	vec3f operator-(double s) {
		return vec3f(x - s, y - s, z - s);
	}
	vec3f operator*(double s) {
		return vec3f(x * s, y * s, z * s);
	}
	vec3f operator/(double s) {
		return vec3f(x / s, y / s, z / s);
	}


	vec3f& operator+=(double s) {
		x += s;
		y += s;
		z += s;
		return *this;
	}
	vec3f& operator-=(double s) {
		x -= s;
		y -= s;
		z -= s;
		return *this;
	}
	vec3f& operator*=(double s) {
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}
	vec3f& operator/=(double s) {
		x /= s;
		y /= s;
		z /= s;
		return *this;
	}

	void set(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

// 	void rotate(double deg) {
// 		double theta = deg / 180.0 * M_PI;
// 		double c = cos(theta);
// 		double s = sin(theta);
// 		double tx = x * c - y * s;
// 		double ty = x * s + y * c;
// 		x = tx;
// 		y = ty;
// 	}

	vec3f& normalize() {
		if (length() == 0) return *this;
		*this *= (1.0 / length());
		return *this;
	}

	vec3f normal() {
		vec3f v1(x,y,z);
		if (v1.length() == 0) return v1;
		v1 *= (1.0 / v1.length());
		return v1;
	}

	double dist(vec3f v) const {
		vec3f d(v.x - x, v.y - y, v.z - z);
		return d.length();
	}
	double length() const {
		return std::sqrt(x * x + y * y + z * z);
	}

	double sqrlength() const {
		return (x * x + y * y + z * z);
	}

// 	void truncate(double length) {
// 		double angle = atan2f(y, x);
// 		x = length * cos(angle);
// 		y = length * sin(angle);
// 	}

//	vec3f ortho() const {
//		return vec3f(y, -x);
//	}

	double dot(vec3f v) {
		return x * v.x + y * v.y + z * v.z;
	}

	void reverse() {
		x = -x;
		y = -y;
		z = -z;
	}

	void print() {
		printf("x = %f, y = %f, z = %f\n", x, y, z);
	}
// 	static double cross(vec3f v1, vec3f v2) {
// 		return (v1.x * v2.y) - (v1.y * v2.x);
// 	}
};

#endif