#ifndef __GEOMETRIC_SHAPES_H__
#define __GEOMETRIC_SHAPES_H__

#include <vector>
#include <string>
#include "vec3f.h"

using namespace std;

enum shape_type_enum { shapeNA=0, shapePoint, shapeSphere, shapePolygon };

class Shape_virtual {
public:
	shape_type_enum shapeType;
	vec3f position;
	vec3f velocity;
	vec3f velocity_normal;
	vector<string> tags;
	double time_before_collision;
	Shape_virtual* collided_shape;
	
	Shape_virtual();
};

class Shape_Point : public Shape_virtual{
public:
	
	Shape_Point();
	Shape_Point(vec3f _position, vec3f _velocity);
};

class Shape_Sphere : public Shape_virtual{
public:
	double radius;
	
	Shape_Sphere();
	Shape_Sphere(vec3f _position, vec3f _velocity, double radius);
};


#endif