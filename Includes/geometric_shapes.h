#ifndef __GEOMETRIC_SHAPES_H__
#define __GEOMETRIC_SHAPES_H__

#include <list>
#include <string>
#include "vec3f.h"

using namespace std;

enum shape_type_enum { shapeNA=0, shapePoint, shapeSphere, shapePlan, shapePolygon };

class Shape_virtual {
public:
	shape_type_enum shapeType;
	vec3f position;
	vec3f velocity;
	vec3f velocity_normal;
	
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

class Shape_Plane : public Shape_virtual{
public:
	vec3f normale;
	
	Shape_Plane();
	Shape_Plane(vec3f _position, vec3f _velocity, vec3f _normal);
};

class Shape_Polygon : public Shape_virtual{
public:
	vec3f normale;
	list<Shape_Plane*> sides;
	
	Shape_Polygon();
	Shape_Polygon(vec3f _position, vec3f _velocity, vec3f _normal);
	void add_side(vec3f offset);
};


#endif