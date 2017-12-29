#ifndef __OBJECTS_H__
#define __OBJECTS_H__

#include <vector>
#include <string>
#include <limits>

#include "vec3f.h"
#include "geometric_shapes.h"

using namespace std;

enum object_type_enum { objNA=0, objPoint, objSphere, objPlane, objPolygon, objPolyhedron};
enum projectile_type_enum { projNA=0, projBullet, projMissile};

class Object_virtual {
public:
	string name;
	object_type_enum objectType;
	Shape_Sphere englobing_sphere;
	bool immaterial;
	
	Object_virtual();
};

class Object_Sphere : public Object_virtual{
public:
	double radius;
	
	Object_Sphere();
	Object_Sphere(vec3f _position, vec3f _velocity, double radius, string _name);
};

class Object_Plane : public Object_virtual{
public:
	Shape_Plane plane;
	
	Object_Plane();
	Object_Plane(vec3f _position, vec3f _velocity, vec3f _normale, string _name);
};

class Object_Polygon : public Object_virtual{
public:
	Shape_Polygon polygon;
	
	Object_Polygon();
	Object_Polygon(vec3f _position, vec3f _velocity, vec3f _normale, string _name);
};

class Object_Polyhedron : public Object_virtual{
public:
	list<Shape_Polygon*> polygons;
	
	Object_Polyhedron();
	Object_Polyhedron(vec3f _position, vec3f _velocity, string _name);
};

class Projectile_virtual {
public:
	string name;
	projectile_type_enum projectileType;
	Shape_Point Point;
	double granularity_debt;
	bool is_affected_by_gravity;
	bool is_destroyed_on_contact;
	
	Projectile_virtual();
};

class Projectile_Bullet : public Projectile_virtual{
public:
	
	Projectile_Bullet();
	Projectile_Bullet(vec3f _position, vec3f _velocity, string _name, bool _is_affected_by_gravity);
};

class Projectile_Missile : public Projectile_virtual{
public:
	double power;
	Object_virtual* target;
	vec3f* target_position;
	vec3f* target_velocity;

	
	Projectile_Missile();
	Projectile_Missile(vec3f _position, vec3f _velocity, Object_virtual* _target, double _power, string _name, bool _is_affected_by_gravity);
	void Bind_target();
};

#endif