#ifndef __OBJECTS_H__
#define __OBJECTS_H__

#include <vector>
#include <string>
#include <limits>

#include "vec3f.h"
#include "geometric_shapes.h"

using namespace std;

enum object_type_enum { objNA=0, objPoint, objSphere };
enum projectile_type_enum { projNA=0, projBullet};

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

class Projectile_virtual {
public:
	string name;
	projectile_type_enum projectileType;
	Shape_Point Point;
	double time_before_collision;
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

#endif