#include "world_objects.h"



Object_virtual::Object_virtual(){
	name = "Unnamed";
	objectType = objNA;
	englobing_sphere = Shape_Sphere(vec3f(0,0,0), vec3f(0,0,0), 0);
	immaterial = false;
}

Object_Sphere::Object_Sphere() {
	objectType = objSphere;
}

Object_Sphere::Object_Sphere(vec3f _position, vec3f _velocity, double _radius, string _name){
	name = _name;
	objectType = objSphere;
	englobing_sphere = Shape_Sphere(_position, _velocity, _radius);
}

Object_Plane::Object_Plane() {
	objectType = objPlane;
}

Object_Plane::Object_Plane(vec3f _position, vec3f _velocity, vec3f _normale, string _name){
	name = _name;
	objectType = objPlane;
	plane = Shape_Plane(_position, _velocity, _normale);
}

Object_Polygon::Object_Polygon() {
	objectType = objPolygon;
}

Object_Polygon::Object_Polygon(vec3f _position, vec3f _velocity, vec3f _normale, string _name){
	name = _name;
	objectType = objPolygon;
	polygon = Shape_Polygon(_position, _velocity, _normale);
}

Object_Polyhedron::Object_Polyhedron() {
	objectType = objPolyhedron;
}

Object_Polyhedron::Object_Polyhedron(vec3f _position, vec3f _velocity, string _name){
	name = _name;
	objectType = objPolyhedron;
}

Projectile_virtual::Projectile_virtual(){
	name = "Unnamed";
	projectileType = projNA;
	Point = Shape_Point(vec3f(0,0,0), vec3f(0,0,0));
}

Projectile_Bullet::Projectile_Bullet() {
	projectileType = projBullet;
}

Projectile_Bullet::Projectile_Bullet(vec3f _position, vec3f _velocity, string _name, bool _is_affected_by_gravity){
	name = _name;
	projectileType = projBullet;
	Point = Shape_Point(_position, _velocity);
	is_affected_by_gravity = _is_affected_by_gravity;
}

Projectile_Missile::Projectile_Missile() {
	projectileType = projMissile;
	target = NULL;
	target_position = NULL;
	target_velocity = NULL;
}

Projectile_Missile::Projectile_Missile(vec3f _position, vec3f _velocity, Object_virtual* _target, double _power, string _name, bool _is_affected_by_gravity){
	name = _name;
	projectileType = projMissile;
	Point = Shape_Point(_position, _velocity);
	is_affected_by_gravity = _is_affected_by_gravity;
	target = _target;
	power = _power;
	Bind_target();
}

void Projectile_Missile::Bind_target() {
	if(target != NULL) {
		if(target->objectType == objSphere || target->objectType == objPolyhedron) {
			target_position = &(target->englobing_sphere.position);
			target_velocity = &(target->englobing_sphere.velocity);
		} else if(target->objectType == objPlane) {
			Object_Plane* plane = (Object_Plane*)target;
			target_position = &(plane->plane.position);
			target_velocity = &(plane->plane.velocity);
		}
	}
}
