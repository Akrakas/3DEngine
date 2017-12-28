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
	time_before_collision = std::numeric_limits<double>::max();
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
