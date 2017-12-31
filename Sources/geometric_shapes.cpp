#include "geometric_shapes.h"

Shape_virtual::Shape_virtual(){
	shapeType = shapeNA;
	position.set(0,0,0);
	velocity.set(0,0,0);
}

Shape_Point::Shape_Point(){
	shapeType = shapePoint;
	position = vec3f(0,0,0);
	velocity = vec3f(0,0,0);
}

Shape_Point::Shape_Point(vec3f _position, vec3f _velocity){
	shapeType = shapePoint;
	position = _position;
	velocity = _velocity;
}

Shape_Sphere::Shape_Sphere(){
	shapeType = shapeSphere;
	radius = 0;
	optim_sqradius = 0;
	position = vec3f(0,0,0);
	velocity = vec3f(0,0,0);
}

Shape_Sphere::Shape_Sphere(vec3f _position, vec3f _velocity, double _radius){
	shapeType = shapeSphere;
	radius = _radius;
	optim_sqradius = radius*radius;
	position = _position;
	velocity = _velocity;
}

Shape_Plane::Shape_Plane(){
	shapeType = shapePlan;
	position = vec3f(0,0,0);
	velocity = vec3f(0,0,0);
	normale = vec3f(0,0,0);
}

Shape_Plane::Shape_Plane(vec3f _position, vec3f _velocity, vec3f _normale){
	shapeType = shapePlan;
	position = _position;
	velocity = _velocity;
	normale = _normale;
}

Shape_Polygon::Shape_Polygon(){
	shapeType = shapePolygon;
	position = vec3f(0,0,0);
	velocity = vec3f(0,0,0);
	normale = vec3f(0,0,0);
}

Shape_Polygon::Shape_Polygon(vec3f _position, vec3f _velocity, vec3f _normale){
	shapeType = shapePolygon;
	position = _position;
	velocity = _velocity;
	normale = _normale;
}

void Shape_Polygon::add_side(vec3f offset) {
	Shape_Plane* plan = new Shape_Plane(position + offset, velocity, offset.normal());
	sides.insert(sides.begin(), plan);
}