#include "geometric_shapes.h"

Shape_virtual::Shape_virtual(){
	shapeType = shapeNA;
	position.set(0,0,0);
	velocity.set(0,0,0);
	collided_shape = NULL;
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
	position = vec3f(0,0,0);
	velocity = vec3f(0,0,0);
}

Shape_Sphere::Shape_Sphere(vec3f _position, vec3f _velocity, double _radius){
	shapeType = shapeSphere;
	radius = _radius;
	position = _position;
	velocity = _velocity;
}


