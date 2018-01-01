#ifndef __PHYSICS_H__
#define __PHYSICS_H__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "vec3f.h"
#include "geometric_shapes.h"

using namespace std;

vec3f ClosestPointOnLine(vec3f l1, vec3f l2, vec3f p);
bool LineIsGoingToCollideSphere(vec3f line_point, vec3f line_vec_norm, vec3f sphere_center, double sphere_radius);
bool LineContinuousCollisionSphere(Shape_Point* Projectile, Shape_Sphere* Sphere, double* time_before_collision);
bool LineContinuousCollisionPlane(Shape_Point* Projectile, Shape_Plane* Plane, double* time_before_collision);
bool LineContinuousCollisionPolygon(Shape_Point* Projectile, Shape_Polygon* Polygon, double* time_before_collision);

vec3f optim_ClosestPointOnLine(vec3f line_point, vec3f line_vec_norm, vec3f point);
bool optim_LineIsGoingToCollideSphere(vec3f line_point, vec3f line_vec_norm, vec3f sphere_center, double sphere_radius_squared);
bool optim_LineContinuousCollisionSphere(Shape_Point* Projectile, Shape_Sphere* Sphere, double* time_before_collision);
bool optim_LineContinuousCollisionPlane(Shape_Point* Projectile, Shape_Plane* Plane, double* time_before_collision);
bool optim_LineContinuousCollisionPolygon(Shape_Point* Projectile, Shape_Polygon* Polygon, double* time_before_collision);

double optim_invsqrt(double number);
#endif