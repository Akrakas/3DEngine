#include "physics.h"

//line_point, lvec_norm = points sur la droite, direction de la droite
//p = point que l'on veut projeter sur la droite
//Renvois les coordonnées de la projection du point p sur la ligne llinepp->line_vec_norm
vec3f ClosestPointOnLine(vec3f line_point, vec3f line_vec_norm, vec3f point) {
    vec3f l1minp = point-line_point;
    double t = l1minp.dot(line_vec_norm);
    vec3f l2multp = (line_vec_norm * t);
    return line_point + l2multp;
}

//Renvois vrai si la ligne va rentrer en contact avec la sphere
bool LineSphereIsGoingToCollide(vec3f line_point, vec3f line_vec_norm, vec3f sphere_center, double sphere_radius) {
	if(sphere_center.dist(line_point) < sphere_radius) {return false;} //On est déja dans la sphere
	//Sanity check : on verifie que la ligne va vers la sphere
	if((sphere_center - line_point).dot(line_vec_norm) < 0) {return false;} //Le projectile ne va pas vers la sphere
	vec3f center_projection = ClosestPointOnLine(line_point, line_vec_norm, sphere_center);
	double range = sphere_center.dist(center_projection);
	if(range > sphere_radius) {//Pas de colisions, le point le plus proche est plus loin que le rayon de la sphere
		return false;
	} else { //On rentre en colision
		return true;
	}
}

//Renvois vrai si la ligne va rentrer en contact avec la sphere, et met le temps avant cette collision dans "time_before_collision"
bool LineContinuousCollisionSphere(Shape_Point* Projectile, Shape_Sphere* Sphere, double* time_before_collision) {
	if(Sphere->position.dist(Projectile->position) < Sphere->radius) {return false;} //On est déja dans la sphere
	//On calcul la vitesse relative du projectile par rapport à la sphere
	vec3f projectile_relative_velocity = Projectile->velocity - Sphere->velocity;
	if((projectile_relative_velocity).dot(Sphere->position - Projectile->position) < 0) {return false;} //Le projectile ne va pas vers la sphere
	//On projete le centre de la sphere sur la trajectoire du projectile
	vec3f center_projection = ClosestPointOnLine(Projectile->position, projectile_relative_velocity.normal(), Sphere->position);
	double range = Sphere->position.dist(center_projection);
	if(range > Sphere->radius) {return false;} //Le point le plus proche est plus loin que le rayon de la sphere
	else { //On rentre en colision, le projectile passe par la sphere à un moment donné : on calcul le temps avant de rentrer en collision
		//Distance entre la projection du centre de la sphere et le point de collision
		//Merci Pythagore
		double sqrt_leg = sqrt(Sphere->radius*Sphere->radius - range*range);
		//Distance entre le projectile et la projection du centre de la sphere
		double distance_projection_point = (center_projection - Projectile->position).length();
		*(time_before_collision) = ((distance_projection_point - sqrt_leg) / (double)projectile_relative_velocity.length()) + 0.00000001;
		return true;
	}
}