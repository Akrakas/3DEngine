#include "physics.h"

//line_point, lvec_norm = points sur la droite, direction de la droite
//p = point que l'on veut projeter sur la droite
//Renvois les coordonnées de la projection du point p sur la ligne line_point->line_vec_norm
vec3f ClosestPointOnLine(vec3f line_point, vec3f line_vec_norm, vec3f point) {
    vec3f l1minp = point-line_point;
    double t = l1minp.dot(line_vec_norm);
    vec3f l2multp = (line_vec_norm * t);
    return line_point + l2multp;
}

vec3f optim_ClosestPointOnLine(vec3f line_point, vec3f line_vec_norm, vec3f point) {
	double dotprod = ((point.x-line_point.x)*line_vec_norm.x + (point.y-line_point.y)*line_vec_norm.y + (point.z-line_point.z)*line_vec_norm.z);
    return vec3f(line_point.x + line_vec_norm.x*dotprod, line_point.y + line_vec_norm.y*dotprod, line_point.z + line_vec_norm.z*dotprod);
}

//Renvois vrai si la ligne va rentrer en contact avec la sphere
bool LineIsGoingToCollideSphere(vec3f line_point, vec3f line_vec_norm, vec3f sphere_center, double sphere_radius) {
	if(sphere_center.dist(line_point) < sphere_radius) {return false;}
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

bool optim_LineIsGoingToCollideSphere(vec3f line_point, vec3f line_vec_norm, vec3f sphere_center, double sphere_radius_squared) {
	if((line_point.x - sphere_center.x)*(line_point.x - sphere_center.x) + (line_point.y - sphere_center.y)*(line_point.y - sphere_center.y) + (line_point.z - sphere_center.z)*(line_point.z - sphere_center.z) < sphere_radius_squared) {return false;}
	if(((sphere_center.x - line_point.x)*line_vec_norm.x + (sphere_center.y - line_point.y)*line_vec_norm.y + (sphere_center.z - line_point.z)*line_vec_norm.z) < 0) {return false;} //Le projectile ne va pas vers la sphere
	double dotprod = ((sphere_center.x-line_point.x)*line_vec_norm.x + (sphere_center.y-line_point.y)*line_vec_norm.y + (sphere_center.z-line_point.z)*line_vec_norm.z);
	if(((line_point.x+(line_vec_norm.x*dotprod)) - sphere_center.x)*((line_point.x+(line_vec_norm.x*dotprod)) - sphere_center.x) + ((line_point.y+(line_vec_norm.y*dotprod)) - sphere_center.y)*((line_point.y+(line_vec_norm.y*dotprod)) - sphere_center.y) + ((line_point.z+(line_vec_norm.z*dotprod)) - sphere_center.z)*((line_point.z+(line_vec_norm.z*dotprod)) - sphere_center.z) > sphere_radius_squared) {return false;}
	return true;
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

bool optim_LineContinuousCollisionSphere(Shape_Point* Projectile, Shape_Sphere* Sphere, double* time_before_collision) {
	if((Projectile->position.x - Sphere->position.x)*(Projectile->position.x - Sphere->position.x) + (Projectile->position.y - Sphere->position.y)*(Projectile->position.y - Sphere->position.y) + (Projectile->position.z - Sphere->position.z)*(Projectile->position.z - Sphere->position.z) < Sphere->optim_sqradius) {return false;} //On est déja dans la sphere
	double projectile_relative_velocity_x = Projectile->velocity.x - Sphere->velocity.x;
	double projectile_relative_velocity_y = Projectile->velocity.y - Sphere->velocity.y;
	double projectile_relative_velocity_z = Projectile->velocity.z - Sphere->velocity.z;
	if(((Sphere->position.x - Projectile->position.x)*projectile_relative_velocity_x + (Sphere->position.y - Projectile->position.y)*projectile_relative_velocity_y + (Sphere->position.z - Projectile->position.z)*projectile_relative_velocity_z) < 0) {return false;}

	double velocity_length = std::sqrt(projectile_relative_velocity_x*projectile_relative_velocity_x + projectile_relative_velocity_y*projectile_relative_velocity_y + projectile_relative_velocity_z*projectile_relative_velocity_z);
	double dotprod = ((Sphere->position.x-Projectile->position.x)*(projectile_relative_velocity_x/velocity_length) + (Sphere->position.y-Projectile->position.y)*(projectile_relative_velocity_y/velocity_length) + (Sphere->position.z-Projectile->position.z)*(projectile_relative_velocity_z/velocity_length));
	double vector_projection_point_x = (projectile_relative_velocity_x/velocity_length) * dotprod;
	double vector_projection_point_y = (projectile_relative_velocity_y/velocity_length) * dotprod;
	double vector_projection_point_z = (projectile_relative_velocity_z/velocity_length) * dotprod;
	double range = ((Projectile->position.x + vector_projection_point_x) - Sphere->position.x)*((Projectile->position.x + vector_projection_point_x) - Sphere->position.x) + ((Projectile->position.y + vector_projection_point_y) - Sphere->position.y)*((Projectile->position.y + vector_projection_point_y) - Sphere->position.y) + ((Projectile->position.z + vector_projection_point_z) - Sphere->position.z)*((Projectile->position.z + vector_projection_point_z) - Sphere->position.z);
	if(range > Sphere->optim_sqradius) {return false;}
	double distance_projection_point = std::sqrt((vector_projection_point_x)*(vector_projection_point_x) + (vector_projection_point_y)*(vector_projection_point_y) + (vector_projection_point_z)*(vector_projection_point_z));
	double triangle_leg = std::sqrt(Sphere->optim_sqradius - range);
	*(time_before_collision) = ((distance_projection_point - triangle_leg) / velocity_length) + 0.00000001;
	return true;
}

//Renvois vrai si la ligne va rentrer en contact avec le plan, et met le temps avant cette collision dans "time_before_collision"
bool LineContinuousCollisionPlane(Shape_Point* Projectile, Shape_Plane* Plane, double* time_before_collision) {
	//On calcul la vitesse relative du projectile par rapport au plan
	vec3f projectile_relative_velocity = Projectile->velocity - Plane->velocity;
	if((projectile_relative_velocity).dot(Plane->normale) > 0) {return false;} //Le projectile ne va pas vers le plan dans le bon sens
	double dotNumerator = (Plane->position - Projectile->position).dot(Plane->normale);
	double dotDenominator = projectile_relative_velocity.dot(Plane->normale);
	if(dotDenominator != 0.0f){
		*time_before_collision = (dotNumerator / dotDenominator);
		if(*time_before_collision < 0) return false;
		*time_before_collision += 0.00000001;
		return true;
	} else { //La ligne est parallele au plan
		return false;
	}
}

bool optim_LineContinuousCollisionPlane(Shape_Point* Projectile, Shape_Plane* Plane, double* time_before_collision) {
	double projectile_relative_velocity_x = Projectile->velocity.x - Plane->velocity.x;
	double projectile_relative_velocity_y = Projectile->velocity.y - Plane->velocity.y;
	double projectile_relative_velocity_z = Projectile->velocity.z - Plane->velocity.z;
	double dotDenominator = (projectile_relative_velocity_x*Plane->normale.x + projectile_relative_velocity_y*Plane->normale.y + projectile_relative_velocity_z*Plane->normale.z);
	if(dotDenominator >= 0.0f){return false;}
	double dotNumerator = ((Plane->position.x-Projectile->position.x)*Plane->normale.x + (Plane->position.y-Projectile->position.y)*Plane->normale.y + (Plane->position.z-Projectile->position.z)*Plane->normale.z);
	if(dotNumerator > 0.0f){return false;}
	*time_before_collision = (dotNumerator / dotDenominator) + 0.00000001;
	return true;
}

//Renvois vrai si la ligne va rentrer en contact avec le polygone, et met le temps avant cette collision dans "time_before_collision"
//Cette opération prend plus ou moins le meme temps que la collision avec une simple sphere, il vaudrait donc mieux utiliser le plus de spheres possible, 
//car il faut le faire pour chaque coté du polyhédron qui contient ces polygones
bool LineContinuousCollisionPolygon(Shape_Point* Projectile, Shape_Polygon* Polygon, double* time_before_collision) {
	//On calcul la vitesse relative du projectile par rapport au polygone
	vec3f projectile_relative_velocity = Projectile->velocity - Polygon->velocity;
	if((projectile_relative_velocity).dot(Polygon->normale) > 0) {return false;} //Le projectile ne va pas vers le polygone dans le bon sens
	double dotNumerator = (Polygon->position - Projectile->position).dot(Polygon->normale);
	double dotDenominator = projectile_relative_velocity.dot(Polygon->normale);
	if(dotDenominator != 0.0f){
		*time_before_collision = (dotNumerator / dotDenominator);
		if(*time_before_collision < 0) return false;
		vec3f intersection = Projectile->position + projectile_relative_velocity.truncate(*time_before_collision);
		//On verifie que le point d'intersection avec le plan du polygone se trouve à l'interieur de tout les cotés du polygone (qui sont représentés par des plans). 
		for (list<Shape_Plane*>::iterator it_side=Polygon->sides.begin(); it_side != Polygon->sides.end(); ++it_side) {
			if((*it_side)->normale.dot(intersection - (*it_side)->position) > 0) { //On est à l'exterieur d'un plan : on est en dehors du polygone
				return false;
			}
		}
		//Tout bon, on est dans le polygone : on applique les meme regles que pour la collision avec un simple plan
		*time_before_collision += 0.00000001;
		return true;
	} else { //La ligne est parallele au plan du polygone
		return false;
	}
}

