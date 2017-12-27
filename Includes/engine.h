#ifndef __ENGINE_H__
#define __ENGINE_H__

#include <limits>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>

#include "vec3f.h"
#include "geometric_shapes.h"
#include "physics.h"
#include "world_objects.h"
#include "json.h"

#define MAX_SPHERES 10
#define MAX_PROJECTILES 10

using namespace std;

enum event_type_enum { eventNA=0, eventCollision, eventDiscretUpdate, eventPathChange};

class Event_virtual {
public:
	string name;
	event_type_enum eventType;
	double time_of_event;
	
	Event_virtual();
};

class Event_Collision : public Event_virtual{
public:
	int index_projectile;
	int index_object;
	
	Object_Sphere* affected_object;
	Projectile_Bullet* affected_projectile;
	
	
	Event_Collision();
	Event_Collision(int _index_projectile, int _index_object);
	Event_Collision(list<Object_Sphere*>::iterator _affected_object, list<Projectile_Bullet*>::iterator _affected_projectile);
};

class Event_Discret_Update : public Event_virtual{
public:
	double update_delay;
	Event_Discret_Update();
};

class Event_Path_Change : public Event_virtual{
public:
	vec3f new_velocity;
	
	Object_Sphere* affected_object;
	
	Event_Path_Change();
};

class Engine {
public:
	list<Object_Sphere*> objets_list;
	list<Projectile_Bullet*> projectiles_list;
	list<Event_virtual*> evenements_list;

	double elapsed_time;
	bool optim_granularity_event_active;
	vec3f optim_gravity_source;
	double gravity_cst;
	
	Engine();
	~Engine();
	Engine(string config_filename);
	
	void update_collision_sphere_withlist(Object_Sphere* affected_object);
	void create_collision_projectile_withlist(Projectile_Bullet* affected_projectile);
	void update_collision_projectile_withlist(Projectile_Bullet* affected_projectile, Event_virtual* event);
	bool erase_evenement_withlist(Event_virtual* event);
	bool erase_projectile_withlist(Projectile_Bullet* affected_projectile);
	
	Object_Sphere* create_sphere_withlist(vec3f position, vec3f velocity, double radius, string name, string aipath_filename, bool immaterial);
	void step_withlist(double timestep);
	void update_collision_projectile_withlist(list<Projectile_Bullet*>::iterator it_projectile, list<Event_virtual*>::iterator it_event);
	bool get_earliest_event_withlist(Event_virtual** event);
	Projectile_Bullet* create_projectile_withlist(vec3f position, vec3f velocity, string name, bool is_affected_by_gravity, bool is_destroyed_on_contact);
	
	void load_config_from_json(string config_filename);
	void load_world_from_json(string filename);
	void load_aipath_from_json(Object_Sphere* affected_object, string aipath_filename);
};


#endif