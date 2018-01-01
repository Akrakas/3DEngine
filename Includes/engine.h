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

#define MAX_OBJECTS 10
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
	
	Object_virtual* affected_object;
	Projectile_virtual* affected_projectile;
	
	
	Event_Collision();
};

class Event_Discret_Update : public Event_virtual{
public:
	double update_delay;
	Event_Discret_Update();
};

class Event_Path_Change : public Event_virtual{
public:
	vec3f new_velocity;
	
	Object_virtual* affected_object;
	
	Event_Path_Change();
};

class Engine {
public:
	list<Object_virtual*> objets_list;
	list<Projectile_virtual*> projectiles_list;
	list<Event_virtual*> evenements_list;

	double elapsed_time;
	bool optim_granularity_event_active;
	vec3f optim_gravity_source;
	double gravity_cst;
	
	Engine();
	~Engine();
	Engine(string config_filename);
	
	void update_collision_sphere_withlist(Object_Sphere* affected_object);
	void update_collision_plane_withlist(Object_Plane* affected_object);
	void update_collision_polygon_withlist(Object_Polygon* affected_object);
	void create_collision_projectile_withlist(Projectile_virtual* affected_projectile);
	void update_collision_projectile_withlist(Projectile_virtual* affected_projectile, Event_virtual* event);
	bool erase_evenement_withlist(Event_virtual* event);
	bool erase_projectile_withlist(Projectile_virtual* affected_projectile);
	bool get_object_with_name(Object_virtual** concerned_object, string name);
	
	void step_withlist(double timestep);
	bool get_earliest_event_withlist(Event_virtual** event);
	Projectile_Bullet* create_bullet_withlist(vec3f position, vec3f velocity, string name, bool is_affected_by_gravity, bool is_destroyed_on_contact);
	Projectile_Missile* create_missile_withlist(vec3f position, vec3f velocity, Object_virtual* target, double power, string name, bool is_affected_by_gravity, bool is_destroyed_on_contact);
	Object_Sphere* create_sphere_withlist(vec3f position, vec3f velocity, double radius, string name, string aipath_filename, bool immaterial);
	Object_Plane* create_plane_withlist(vec3f position, vec3f velocity, vec3f normale, string name, string aipath_filename, bool immaterial);
	Object_Polygon* create_polygon_withlist(vec3f position, vec3f velocity, vec3f normale, string name, string aipath_filename, bool immaterial);
	void load_config_from_json(string config_filename);
	void load_world_from_json(string filename);
	void load_aipath_from_json(Object_virtual* affected_object, string aipath_filename);
};


#endif