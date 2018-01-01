#include "engine.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////CONSTRUCTEURS/////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

Engine::Engine() {
	elapsed_time = 0;
	gravity_cst = 9.81;
	
	Event_Discret_Update* event = new Event_Discret_Update();
	event->update_delay = 1.5;
	event->time_of_event = std::numeric_limits<double>::max();
	optim_granularity_event_active = false;
	evenements_list.insert(evenements_list.begin(), event);
}

Engine::Engine(string config_filename) {
	load_config_from_json(config_filename);
}

Engine::~Engine() {
}

Event_virtual::Event_virtual() {
	name = "";
	eventType = eventNA;
	time_of_event = std::numeric_limits<double>::max();
}

Event_Collision::Event_Collision() {
	eventType = eventCollision;
}

Event_Discret_Update::Event_Discret_Update() {
	eventType = eventDiscretUpdate;
}

Event_Path_Change::Event_Path_Change() {
	eventType = eventPathChange;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////TRAITEMENTS///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Engine::step_withlist(double timestep) {
	do {
		Event_virtual* event;

		list<Event_virtual*>::iterator it_earliest_event = evenements_list.begin();
		double time_of_event = std::numeric_limits<double>::max();
		for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
			if((*it_event)->time_of_event < time_of_event) {
				it_earliest_event = it_event;
				time_of_event = (*it_event)->time_of_event;
			}
			//cout << "time_of_event de event = " << (*it_event)->time_of_event << endl;
		}
		event = (*it_earliest_event);
		
		//cout << "Nombre d'evenements = " << evenements_list.size() << endl;			
		//cout << "Prochain evenements à " << event->time_of_event << ", soit dans " << event->time_of_event - elapsed_time << " secondes." << endl;
		
		double max_timestep = min(timestep, event->time_of_event - elapsed_time);
				
		for (list<Object_virtual*>::iterator it_object=objets_list.begin(); it_object != objets_list.end(); ++it_object) {
			if((*(it_object))->objectType == objSphere) {
				(*it_object)->englobing_sphere.position =  (*it_object)->englobing_sphere.position + ((*it_object)->englobing_sphere.velocity * max_timestep);
			} else if((*(it_object))->objectType == objPlane) {
				Object_Plane* Plane = (Object_Plane*)(*it_object);
				Plane->plane.position =  Plane->plane.position + (Plane->plane.velocity * max_timestep);
			} else if((*(it_object))->objectType == objPolygon) {
				Object_Polygon* Polygon = (Object_Polygon*)(*it_object);
				Polygon->polygon.position =  Polygon->polygon.position + (Polygon->polygon.velocity * max_timestep);
			}
		}
		
		for (list<Projectile_virtual*>::iterator it_projectile=projectiles_list.begin(); it_projectile != projectiles_list.end(); ++it_projectile) {
			(*it_projectile)->Point.position =  (*it_projectile)->Point.position + ((*it_projectile)->Point.velocity * max_timestep);
		}

		elapsed_time += max_timestep;
		
		//Au moins 1 evenement à eu lieu dans l'interval timestep
		if(max_timestep < timestep) {
			if(event->eventType == eventCollision) {
				Event_Collision* collision = (Event_Collision*)event;
				if(collision->affected_object->immaterial != true || collision->affected_projectile->is_destroyed_on_contact == true) {
					cout << "Le projectile \"" << collision->affected_projectile->name << "\" impacte l'objet \"" << collision->affected_object->name << "\" à T=" << elapsed_time << endl;
					erase_projectile_withlist(collision->affected_projectile);
				} else {
					cout << "Le projectile \"" << collision->affected_projectile->name << "\" traverse l'objet \"" << collision->affected_object->name << "\" à T=" << elapsed_time << endl;
					update_collision_projectile_withlist(collision->affected_projectile, event);
				}
			} else if(event->eventType == eventDiscretUpdate) {
				Event_Discret_Update* update = (Event_Discret_Update*)event;
				cout << "Granularité." << endl;
				int projectile_needing_granularity_counter = 0;
				for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
					if((*it_event)->eventType == eventCollision) {
						Event_Collision* collision = (Event_Collision*)(*it_event);
						if(collision->affected_projectile->is_affected_by_gravity) {
							projectile_needing_granularity_counter++;
							//Reetablis vz à sa valeur correcte
							double debt_value = 0.5*gravity_cst*collision->affected_projectile->granularity_debt; //0.5*g*ecart
							collision->affected_projectile->Point.velocity.z += (debt_value - gravity_cst*collision->affected_projectile->granularity_debt); //vz = (debt - g*ecart)
							
							//Calcul la valeur à ajouter à vz pour avoir une ballistique correcte, plus la dette à ajouter à la prochaine iteration
							collision->affected_projectile->granularity_debt = update->update_delay;
							collision->affected_projectile->Point.velocity.z -= 0.5*gravity_cst*collision->affected_projectile->granularity_debt;
							
							//Recalcul des collisions
							update_collision_projectile_withlist(collision->affected_projectile, collision);
						}
						if(collision->affected_projectile->projectileType == projMissile) {
							projectile_needing_granularity_counter++;
							Projectile_Missile* missile = (Projectile_Missile*)collision->affected_projectile;
							if(missile->target == NULL) {
								cout << "Missile \"" << missile->name << "\" : NO TARGET" << endl;
								//self destruct or something
							} else {
								cout << "Missile \"" << missile->name << "\" : TARGET AQUIRED" << endl;
								vec3f vector_to_target_normale = (*(missile->target_position) - missile->Point.position).normal();
								cout << "CIBLE : ";
								(*(missile->target_position) - missile->Point.position).print();
								//if(vector_to_target_normale.dot(missile->Point.velocity) > 0) vector_to_target_normale = vec3f(0,0,0)-vector_to_target_normale;
								vec3f part2 = (vector_to_target_normale * (missile->Point.velocity.dot(vector_to_target_normale)))*2.0;
								vec3f part3 = (missile->Point.velocity - part2);
								vec3f reflection = (vec3f(0,0,0)-part3);
								cout << "COMPENSATION : ";
								reflection.print();
								missile->Point.velocity += reflection;
								vec3f part4 = (vector_to_target_normale * ((missile->power*update->update_delay) - reflection.length()));
								missile->Point.velocity += part4;
							}
							update_collision_projectile_withlist(collision->affected_projectile, collision);
						}
					}
				}
				if(projectile_needing_granularity_counter == 0) {
					update->time_of_event = std::numeric_limits<double>::max();
				} else {
					update->time_of_event += update->update_delay;
				}
			} else if(event->eventType == eventPathChange) {
				Event_Path_Change* change = (Event_Path_Change*)event;
				cout << "Path change." << endl;
				change->affected_object->englobing_sphere.velocity = change->new_velocity;
				erase_evenement_withlist(event);
			} else {
				cout << "Evenement inconnu. Ne devrait pas arriver." << endl;
				erase_evenement_withlist(event);
			}
			

		}
		
		timestep -= max_timestep;
	} while(timestep > 0);
}

bool Engine::get_earliest_event_withlist(Event_virtual** event) {
	if(evenements_list.size() > 0) {
		double time_of_event = std::numeric_limits<double>::max();
		for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
			if((*it_event)->time_of_event < time_of_event) *event = *it_event;
			time_of_event = (*it_event)->time_of_event;
		}
		return true;
	} else {
		return false;
	}
}

Object_Sphere* Engine::create_sphere_withlist(vec3f position, vec3f velocity, double radius, string name, string aipath_filename, bool immaterial) {
	if(objets_list.size() < MAX_OBJECTS) {
		Object_Sphere* new_sphere_ptr = new Object_Sphere(position, velocity, radius, name);
		update_collision_sphere_withlist(new_sphere_ptr);
		if(aipath_filename.size() != 0) {
			load_aipath_from_json(new_sphere_ptr, aipath_filename);
		}
		new_sphere_ptr->immaterial = immaterial;
		cout << "Creation de l'objet \"" << new_sphere_ptr->name << "\" à T=" << elapsed_time << endl;
		objets_list.insert(objets_list.begin(), new_sphere_ptr);
		return new_sphere_ptr;
	} else {
		return NULL;
	}
}

Object_Plane* Engine::create_plane_withlist(vec3f position, vec3f velocity, vec3f normale, string name, string aipath_filename, bool immaterial) {
	if(objets_list.size() < MAX_OBJECTS) {
		Object_Plane* new_plane_ptr = new Object_Plane(position, velocity, normale, name);
		update_collision_plane_withlist(new_plane_ptr);
		if(aipath_filename.size() != 0) {
			load_aipath_from_json(new_plane_ptr, aipath_filename);
		}
		new_plane_ptr->immaterial = immaterial;
		cout << "Creation de l'objet \"" << new_plane_ptr->name << "\" à T=" << elapsed_time << endl;
		objets_list.insert(objets_list.begin(), new_plane_ptr);
		return new_plane_ptr;
	} else {
		return NULL;
	}
}

Object_Polygon* Engine::create_polygon_withlist(vec3f position, vec3f velocity, vec3f normale, string name, string aipath_filename, bool immaterial) {
	if(objets_list.size() < MAX_OBJECTS) {
		Object_Polygon* new_polygon_ptr = new Object_Polygon(position, velocity, normale, name);
		new_polygon_ptr->polygon.add_side(vec3f(1,0,0));
		new_polygon_ptr->polygon.add_side(vec3f(0,1,0));
		new_polygon_ptr->polygon.add_side(vec3f(-1,0,0));
		new_polygon_ptr->polygon.add_side(vec3f(0,-1,0));
		update_collision_polygon_withlist(new_polygon_ptr);
		if(aipath_filename.size() != 0) {
			load_aipath_from_json(new_polygon_ptr, aipath_filename);
		}
		new_polygon_ptr->immaterial = immaterial;
		cout << "Creation de l'objet \"" << new_polygon_ptr->name << "\" à T=" << elapsed_time << endl;
		objets_list.insert(objets_list.begin(), new_polygon_ptr);
		return new_polygon_ptr;
	} else {
		return NULL;
	}
}

Projectile_Bullet* Engine::create_bullet_withlist(vec3f position, vec3f velocity, string name, bool is_affected_by_gravity, bool is_destroyed_on_contact) {
	if(projectiles_list.size() < MAX_PROJECTILES) {
		Projectile_Bullet* new_bullet_ptr = new Projectile_Bullet(position, velocity, name, is_affected_by_gravity);
		if(is_affected_by_gravity) {
			for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
				if((*it_event)->eventType == eventDiscretUpdate) {
					if((*it_event)->time_of_event == std::numeric_limits<double>::max()) {
						(*it_event)->time_of_event = elapsed_time;
					}
					new_bullet_ptr->granularity_debt = (*it_event)->time_of_event - elapsed_time;
					new_bullet_ptr->Point.velocity.z -= 0.5*gravity_cst*(new_bullet_ptr->granularity_debt);
					break;
				}
			}
		}
		new_bullet_ptr->is_destroyed_on_contact = is_destroyed_on_contact;
		projectiles_list.insert(projectiles_list.begin(), new_bullet_ptr);
		create_collision_projectile_withlist(new_bullet_ptr);
		cout << "Creation du bullet = \"" << new_bullet_ptr->name << "\" à T=" << elapsed_time << endl;
		return new_bullet_ptr;
	} else {
		return NULL;
	}
}

Projectile_Missile* Engine::create_missile_withlist(vec3f position, vec3f velocity, Object_virtual* target, double power, string name, bool is_affected_by_gravity, bool is_destroyed_on_contact) {
	if(projectiles_list.size() < MAX_PROJECTILES) {
		Projectile_Missile* new_missile_ptr = new Projectile_Missile(position, velocity, target, power, name, is_affected_by_gravity);
		for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
			if((*it_event)->eventType == eventDiscretUpdate) {
				if((*it_event)->time_of_event == std::numeric_limits<double>::max()) {
					(*it_event)->time_of_event = elapsed_time;
				}
				new_missile_ptr->granularity_debt = (*it_event)->time_of_event - elapsed_time;
				new_missile_ptr->Point.velocity.z -= 0.5*gravity_cst*(new_missile_ptr->granularity_debt);
				break;
			}
		}
		new_missile_ptr->is_destroyed_on_contact = is_destroyed_on_contact;
		projectiles_list.insert(projectiles_list.begin(), new_missile_ptr);
		create_collision_projectile_withlist(new_missile_ptr);
		cout << "Creation du missile = \"" << new_missile_ptr->name << "\" à T=" << elapsed_time << endl;
		return new_missile_ptr;
	} else {
		return NULL;
	}
}

bool Engine::erase_evenement_withlist(Event_virtual* event) {
	evenements_list.remove(event);
}

bool Engine::erase_projectile_withlist(Projectile_virtual* affected_projectile) {
	for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
		if((*it_event)->eventType == eventCollision) {
			Event_Collision* collision = (Event_Collision*)(*it_event);
			if(collision->affected_projectile == affected_projectile) {
				projectiles_list.remove(collision->affected_projectile);
				erase_evenement_withlist(*it_event);
				return true;
			}
		}
	}
	return false;
}

void Engine::update_collision_sphere_withlist(Object_Sphere* affected_object) {
	for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
		if((*it_event)->eventType == eventCollision) {
			Event_Collision* collision = (Event_Collision*)(*it_event);
			if(collision->affected_object == affected_object) { //Le projectile rentrait en collision avec l'objet modifié, on lui fait tout verifier
				update_collision_projectile_withlist(collision->affected_projectile, (*it_event));
			} else { //Le projectile ne rentrait pas en collision avec l'objet modifié, on verifie si c'est le cas maintenant
				double time_before_collision;
				if(LineContinuousCollisionSphere(&(collision->affected_projectile->Point), &(affected_object->englobing_sphere), &time_before_collision)) {
					if(elapsed_time + time_before_collision < collision->time_of_event) {
						collision->affected_object = affected_object;
						collision->time_of_event = elapsed_time + time_before_collision;
					}
				}
			}
		}
	}
}

void Engine::update_collision_plane_withlist(Object_Plane* affected_object) {
	for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
		if((*it_event)->eventType == eventCollision) {
			Event_Collision* collision = (Event_Collision*)(*it_event);
			if(collision->affected_object == affected_object) { //Le projectile rentrait en collision avec l'objet modifié, on lui fait tout verifier
				update_collision_projectile_withlist(collision->affected_projectile, (*it_event));
			} else { //Le projectile ne rentrait pas en collision avec l'objet modifié, on verifie si c'est le cas maintenant
				double time_before_collision;
				if(LineContinuousCollisionPlane(&(collision->affected_projectile->Point), &(affected_object->plane), &time_before_collision)) {
					if(elapsed_time + time_before_collision < collision->time_of_event) {
						collision->affected_object = affected_object;
						collision->time_of_event = elapsed_time + time_before_collision;
					}
				}
			}
		}
	}
}

void Engine::update_collision_polygon_withlist(Object_Polygon* affected_object) {
	for (list<Event_virtual*>::iterator it_event=evenements_list.begin(); it_event != evenements_list.end(); ++it_event) {
		if((*it_event)->eventType == eventCollision) {
			Event_Collision* collision = (Event_Collision*)(*it_event);
			if(collision->affected_object == affected_object) { //Le projectile rentrait en collision avec l'objet modifié, on lui fait tout verifier
				update_collision_projectile_withlist(collision->affected_projectile, (*it_event));
			} else { //Le projectile ne rentrait pas en collision avec l'objet modifié, on verifie si c'est le cas maintenant
				double time_before_collision;
				if(LineContinuousCollisionPolygon(&(collision->affected_projectile->Point), &(affected_object->polygon), &time_before_collision)) {
					if(elapsed_time + time_before_collision < collision->time_of_event) {
						collision->affected_object = affected_object;
						collision->time_of_event = elapsed_time + time_before_collision;
					}
				}
			}
		}
	}
}

void Engine::create_collision_projectile_withlist(Projectile_virtual* affected_projectile) {	
	Event_Collision* new_event = new Event_Collision();
	new_event->affected_projectile = affected_projectile;
	for (list<Object_virtual*>::iterator it_object=objets_list.begin(); it_object != objets_list.end(); ++it_object) {
		double time_before_collision;
		if((*(it_object))->objectType == objSphere) {
			if(LineContinuousCollisionSphere(&(affected_projectile->Point), &((*(it_object))->englobing_sphere), &time_before_collision)) {
				if(elapsed_time + time_before_collision < new_event->time_of_event) {
					new_event->affected_object = (*it_object);
					new_event->time_of_event = elapsed_time + time_before_collision;
				}
			}
		} else if((*(it_object))->objectType == objPlane) {
			Object_Plane* Plane = (Object_Plane*)(*it_object);
			if(LineContinuousCollisionPlane(&(affected_projectile->Point), &(Plane->plane), &time_before_collision)) {
				if(elapsed_time + time_before_collision < new_event->time_of_event) {
					new_event->affected_object = (*it_object);
					new_event->time_of_event = elapsed_time + time_before_collision;
				}
			}
		} else if((*(it_object))->objectType == objPolygon) {
			Object_Polygon* Polygon = (Object_Polygon*)(*it_object);
			if(LineContinuousCollisionPolygon(&(affected_projectile->Point), &(Polygon->polygon), &time_before_collision)) {
				if(elapsed_time + time_before_collision < new_event->time_of_event) {
					new_event->affected_object = (*it_object);
					new_event->time_of_event = elapsed_time + time_before_collision;
				}
			}
		}
	}
	evenements_list.insert(evenements_list.begin(), new_event);
}

void Engine::update_collision_projectile_withlist(Projectile_virtual* affected_projectile, Event_virtual* event) {
	Event_Collision* collision = (Event_Collision*)event;
	collision->time_of_event = std::numeric_limits<double>::max();
	for (list<Object_virtual*>::iterator it_object=objets_list.begin(); it_object != objets_list.end(); ++it_object) {
		double time_before_collision;
		if((*(it_object))->objectType == objSphere) {
			if(LineContinuousCollisionSphere(&(affected_projectile->Point), &((*(it_object))->englobing_sphere), &time_before_collision)) {
				if(elapsed_time + time_before_collision < collision->time_of_event) {
					collision->affected_object = (*it_object);
					collision->time_of_event = elapsed_time + time_before_collision;
				}
			}
		} else if((*(it_object))->objectType == objPlane) {
			Object_Plane* Plane = (Object_Plane*)(*it_object);
			if(LineContinuousCollisionPlane(&(affected_projectile->Point), &(Plane->plane), &time_before_collision)) {
				if(elapsed_time + time_before_collision < collision->time_of_event) {
					collision->affected_object = (*it_object);
					collision->time_of_event = elapsed_time + time_before_collision;
				}
			}
		} else if((*(it_object))->objectType == objPolygon) {
			Object_Polygon* Polygon = (Object_Polygon*)(*it_object);
			if(LineContinuousCollisionPolygon(&(affected_projectile->Point), &(Polygon->polygon), &time_before_collision)) {
				if(elapsed_time + time_before_collision < collision->time_of_event) {
					collision->affected_object = (*it_object);
					collision->time_of_event = elapsed_time + time_before_collision;
				}
			}
		}
	}
}

bool Engine::get_object_with_name(Object_virtual** concerned_object, string name) {
	*concerned_object = NULL;
	for (list<Object_virtual*>::iterator it_object=objets_list.begin(); it_object != objets_list.end(); ++it_object) {
		if((*it_object)->name == name) {
			*concerned_object = (*it_object);
			return true;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////FILE IO///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Engine::load_world_from_json(string filename) {
	using nlohmann::json;
	
	objets_list.clear();
	projectiles_list.clear();
	std::ifstream ifs(filename);
	if(ifs.fail()) {
		throw std::runtime_error("Error: de lecture du fichier " + filename);
	}
	json j;
	ifs >> j;
	ifs.close();
	
	for (json::iterator it_glob = j.begin(); it_glob != j.end(); ++it_glob) {
		if(it_glob.key() == "Objets") {
			for (json::iterator it_objetstype = it_glob.value().begin(); it_objetstype != it_glob.value().end(); ++it_objetstype) {
				if(it_objetstype.key() == "Sphere") {
					for (json::iterator it_sphere = it_objetstype.value().begin(); it_sphere != it_objetstype.value().end(); ++it_sphere) {
						vec3f position(0,0,0);
						vec3f velocity(0,0,0);
						double radius = 0;
						string name = "Unnamed";
						bool is_source_of_gravity = false;
						bool immaterial = false;
						string aipath_filename = "";
						for (json::iterator it_components = it_sphere.value().begin(); it_components != it_sphere.value().end(); ++it_components) {
							if(it_components.key() == "radius") {
								radius = double(it_components.value());
							} else if(it_components.key() == "position") {
								for (json::iterator it_position = it_components.value().begin(); it_position != it_components.value().end(); ++it_position) {
									if(it_position.key() == "x") {
										position.x = double(it_position.value());
									} else if(it_position.key() == "y") {
										position.y = double(it_position.value());
									} else if(it_position.key() == "z") {
										position.z = double(it_position.value());
									}
								}
							} else if(it_components.key() == "velocity") {
								for (json::iterator it_velocity = it_components.value().begin(); it_velocity != it_components.value().end(); ++it_velocity) {
									if(it_velocity.key() == "x") {
										velocity.x = double(it_velocity.value());
									} else if(it_velocity.key() == "y") {
										velocity.y = double(it_velocity.value());
									} else if(it_velocity.key() == "z") {
										velocity.z = double(it_velocity.value());
									}
								}
							} else if(it_components.key() == "name") {
								name = it_components.value();
							} else if(it_components.key() == "source_of_gravity") { //La position de l'objet va initialliser optim_gravity_source
								is_source_of_gravity = it_components.value();
							} else if(it_components.key() == "filepath") { //La position de l'objet va initialliser optim_gravity_source
								aipath_filename = it_components.value();
							} else if(it_components.key() == "immaterial") { //La position de l'objet va initialliser optim_gravity_source
								immaterial = it_components.value();
							}
						}
						if(is_source_of_gravity) {
							optim_gravity_source = position;
						}
						create_sphere_withlist(position, velocity, radius, name, aipath_filename, immaterial);
					}
				} else if(it_objetstype.key() == "Plane") {
					for (json::iterator it_plan = it_objetstype.value().begin(); it_plan != it_objetstype.value().end(); ++it_plan) {
						vec3f position(0,0,0);
						vec3f velocity(0,0,0);
						vec3f normale(0,0,0);
						string name = "Unnamed";
						bool is_source_of_gravity = false;
						bool immaterial = false;
						string aipath_filename = "";
						for (json::iterator it_components = it_plan.value().begin(); it_components != it_plan.value().end(); ++it_components) {
							if(it_components.key() == "position") {
								for (json::iterator it_position = it_components.value().begin(); it_position != it_components.value().end(); ++it_position) {
									if(it_position.key() == "x") {
										position.x = double(it_position.value());
									} else if(it_position.key() == "y") {
										position.y = double(it_position.value());
									} else if(it_position.key() == "z") {
										position.z = double(it_position.value());
									}
								}
							} else if(it_components.key() == "velocity") {
								for (json::iterator it_velocity = it_components.value().begin(); it_velocity != it_components.value().end(); ++it_velocity) {
									if(it_velocity.key() == "x") {
										velocity.x = double(it_velocity.value());
									} else if(it_velocity.key() == "y") {
										velocity.y = double(it_velocity.value());
									} else if(it_velocity.key() == "z") {
										velocity.z = double(it_velocity.value());
									}
								}
							} else if(it_components.key() == "normale") {
								for (json::iterator it_position = it_components.value().begin(); it_position != it_components.value().end(); ++it_position) {
									if(it_position.key() == "x") {
										normale.x = double(it_position.value());
									} else if(it_position.key() == "y") {
										normale.y = double(it_position.value());
									} else if(it_position.key() == "z") {
										normale.z = double(it_position.value());
									}
								}
							} else if(it_components.key() == "name") {
								name = it_components.value();
							} else if(it_components.key() == "filepath") { 
								aipath_filename = it_components.value();
							} else if(it_components.key() == "immaterial") {
								immaterial = it_components.value();
							}
						}
						create_plane_withlist(position, velocity, normale.normal(), name, aipath_filename, immaterial);
					}
				}
			}
		} else if(it_glob.key() == "Projectiles") {
			for (json::iterator it_projectiles = it_glob.value().begin(); it_projectiles != it_glob.value().end(); ++it_projectiles) {
				if(it_projectiles.key() == "Bullet") {
					for (json::iterator it_bullet = it_projectiles.value().begin(); it_bullet != it_projectiles.value().end(); ++it_bullet) {
						vec3f position(0,0,0);
						vec3f velocity(0,0,0);
						string name = "Unnamed";
						bool is_affected_by_gravity = false;
						bool is_destroyed_on_contact = false;
						for (json::iterator it_components = it_bullet.value().begin(); it_components != it_bullet.value().end(); ++it_components) {
							if(it_components.key() == "position") {
								for (json::iterator it_position = it_components.value().begin(); it_position != it_components.value().end(); ++it_position) {
									if(it_position.key() == "x") {
										position.x = double(it_position.value());
									} else if(it_position.key() == "y") {
										position.y = double(it_position.value());
									} else if(it_position.key() == "z") {
										position.z = double(it_position.value());
									}
								}
							} else if(it_components.key() == "velocity") {
								for (json::iterator it_velocity = it_components.value().begin(); it_velocity != it_components.value().end(); ++it_velocity) {
									if(it_velocity.key() == "x") {
										velocity.x = double(it_velocity.value());
									} else if(it_velocity.key() == "y") {
										velocity.y = double(it_velocity.value());
									} else if(it_velocity.key() == "z") {
										velocity.z = double(it_velocity.value());
									}
								}
							} else if(it_components.key() == "name") {
								name = it_components.value();
							} else if(it_components.key() == "affected_by_gravity") {
								is_affected_by_gravity = it_components.value();
							} else if(it_components.key() == "destroyed_on_contact") {
								is_destroyed_on_contact = it_components.value();
							}
						}
						create_bullet_withlist(position, velocity, name, is_affected_by_gravity, is_destroyed_on_contact);
					}
				} else if(it_projectiles.key() == "Missile") {
					for (json::iterator it_missile = it_projectiles.value().begin(); it_missile != it_projectiles.value().end(); ++it_missile) {
						vec3f position(0,0,0);
						vec3f velocity(0,0,0);
						string name = "Unnamed";
						bool is_affected_by_gravity = false;
						bool is_destroyed_on_contact = false;
						Object_virtual* target = NULL;
						double power = 0;
						for (json::iterator it_components = it_missile.value().begin(); it_components != it_missile.value().end(); ++it_components) {
							if(it_components.key() == "position") {
								for (json::iterator it_position = it_components.value().begin(); it_position != it_components.value().end(); ++it_position) {
									if(it_position.key() == "x") {
										position.x = double(it_position.value());
									} else if(it_position.key() == "y") {
										position.y = double(it_position.value());
									} else if(it_position.key() == "z") {
										position.z = double(it_position.value());
									}
								}
							} else if(it_components.key() == "velocity") {
								for (json::iterator it_velocity = it_components.value().begin(); it_velocity != it_components.value().end(); ++it_velocity) {
									if(it_velocity.key() == "x") {
										velocity.x = double(it_velocity.value());
									} else if(it_velocity.key() == "y") {
										velocity.y = double(it_velocity.value());
									} else if(it_velocity.key() == "z") {
										velocity.z = double(it_velocity.value());
									}
								}
							} else if(it_components.key() == "name") {
								name = it_components.value();
							} else if(it_components.key() == "affected_by_gravity") {
								is_affected_by_gravity = it_components.value();
							} else if(it_components.key() == "destroyed_on_contact") {
								is_destroyed_on_contact = it_components.value();
							} else if(it_components.key() == "target") {
								get_object_with_name(&target, it_components.value());
							} else if(it_components.key() == "power") {
								power = double(it_components.value());
							}
						}
						create_missile_withlist(position, velocity, target, power, name, is_affected_by_gravity, is_destroyed_on_contact);
					}
				}
			}
		}
	}
	elapsed_time = 0;
}

void Engine::load_config_from_json(string config_filename) {
	using nlohmann::json;
	
	objets_list.clear();
	projectiles_list.clear();
	evenements_list.clear();
	
	std::ifstream ifs(config_filename);
	if(ifs.fail()) {
		throw std::runtime_error("Error: de lecture du fichier " + config_filename);
	}
	json j;
	ifs >> j;
	ifs.close();
	
	double Discrete_update_delay = 1.0;
	string world_file = "";
	bool gravity = false;
	
	for (json::iterator it_glob = j.begin(); it_glob != j.end(); ++it_glob) {
		if(it_glob.key() == "Discrete_update_delay") {
			Discrete_update_delay = it_glob.value();
		} else if(it_glob.key() == "world_file") {
			world_file = it_glob.value();
		} else if(it_glob.key() == "gravity") {
			gravity = it_glob.value();
		} else if(it_glob.key() == "gravity_cst") {
			gravity_cst = it_glob.value();
		}
	}
	elapsed_time = 0;
	
	Event_Discret_Update* event = new Event_Discret_Update();
	event->update_delay = Discrete_update_delay;
	event->time_of_event = std::numeric_limits<double>::max();
	optim_granularity_event_active = false;
	evenements_list.insert(evenements_list.begin(), event);
	
	if(world_file.size() != 0) {
		load_world_from_json(world_file);
	}
}

void Engine::load_aipath_from_json(Object_virtual* affected_object, string aipath_filename) {
	using nlohmann::json;
	
	std::ifstream ifs(aipath_filename);
	if(ifs.fail()) {
		throw std::runtime_error("Error: de lecture du fichier " + aipath_filename);
	}
	json j;
	ifs >> j;
	ifs.close();
	
	double Discrete_update_delay = 1.0;
	string world_file = "";
	bool gravity = false;
	
	double time_counter = elapsed_time;
	
	for (json::iterator it_glob = j.begin(); it_glob != j.end(); ++it_glob) {
		Event_Path_Change* new_event = new Event_Path_Change();
		new_event->affected_object = affected_object;
		for (json::iterator it_component = it_glob.value().begin(); it_component != it_glob.value().end(); ++it_component) {
			if(it_component.key() == "start_after") {
				new_event->time_of_event = time_counter + (double)it_component.value();
				time_counter = time_counter + (double)it_component.value();
			} else if(it_component.key() == "vx") {
				new_event->new_velocity.x = it_component.value();
			} else if(it_component.key() == "vy") {
				new_event->new_velocity.y = it_component.value();
			} else if(it_component.key() == "vz") {
				new_event->new_velocity.z = it_component.value();
			}
		}
		evenements_list.insert(evenements_list.begin(), new_event);
	}
}