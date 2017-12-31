#ifndef __3DENGINE_TESTBENCH_MAIN_H__
#define __3DENGINE_TESTBENCH_MAIN_H__

//General libraries
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
//sleep/time measure
#include <chrono>
#include <thread>

//Local headers
#include "vec3f.h" //Vectors operations
#include "geometric_shapes.h" //Various shapes class
#include "physics.h" //Interactions between shapes
#include "world_objects.h" //Objects used in the Engine class
#include "engine.h" //Combine Objects and Interactions

#define IMG_X 400
#define IMG_Y 800
#define ENV_X 50
#define ENV_Y 100

using namespace std;


bool IsCloseTodouble(double v1, double v2, double error_margin);
bool IsCloseTo(vec3f point1, vec3f point2, double error_margin);

#endif