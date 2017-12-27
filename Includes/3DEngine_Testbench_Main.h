#ifndef __3DENGINE_TESTBENCH_MAIN_H__
#define __3DENGINE_TESTBENCH_MAIN_H__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "vec3f.h"
#include "geometric_shapes.h"
#include "physics.h"
#include "world_objects.h"
#include "engine.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;

void CallBackFunc(int event, int x, int y, int flags, void* userdata);
bool IsCloseTodouble(double v1, double v2);
bool IsCloseTo(vec3f point1, vec3f point2);
void engine_show(Engine physic_engine, cv::Mat image, double dx, double dy);

#define IMG_X 400
#define IMG_Y 800
#define ENV_X 50
#define ENV_Y 100



#endif