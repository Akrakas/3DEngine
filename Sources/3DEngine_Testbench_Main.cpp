#include "3DEngine_Testbench_Main.h"

int mousex = 0;
int mousey = 0;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
		if  ( event == cv::EVENT_LBUTTONDOWN ) {
		}
		else if  ( event == cv::EVENT_RBUTTONDOWN ) {
		}
		else if  ( event == cv::EVENT_MBUTTONDOWN ){
		}
		else if ( event == cv::EVENT_MOUSEMOVE ){
		mousex = x;
		mousey = y;
     }
}

bool IsCloseTodouble(double v1, double v2) {
	if(v1 - v2 < 0.001 && v1 - v2 > -0.001) return true;
	else return false;
}

bool IsCloseTo(vec3f point1, vec3f point2) {
	if(IsCloseTodouble(point1.x, point2.x) && IsCloseTodouble(point1.y, point2.y), IsCloseTodouble(point1.z, point2.z)) return true;
	else return false;
}

void engine_show(Engine* physic_engine, cv::Mat* image, double dx, double dy) {
	for (list<Object_virtual*>::iterator it_object=physic_engine->objets_list.begin(); it_object != physic_engine->objets_list.end(); ++it_object) {
		if(((*it_object))->objectType == objSphere) {
			Object_Sphere* current_sphere = (Object_Sphere*)(*it_object);
			double zpos = min(max((current_sphere->englobing_sphere.position.z+127)/255.0, 0.0), 1.0);
			cv::circle(*image, cv::Point(current_sphere->englobing_sphere.position.x*dx, current_sphere->englobing_sphere.position.y*dy), current_sphere->englobing_sphere.radius*dx, cv::Scalar( 0*zpos, 127*zpos, 0*zpos ), 3, 8 );
			cv::line( *image, cv::Point(current_sphere->englobing_sphere.position.x*dx,current_sphere->englobing_sphere.position.y*dy), cv::Point((current_sphere->englobing_sphere.position.x+current_sphere->englobing_sphere.velocity.x)*dx,(current_sphere->englobing_sphere.position.y+current_sphere->englobing_sphere.velocity.y)*dy), cv::Scalar( 127*zpos, 127*zpos, 127*zpos ), 3, 8 );
		}
	}
	for (list<Projectile_Bullet*>::iterator it_projectile=physic_engine->projectiles_list.begin(); it_projectile != physic_engine->projectiles_list.end(); ++it_projectile) {
		Projectile_Bullet* current_Projectile = (*it_projectile);
		double zpos = min(max((current_Projectile->Point.position.z+127)/255.0, 0.0), 1.0);
		cv::circle(*image, cv::Point(current_Projectile->Point.position.x*dx, current_Projectile->Point.position.y*dy), 1, cv::Scalar( 0*zpos, 127*zpos, 0*zpos ), 3, 8 );
		cv::line( *image, cv::Point(current_Projectile->Point.position.x*dx,current_Projectile->Point.position.y*dy), cv::Point((current_Projectile->Point.position.x+current_Projectile->Point.velocity.x)*dx,(current_Projectile->Point.position.y+current_Projectile->Point.velocity.y)*dy), cv::Scalar( 127*zpos, 127*zpos, 127*zpos ), 3, 8 );
	}
}

int main() {	
	cv::namedWindow("TestBench", 1);
	cv::setMouseCallback("TestBench", CallBackFunc, NULL);
	double dx = (double)IMG_X/ENV_X;
	double dy = (double)IMG_Y/ENV_Y;
	
	Engine physic_engine("../Config/config.json");
	while(1) {
		cv::Mat testbench_img = cv::Mat::zeros( ENV_X*dx, ENV_Y*dy, CV_8UC3 );
		physic_engine.step_withlist(0.2);
		cout << "elapsed_time = " << physic_engine.elapsed_time << endl;
		engine_show(&physic_engine, &testbench_img, dx, dy);
		cv::imshow("TestBench", testbench_img);
		cv::waitKey(200);
	}
	
	return 0;
}