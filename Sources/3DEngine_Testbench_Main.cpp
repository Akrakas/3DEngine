#include "3DEngine_Testbench_Main.h"


bool IsCloseTodouble(double v1, double v2, double error_margin) {
	if(v1 - v2 < error_margin && v1 - v2 > -error_margin) return true;
	else return false;
}

bool IsCloseTo(vec3f point1, vec3f point2, double error_margin) {
	if(IsCloseTodouble(point1.x, point2.x, error_margin) && IsCloseTodouble(point1.y, point2.y, error_margin), IsCloseTodouble(point1.z, point2.z, error_margin)) return true;
	else return false;
}

int main() {
	chrono::time_point<chrono::system_clock> Start, Stop;
	int iteration_number = 10000000;
	
	//Basic vector operation tests
	try {
		
		vec3f point1(8.0, 1.7, -10.0);
		vec3f point2(-6.1, 4.6, -0.2);
		double dot_product = point1.dot(point2);
		double expected_result = -38.98;
		if(IsCloseTodouble(dot_product, expected_result, 0.000001)){
			cout << "Dot product test [success]. ";
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				point1.dot(point2);
			}
			Stop = chrono::system_clock::now();
			int duration = chrono::duration_cast<chrono::milliseconds>(Stop - Start).count();
			cout << "Time to run " << iteration_number << " iterations : " << duration << "ms." << endl;
		} else {
			stringstream error;
			error << "Dot product test [failed] : expected " << expected_result << ", got " << dot_product << endl;
			throw std::runtime_error(error.str());
		}
	} catch (exception& e) {
		cout << e.what() << endl;
	}
	
	//Basic physic tests
	//ClosestPointOnLine
	try {
		
		vec3f l1(8.0, 1.7, -10.0);
		vec3f lvec(-6.1, 4.6, -0.2);
		vec3f p(-1.2,8.8,9.1);
		vec3f closest_point = ClosestPointOnLine(l1, lvec, p);
		vec3f expected_result = vec3f(-510.256,392.516,-26.992);
		if(IsCloseTo(closest_point, expected_result, 0.000001)){
			cout << "ClosestPointOnLine test [success]. ";
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				ClosestPointOnLine(l1, lvec, p);
			}
			Stop = chrono::system_clock::now();
			int duration = chrono::duration_cast<chrono::milliseconds>(Stop - Start).count();
			cout << "Time to run " << iteration_number << " iterations : " << duration << "ms." << endl;
		} else {
			stringstream error;
			error << "ClosestPointOnLine test [failed] : expected " << expected_result.print() << ", got " << closest_point.print() << endl;
			throw std::runtime_error(error.str());
		}
	} catch (exception& e) {
		cout << e.what() << endl;
	}
	//Basic physic tests
	//LineIsGoingToCollideSphere
	try {
		
		vec3f l1(10.0, 25.0, 0.0);
		vec3f line_vec_norm(-1.0, 0.0, 0.0);
		line_vec_norm.normalize();
		vec3f sphere_center(0.0,20.0,0.0);
		double sphere_radius = 1;
		bool normal = LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, 7.0);
		bool already_inside_sphere = LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, 50.0);
		bool wrong_way = LineIsGoingToCollideSphere(l1, vec3f(0,0,0)-line_vec_norm, sphere_center, 7.0);
		bool expected_result_normal = true;
		bool expected_result_already_inside_sphere = false;
		bool expected_result_wrong_way = false;
		if(normal == expected_result_normal && already_inside_sphere == expected_result_already_inside_sphere && wrong_way == expected_result_wrong_way){
			cout << "LineIsGoingToCollideSphere test [success]. ";
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, 7.0);
			}
			Stop = chrono::system_clock::now();
			int duration = chrono::duration_cast<chrono::milliseconds>(Stop - Start).count();
			cout << "Time to run " << iteration_number << " iterations : " << duration << "ms." << endl;
		} else {
			stringstream error;
			error << "LineIsGoingToCollideSphere test [failed]." << endl;
			throw std::runtime_error(error.str());
		}
	} catch (exception& e) {
		cout << e.what() << endl;
	}
	//LineContinuousCollisionSphere
	try {
		Shape_Point Projectile(vec3f(5.0, 25.0, 0.0), vec3f(0.1, -2.0, 0.0));
		Shape_Sphere Sphere(vec3f(0.0, 5.0, 0.0), vec3f(0.0, 2.0, 0.0), 7.0);
		double normal_time = 0;
		bool normal = LineContinuousCollisionSphere(&Projectile, &Sphere, &normal_time);
		double already_inside_sphere_time = 0;
		Shape_Point already_inside_Projectile(vec3f(0.0, 5.0, 0.0), vec3f(0.1, -2.0, 0.0));
		bool already_inside_sphere = LineContinuousCollisionSphere(&already_inside_Projectile, &Sphere, &already_inside_sphere_time);
		double wrong_way_time = 0;
		Shape_Point wrong_way_Projectile(vec3f(5.0, 25.0, 0.0), vec3f(0.0, 4.0, 0.0));
		bool wrong_way = LineContinuousCollisionSphere(&wrong_way_Projectile, &Sphere, &wrong_way_time);
		double expected_normal_time = 3.88291;
		bool expected_normal = true;
		bool expected_already_inside_sphere = false;
		bool expected_wrong_way = false;
		if(IsCloseTodouble(normal_time, expected_normal_time, 0.00001) && normal == expected_normal && already_inside_sphere == expected_already_inside_sphere && wrong_way == expected_wrong_way)
		{
			cout << "LineContinuousCollisionSphere test [success]. ";
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				LineContinuousCollisionSphere(&Projectile, &Sphere, &normal_time);
			}
			Stop = chrono::system_clock::now();
			int duration = chrono::duration_cast<chrono::milliseconds>(Stop - Start).count();
			cout << "Time to run " << iteration_number << " iterations : " << duration << "ms." << endl;
		} else {
			stringstream error;
			error << "LineContinuousCollisionSphere test [failed]." << endl;
			throw std::runtime_error(error.str());
		}
	} catch (exception& e) {
		cout << e.what() << endl;
	}
	//LineContinuousCollisionPlane
	try {
		Shape_Plane Plane(vec3f (3.0, 5.0, 0.0), vec3f(0.0, 0.0, 0.0), vec3f(0.0, 0.0, 1.0));
		Shape_Point Projectile(vec3f(5.0, 25.0, 50.0), vec3f(0.1, -2.0, -3.2));
		double normal_time = 0;
		bool normal = LineContinuousCollisionPlane(&Projectile, &Plane, &normal_time);
		double wrong_way_time = 0;
		Shape_Point wrong_way_Projectile(vec3f(5.0, 25.0, 50.0), vec3f(0.0, 0.0, 5.0));
		bool wrong_way = LineContinuousCollisionPlane(&wrong_way_Projectile, &Plane, &wrong_way_time);
		double wrong_way_2_time = 0;
		Shape_Point wrong_way_2_Projectile(vec3f(5.0, 25.0, -50.0), vec3f(0.0, 0.0, 5.0));
		bool wrong_way_2 = LineContinuousCollisionPlane(&wrong_way_2_Projectile, &Plane, &wrong_way_2_time);
		double parallel_time = 0;
		Shape_Point parallel_Projectile(vec3f(5.0, 25.0, 0.0), vec3f(0.0, 0.0, 5.0));
		bool parallel = LineContinuousCollisionPlane(&wrong_way_2_Projectile, &Plane, &wrong_way_2_time);
		double expected_normal_time = 15.625;
		bool expected_normal = true;
		bool expected_wrong_way = false;
		bool expected_wrong_way_2 = false;
		bool expected_parallel = false;
		if(IsCloseTodouble(normal_time, expected_normal_time, 0.00001) && normal == expected_normal && wrong_way == expected_wrong_way && wrong_way_2 == expected_wrong_way_2 && parallel == expected_parallel)
		{
			cout << "LineContinuousCollisionPlane test [success]. ";
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				LineContinuousCollisionPlane(&Projectile, &Plane, &normal_time);
			}
			Stop = chrono::system_clock::now();
			int duration = chrono::duration_cast<chrono::milliseconds>(Stop - Start).count();
			cout << "Time to run " << iteration_number << " iterations : " << duration << "ms." << endl;
		} else {
			stringstream error;
			error << "LineContinuousCollisionPlane test [failed]." << endl;
			throw std::runtime_error(error.str());
		}
	} catch (exception& e) {
		cout << e.what() << endl;
	}
	return 0;
}
