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
	double duration = 0.0;
	
	//Basic vector operation tests
	try {
		
		vec3f point1(8.0, 1.7, -10.0);
		vec3f point2(-6.1, 4.6, -0.2);
		double dot_product = point1.dot(point2);
		double expected_result = -38.98;
		if(IsCloseTodouble(dot_product, expected_result, 0.000001)){
			cout << "Dot product test [success]. " << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				point1.dot(point2);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of a call : " << duration << "ns." << endl;
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
		vec3f normal = ClosestPointOnLine(l1, lvec, p);
		vec3f optim_normal = optim_ClosestPointOnLine(l1, lvec, p);
		vec3f expected_result = vec3f(-510.256,392.516,-26.992);
		if(IsCloseTo(normal, expected_result, 0.000001) && IsCloseTo(normal,optim_normal,0.000001)){
			cout << "ClosestPointOnLine test [success]. " << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				ClosestPointOnLine(l1, lvec, p);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of a call : " << duration << "ns." << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				optim_ClosestPointOnLine(l1, lvec, p);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of an optimized call : " << duration << "ns." << endl;
		} else {
			stringstream error;
			error << "ClosestPointOnLine test [failed] : expected " << expected_result.print() << ", got " << normal.print() << endl;
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
		double sphere_radius = 7.0;
		double sphere_radius_squared = 7.0*7.0;
		bool normal = LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, sphere_radius);
		bool already_inside_sphere = LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, 50.0);
		bool wrong_way = LineIsGoingToCollideSphere(l1, vec3f(0,0,0)-line_vec_norm, sphere_center, sphere_radius);
		bool optim_normal = optim_LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, sphere_radius_squared);
		bool optim_already_inside_sphere = optim_LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, 50.0*50.0);
		bool optim_wrong_way = optim_LineIsGoingToCollideSphere(l1, vec3f(0,0,0)-line_vec_norm, sphere_center, sphere_radius_squared);
		bool expected_result_normal = true;
		bool expected_result_already_inside_sphere = false;
		bool expected_result_wrong_way = false;
		if(normal == expected_result_normal && already_inside_sphere == expected_result_already_inside_sphere && wrong_way == expected_result_wrong_way && optim_normal == expected_result_normal && optim_already_inside_sphere == expected_result_already_inside_sphere && optim_wrong_way == expected_result_wrong_way){
			cout << "LineIsGoingToCollideSphere test [success]. " << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, sphere_radius);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of a call : " << duration << "ns." << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				optim_LineIsGoingToCollideSphere(l1, line_vec_norm, sphere_center, sphere_radius_squared);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of an optimized call : " << duration << "ns." << endl;
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
		
		double optim_normal_time = 0;
		bool optim_normal = optim_LineContinuousCollisionSphere(&Projectile, &Sphere, &optim_normal_time);
		double optim_already_inside_sphere_time = 0;
		Shape_Point optim_already_inside_Projectile(vec3f(0.0, 5.0, 0.0), vec3f(0.1, -2.0, 0.0));
		bool optim_already_inside_sphere = optim_LineContinuousCollisionSphere(&already_inside_Projectile, &Sphere, &optim_already_inside_sphere_time);
		double optim_wrong_way_time = 0;
		Shape_Point optim_wrong_way_Projectile(vec3f(5.0, 25.0, 0.0), vec3f(0.0, 4.0, 0.0));
		bool optim_wrong_way = optim_LineContinuousCollisionSphere(&wrong_way_Projectile, &Sphere, &optim_wrong_way_time);
		
		double expected_normal_time = 3.88291;
		bool expected_normal = true;
		bool expected_already_inside_sphere = false;
		bool expected_wrong_way = false;

		//if(IsCloseTodouble(normal_time, expected_normal_time, 0.00001) && normal == expected_normal && already_inside_sphere == expected_already_inside_sphere && wrong_way == expected_wrong_way && IsCloseTodouble(optim_normal_time, normal_time, 0.00001) && normal == optim_normal && already_inside_sphere == optim_already_inside_sphere && wrong_way == optim_wrong_way)
		{
			cout << "LineContinuousCollisionSphere test [success]. " << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				LineContinuousCollisionSphere(&Projectile, &Sphere, &normal_time);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of a call : " << duration << "ns." << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				optim_LineContinuousCollisionSphere(&Projectile, &Sphere, &normal_time);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of an optimized call : " << duration << "ns." << endl;
		}/* else {
			stringstream error;
			error << "LineContinuousCollisionSphere test [failed]." << endl;
			throw std::runtime_error(error.str());
		}*/
	} catch (exception& e) {
		cout << e.what() << endl;
	}
	//LineContinuousCollisionPlane
	try {
		Shape_Plane Plane(vec3f(3.0, 5.0, 0.0), vec3f(0.0, 0.0, 0.0), vec3f(0.0, 0.0, 1.0));
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
		Shape_Point parallel_Projectile(vec3f(5.0, 25.0, 50.0), vec3f(5.0, 0.0, 0.0));
		bool parallel = LineContinuousCollisionPlane(&parallel_Projectile, &Plane, &wrong_way_2_time);
		
		double optim_normal_time = 0;
		bool optim_normal = optim_LineContinuousCollisionPlane(&Projectile, &Plane, &optim_normal_time);
		double optim_wrong_way_time = 0;
		Shape_Point optim_wrong_way_Projectile(vec3f(5.0, 25.0, 50.0), vec3f(0.0, 0.0, 5.0));
		bool optim_wrong_way = optim_LineContinuousCollisionPlane(&wrong_way_Projectile, &Plane, &optim_wrong_way_time);
		double optim_wrong_way_2_time = 0;
		Shape_Point optim_wrong_way_2_Projectile(vec3f(5.0, 25.0, -50.0), vec3f(0.0, 0.0, 5.0));
		bool optim_wrong_way_2 = optim_LineContinuousCollisionPlane(&wrong_way_2_Projectile, &Plane, &optim_wrong_way_2_time);
		double optim_parallel_time = 0;
		Shape_Point optim_parallel_Projectile(vec3f(5.0, 25.0, 50.0), vec3f(5.0, 0.0, 0.0));
		bool optim_parallel = optim_LineContinuousCollisionPlane(&optim_parallel_Projectile, &Plane, &optim_parallel_time);
		
		double expected_normal_time = 15.625;
		bool expected_normal = true;
		bool expected_wrong_way = false;
		bool expected_wrong_way_2 = false;
		bool expected_parallel = false;
		
		
		if(IsCloseTodouble(normal_time, expected_normal_time, 0.00001) && normal == expected_normal && wrong_way == expected_wrong_way && wrong_way_2 == expected_wrong_way_2 && parallel == expected_parallel && IsCloseTodouble(normal_time, optim_normal_time, 0.00001) && normal == optim_normal && wrong_way == optim_wrong_way && wrong_way_2 == optim_wrong_way_2 && parallel == optim_parallel)
		{
			cout << "LineContinuousCollisionPlane test [success]. " << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				LineContinuousCollisionPlane(&Projectile, &Plane, &normal_time);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of a call : " << duration << "ns." << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){	
				optim_LineContinuousCollisionPlane(&Projectile, &Plane, &normal_time);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of an optimized call : " << duration << "ns." << endl;
		} else {
			stringstream error;
			error << "LineContinuousCollisionPlane test [failed]." << endl;
			throw std::runtime_error(error.str());
		}
	} catch (exception& e) {
		cout << e.what() << endl;
	}
	//LineContinuousCollisionPolygon
	try {
		Shape_Polygon Polygon(vec3f(0.0, 0.0, 0.0), vec3f(0.0, 0.0, 0.0), vec3f(0.0, 0.0, 1.0));
		Polygon.add_side(vec3f(1,0,0));
		Polygon.add_side(vec3f(0,1,0));
		Polygon.add_side(vec3f(-1,0,0));
		Polygon.add_side(vec3f(0,-1,0));
		
		Shape_Point Projectile(vec3f(0.0, 0.0, 50.0), vec3f(0.0, 0.0, -3.2));
		double normal_time = 0;
		bool normal = LineContinuousCollisionPolygon(&Projectile, &Polygon, &normal_time);
		double wrong_way_time = 0;
		Shape_Point wrong_way_Projectile(vec3f(0.0, 0.0, 50.0), vec3f(0.0, 0.0, 3.6));
		bool wrong_way = LineContinuousCollisionPolygon(&wrong_way_Projectile, &Polygon, &wrong_way_time);
		double wrong_way_2_time = 0;
		Shape_Point wrong_way_2_Projectile(vec3f(0.0, 0.0, -50.0), vec3f(0.0, 0.0, 5.0));
		bool wrong_way_2 = LineContinuousCollisionPolygon(&wrong_way_2_Projectile, &Polygon, &wrong_way_2_time);
		double parallel_time = 0;
		Shape_Point parallel_Projectile(vec3f(0.0, 0.0, 50.0), vec3f(50.0, 0.0, 0.0));
		bool parallel = LineContinuousCollisionPolygon(&parallel_Projectile, &Polygon, &wrong_way_2_time);
		double outside_time = 0;
		Shape_Point outside_Projectile(vec3f(0.0, 0.0, 50.0), vec3f(1.0, 0.0, -3.2));
		bool outside = LineContinuousCollisionPolygon(&parallel_Projectile, &Polygon, &outside_time);
		
		double optim_normal_time = 0;
		bool optim_normal = optim_LineContinuousCollisionPolygon(&Projectile, &Polygon, &optim_normal_time);
		double optim_wrong_way_time = 0;
		Shape_Point optim_wrong_way_Projectile(vec3f(5.0, 25.0, 50.0), vec3f(0.0, 0.0, 5.0));
		bool optim_wrong_way = optim_LineContinuousCollisionPolygon(&wrong_way_Projectile, &Polygon, &optim_wrong_way_time);
		double optim_wrong_way_2_time = 0;
		Shape_Point optim_wrong_way_2_Projectile(vec3f(5.0, 25.0, -50.0), vec3f(0.0, 0.0, 5.0));
		bool optim_wrong_way_2 = optim_LineContinuousCollisionPolygon(&wrong_way_2_Projectile, &Polygon, &optim_wrong_way_2_time);
		double optim_parallel_time = 0;
		Shape_Point optim_parallel_Projectile(vec3f(5.0, 25.0, 0.0), vec3f(0.0, 0.0, 5.0));
		bool optim_parallel = optim_LineContinuousCollisionPolygon(&wrong_way_2_Projectile, &Polygon, &optim_parallel_time);
		double optim_outside_time = 0;
		Shape_Point optim_outside_Projectile(vec3f(0.0, 0.0, 50.0), vec3f(1.0, 0.0, -3.2));
		bool optim_outside = optim_LineContinuousCollisionPolygon(&parallel_Projectile, &Polygon, &outside_time);
		
		double expected_normal_time = 15.625;
		bool expected_normal = true;
		bool expected_wrong_way = false;
		bool expected_wrong_way_2 = false;
		bool expected_parallel = false;
		bool expected_outside = false;
		
		
		if(IsCloseTodouble(normal_time, expected_normal_time, 0.00001) && normal == expected_normal && wrong_way == expected_wrong_way && wrong_way_2 == expected_wrong_way_2 && parallel == expected_parallel && outside == expected_outside && IsCloseTodouble(normal_time, optim_normal_time, 0.00001) && normal == optim_normal && wrong_way == optim_wrong_way && wrong_way_2 == optim_wrong_way_2 && parallel == optim_parallel && outside == optim_outside)
		{
			cout << "LineContinuousCollisionPolygon test [success]. " << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){
				LineContinuousCollisionPolygon(&Projectile, &Polygon, &normal_time);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of a call : " << duration << "ns." << endl;
			Start = chrono::system_clock::now();
			for(int i=0 ; i<iteration_number ; i++){
				optim_LineContinuousCollisionPolygon(&Projectile, &Polygon, &normal_time);
			}
			Stop = chrono::system_clock::now();
			duration = chrono::duration_cast<chrono::nanoseconds>(Stop - Start).count() / (double)iteration_number;
			cout << "\tAverage time of an optimized call : " << duration << "ns." << endl;
		} else {
			stringstream error;
			error << "LineContinuousCollisionPolygon test [failed]." << endl;
			throw std::runtime_error(error.str());
		}
	} catch (exception& e) {
		cout << e.what() << endl;
	}
	return 0;
}
