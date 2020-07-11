#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <vector>
#include<cmath>

using namespace std;



class  coord2d {
	double x;
	double y;
public:
	coord2d();
	coord2d(double init);
	coord2d(double init_x, double init_y);
	double getx() { return x; };
	double gety() { return y; };
	void setx(double x);
	void sety(double y);
	coord2d operator+(coord2d ob2);
	coord2d operator-(coord2d ob2);

};
void coord2d::setx(double x)
{
	this->x = x;
}
void coord2d::sety(double y)
{
	this->y = y;
}

coord2d::coord2d() {
	x = 0.0;
	y = 0.0;
}
coord2d::coord2d(double init) {
	x = init;
	y = init;
}
coord2d::coord2d(double init_x, double init_y) {
	x = init_x;
	y = init_y;
}
coord2d coord2d::operator+(coord2d ob2)
{
	return coord2d(
		this->getx() + ob2.getx(),
		this->gety() + ob2.gety()
	);
}
coord2d coord2d::operator-(coord2d ob2)
{
	return coord2d(
		this->getx() - ob2.getx(),
		this->gety() - ob2.gety()
	);
}
/* ãÏìÆéïé‘ÇÃçÏê¨ */
double calcPerimeter_driving(double dist_of_center, double shrink, int div_num);
void calcToothAnglesAndPoints_driving(double dist_of_center, double shrink, double total_perimeter, int tooth_num, int div_num, vector<double>& tooth_angles, vector<coord2d>& tooth_points);
void storeGear_driving(double module, int tooth_num, double pressure_angle, vector<double>& tooth_angles, vector<coord2d>& tooth_points, vector<coord2d>& gear_plots);
void drawGear_driving(vector<coord2d>& gear_plots, double offset_x);
void design_driving_gear(double dist_of_center, double coefficient, double module, double pressure_angle, int tooth_num, int div_num);

/* è]ìÆéïé‘ÇÃçÏê¨ */
double calcPerimeter_driven(double dist_of_center, double shrink, int div_num);
void calcToothAnglesAndPoints_driven(double dist_of_center, double shrink, double total_perimeter, int tooth_num, int div_num, vector<double>& tooth_angles, vector<coord2d>& tooth_points);
void storeGear_driven(double module, int tooth_num, double pressure_angle, vector<double>& tooth_angles, vector<coord2d>& tooth_points, vector<coord2d>& gear_plots);
void drawGear_driven(vector<coord2d>& gear_plots, double offset_x);
void design_driven_gear(double dist_of_center, double coefficient, double module, double pressure_angle, int tooth_num, int div_num);

void thooth_drawing(double img_std_diameter, double module, double pressure_angle, coord2d vct, vector<coord2d>& plots);
coord2d involute_drawing(double angle, double pressure_angle, double start_angle, double std_diameter);
double TT_to_Angle(double std_diameter, double module, double pressure_angle, double target_diameter);
double inv(double angle);
coord2d circle_drawing(double angle, double module, double std_diameter);
void paramove(double dx, double dy, vector<coord2d>& plots);
void rotmove(double theta, vector<coord2d>& plots);
void mirroring_y(vector<coord2d>& plots);
double Speed(const double shrink, const double angle);
double DrivingGear_rad(const double perimeter, const double speed);
double DrivenGear_rad(const double perimeter, const double speed);
coord2d calc_center(coord2d& p1, coord2d& p2, coord2d& p3);
double distance(coord2d p1, coord2d p2);