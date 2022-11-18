#ifndef POINT_BASE_H
#define POINT_BASE_H

#include <cmath>
#include <cstdlib>
#include <cstdio>

const double ACCURACY = 0.0000001;
const double PI = 3.1415926;

class Point
{
public:
	double x;
	double y;
	//static const double ACCURACY;

	static bool check_double(double left, double right);
    Point();
	Point(double x0, double y0);
	int X() const;
	int Y() const;
	
	void set_point(double x0, double y0);

	Point operator+(const Point&) const;
	Point operator-(const Point&) const;
	void operator+=(const Point&);
	void operator-=(const Point&);

	Point operator*(double) const;
	Point operator/(double) const;	
	void operator*=(double);
	void operator/=(double);

	double dot(const Point& fix) const;
	double distance() const;
	double distance(const Point& p2) const;
	double distance_square() const;
	double distance_square(const Point& p2) const;
	
	double Angle() const;
	double Direction_angle(const Point& d1) const;
	double line_distance(const Point& deri,const Point& ref) const;
	double line_distance(const Point& deri) const;
	double line_orthogonal_distance(const Point& deri,const Point& ref) const;
	double line_orthogonal_distance(const Point& deri) const;

	Point To_line_ortho_deri(const Point& deri,const Point& ref) const;
	Point MiddlePoint(const Point& To,const double Sca) const;

	void rotate(const Point& deri);
	void scale(double L);
	void scale(const Point& deri);
	void orthogonal();
	void project(const Point& deri);
	bool Adjust_Dire(const Point& fix);

	bool operator==(const Point&) const;
	bool operator!=(const Point&) const;
	bool check_valid() const;
	
	void print() const
	{
		printf("x = %lf y = %lf\n",x,y);
	}
};
//const double Point::ACCURACY = 0.0000001;






#endif
