#include "point_base.h"

bool Point::check_double(double left, double right)
{
    if(left > right)
    {
        if(ACCURACY < left-right)
            return false;
    }
    else
    {
        if(ACCURACY < right-left)
            return false;
    }
	return true;
}

Point::Point():x(0.0),y(0.0)
{}
Point::Point(double x0, double y0):x(x0),y(y0)
{}

int Point::X() const
{ return static_cast<int>(round(x)); }

int Point::Y() const
{ return static_cast<int>(round(y)); }

Point Point::operator+(const Point& p) const
{
    return Point(x+p.x,y+p.y);
}

Point Point::operator-(const Point& p) const
{
    return Point(x-p.x,y-p.y);
}

void Point::operator+=(const Point& p)
{
    x+=p.x; y+=p.y;
}

void Point::operator-=(const Point& p)
{
    x-=p.x; y-=p.y;
}

Point Point::operator*(double s) const
{
    return Point(x*s,y*s);
}

void Point::set_point(double x0, double y0)
{
    x = x0; y = y0;
}

void Point::operator*=(double s)
{
    x*=s; y*=s;
}

Point Point::operator/(double angle) const
{
    Point p1 = *this;
    p1 /= angle;
    return p1;
}

void Point::operator/=(double angle)
{
    double rad = angle*PI/180.0;
    double x1 = x*cos(rad) - y*sin(rad);
    double y1 = y*cos(rad) + x*sin(rad);
    set_point(x1, y1);
}

void Point::orthogonal()
{
    double x0 = -y;
    y = x;
    x = x0;
}

double Point::distance_square() const
{
    return x*x+y*y;
}

double Point::distance_square(const Point& p2) const
{
    return (x-p2.x)*(x-p2.x)+(y-p2.y)*(y-p2.y);
}

double Point::distance() const
{
    return sqrt(distance_square());
}

double Point::distance(const Point& p2) const
{
    return sqrt(distance_square(p2));
}

double Point::dot(const Point& fix) const
{
    return (x*fix.x + y*fix.y);
}

void Point::rotate(const Point& deri)
{
    *this = deri*( deri.distance() / distance() );
}

void Point::scale(double L)
{
    (*this) *= L / distance();
}

void Point::scale(const Point& deri)
{
    scale(deri.distance());
}

double Point::Direction_angle(const Point& d1) const
{
    return dot(d1)/sqrt(distance_square()*d1.distance_square());
}

void Point::project(const Point& deri)
{
    double angle = Direction_angle(deri);
    *this = deri*( angle*sqrt(distance_square()/deri.distance_square()) );
}

bool Point::Adjust_Dire(const Point& fix)
{
    if(dot(fix) < 0)
    {
        (*this) *= -1.0;
        return false;
    }
    return true;
}

double Point::line_distance(const Point& deri,const Point& ref) const
{
    //calculate the distance between point and line
    // ax + by = c
    return (*this-ref).line_distance(deri);
}

double Point::line_distance(const Point& deri) const
{
    //calculate the distance between point and line
    // ax + by = 0
    Point T = deri;
    T.orthogonal();
    return T.dot(*this) / T.distance();
}

double Point::line_orthogonal_distance(const Point& deri,const Point& ref) const
{
    //calculate the distance between point and line
    // ax + by = c
	//printf("d %lf %lf\n",deri.x,deri.y);
	//printf("r %lf %lf\n",ref.x,ref.y);
	//printf("t %lf %lf\n",this->x,this->y);
    return (*this-ref).line_orthogonal_distance(deri);
}

double Point::line_orthogonal_distance(const Point& deri) const
{
    //calculate the distance between point and line
    // ax + by = 0
	//printf("t %lf %lf\n",this->x,this->y);
    return deri.dot(*this) / deri.distance();
}

Point Point::To_line_ortho_deri(const Point& deri,const Point& ref) const
{
    double len = line_distance(deri,ref);
    Point T = deri;
    T.orthogonal();
    T.scale(len);
    return T;
}

Point Point::MiddlePoint(const Point& To,const double Sca) const
{
    Point T = To - *this;
    T.scale(Sca);
    return *this + T;
}

double Point::Angle() const
{
    return atan2(this->y,this->x) * 180.0 / PI;
}

bool Point::operator==(const Point& p2) const
{
    if(!check_double(p2.x, x))
		return false;
    if(!check_double(p2.y, y))
		return false;
    return true;
}

bool Point::operator!=(const Point& p2) const
{
    return !(*this==p2);
}

bool Point::check_valid() const
{
	//2147483648
	if(X() < -99999999 || X() > 99999999)
		return false;
	if(Y() < -99999999 || Y() > 99999999)
		return false;
	return true;
}
