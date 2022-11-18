#include "linkdata.h"

LinkData::LinkData()
{}

LinkData::LinkData(LinkType ty, QString str, QPen p, int position, CombineData c)
    :type_(ty), name_(str), pen_(p), onepoint_position_(position), combine_(c)
{}

LinkType LinkData::type() const
{
    return type_;
}

QString LinkData::name() const
{
    return name_;
}

QPen LinkData::pen() const
{
    return pen_;
}

bool LinkData::isFrontCombine() const
{
    return combine_.connect_front;
}

bool LinkData::isBackCombine() const
{
    return combine_.connect_back;
}

CombineData LinkData::combine() const
{
    return combine_;
}

int LinkData::onepoint_position() const
{
    return onepoint_position_;
}

QVector<double> LinkData::motor_speeds() const
{
    return  motor_speeds_;
}

void LinkData::set_name(QString str)
{
    name_ = str;
}

void LinkData::set_motor_speeds(QVector<double> v)
{
    motor_speeds_ = v;
}

void LinkData::set_motor_speed(int index, double speed)
{
    motor_speeds_[index] = speed;
}

void LinkData::set_pen(QPen p)
{
    pen_ = p;
}

void LinkData::set_type(LinkType ty)
{
    type_ = ty;
}

void LinkData::set_combine(CombineData c)
{
    combine_ = c;
}

void LinkData::set_onepoint_position(int position)
{
    onepoint_position_ = position;
}

PointSE *LinkData::point1_front()
{
    return point1_front_;
}

PointSE *LinkData::point1_back()
{
    return point1_back_;
}

PointBC *LinkData::point2_front()
{
    return point2_front_;
}

PointBC *LinkData::point2_back()
{
    return point2_back_;
}













