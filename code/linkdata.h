#ifndef LINKDATA_H
#define LINKDATA_H

#include <QString>
#include <QPen>
#include <point.h>

enum LinkType{L_NORMAL = 0x01, L_SLIDER = 0x02};

struct CombineData
{
    bool connect_front = false;
    QString name_front;
    int point1_front;
    int point2_front;
    bool connect_back = false;
    QString name_back;
    int point1_back;
    int point2_back;
};

class LinkData
{
public:
    LinkData();
    LinkData(LinkType, QString, QPen, int, CombineData);
    LinkType type() const;
    QString name() const;
    QPen pen() const;
    bool isFrontCombine() const;
    bool isBackCombine() const;
    CombineData combine() const;
    int onepoint_position() const;
    QVector<double> motor_speeds() const;
    void set_name(QString);
    void set_motor_speeds(QVector<double>);
    void set_motor_speed(int index, double);
    void set_pen(QPen);
    void set_type(LinkType);
    void set_combine(CombineData c);
    void set_onepoint_position(int);
    PointSE* point1_front();
    PointSE* point1_back();
    PointBC* point2_front();
    PointBC* point2_back();
private:
    int onepoint_position_;
    QVector<double> motor_speeds_;
    LinkType type_;
    QString name_;
    QPen pen_;
    CombineData combine_;
    PointSE* point1_front_;
    PointBC* point2_front_;
    PointSE* point1_back_;
    PointBC* point2_back_;
};



#endif // LINKDATA_H
