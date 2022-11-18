#ifndef LINKAGE_H
#define LINKAGE_H

#include <QVector>
#include <QGraphicsView>
#include "point.h"
#include <QMouseEvent>
#include "linkdata.h"

class Linkage : public QObject
{
    Q_OBJECT
public:
    Linkage(PointSE*, PointSE*, QVector<PointRot*>, PointMov*, LinkData*);
    ~Linkage();
    bool calculate();
    void changeMotorSpeed(int index, double speed);
    void show(QPixmap&);
    void sketch(QPixmap&);
    void drawText(QPixmap&);
    void rev_update();
    void movePoint(int, QPoint);
    void deleteBodyPoint(int);
    Linkage* copyLinkage();
    LinkData* getData();
    int getOnePointPos();
    const PointBC* operator[](size_t i) const;
    PointBC* operator[](size_t i);
    size_t size() const;
    PointSE* front();
    PointSE* back();


signals:

public slots:
private:
    PointSE* startPoint;
    PointSE* endPoint;
    QVector<PointRot*> body;
    LinkData* data;
    PointMov* onePoint;


};

#endif // LINKAGE_H
