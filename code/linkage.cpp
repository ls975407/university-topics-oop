    #include "linkage.h"


Linkage::Linkage(PointSE* st, PointSE* en, QVector<PointRot *> bo, PointMov* single, LinkData* da)
{
    startPoint = st;
    body = bo;
    endPoint = en;
    onePoint = single;
    data = da;
}

Linkage::~Linkage()
{
    startPoint->reset_condition();
    startPoint->erase();
    endPoint->reset_condition();
    endPoint->erase();
    for(int i=0;i<body.size();i++)
    {
        body[i]->reset_condition();
        body[i]->erase();
    }
    onePoint->reset_condition();
    onePoint->erase();
    delete startPoint;
    delete endPoint;
    delete onePoint;
    for(size_t i=0;i<body.size();i++)
        delete body[i];
}

void Linkage::show(QPixmap& pm)
{
    startPoint->rev_update();
    int count = 0;
    int total = body.size();
    for(size_t j=0;j<total;j++)
        if(body[j]->get_type() != PointBC::FIX)
            body[j]->set_cal_F();
    do
    {
        total -= count;
        count = 0;
        for(int j=0;j<total;j++)
            if(body[j]->get_type() != PointBC::FIX
                    && body[j]->if_cal() == false)
            {
                body[j]->cal_condition();
                count++;
            }
    }while(count == 0);

    for(int j=0;j<total;j++)
        if(body[j]->get_type() != PointBC::FIX)
            body[j]->update();
    sketch(pm);
}

void Linkage::sketch(QPixmap& pm)
{
    QPainter p(&pm);
    QPainter* painter = &p;
    int size = body.size();
    painter->setPen(data->pen());
    if(size != 0)
    {
    painter->drawLine(startPoint->X(), startPoint->Y(), body[0]->X(), body[0]->Y());
    painter->drawLine(onePoint->X(), onePoint->Y(), body[size-1]->X(), body[size-1]->Y());
    }
    else
    {
        painter->drawLine(startPoint->X(), startPoint->Y(), onePoint->X(), onePoint->Y());
        painter->drawLine(onePoint->X(), onePoint->Y(), onePoint->X(), onePoint->Y());
    }
    painter->drawLine(endPoint->X(), endPoint->Y(), onePoint->X(), onePoint->Y());
    painter->drawLine(endPoint->X()-5, endPoint->Y()+10, endPoint->X()+5, endPoint->Y()+10);
    painter->drawLine(endPoint->X()-5, endPoint->Y()+10, endPoint->X(), endPoint->Y());
    painter->drawLine(endPoint->X()+5, endPoint->Y()+10, endPoint->X(), endPoint->Y());
    painter->drawLine(startPoint->X()-5, startPoint->Y()+10, startPoint->X()+5, startPoint->Y()+10);
    painter->drawLine(startPoint->X()-5, startPoint->Y()+10, startPoint->X(), startPoint->Y());
    painter->drawLine(startPoint->X()+5, startPoint->Y()+10, startPoint->X(), startPoint->Y());

    if(size>1)
        for(size_t i=0;i<size-1;i++)
            painter->drawLine(body[i]->X(), body[i]->Y(), body[i+1]->X(), body[i+1]->Y());
    for(size_t i=0;i<size;i++)
    {
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(QPoint(body[i]->X(), body[i]->Y()), 7, 7);
        painter->drawEllipse(QPoint(body[i]->X(), body[i]->Y()), 12, 12);
    }
    painter->drawEllipse(QPoint(onePoint->X(), onePoint->Y()), 7, 7);
    painter->drawEllipse(QPoint(onePoint->X(), onePoint->Y()), 12, 12);

}

void Linkage::drawText(QPixmap &pm)
{
    QPainter painter(&pm);
    painter.setPen(QPen(data->pen().color(), 15));
    int i = 0;
    painter.drawText(QPoint(startPoint->X()-20, startPoint->Y()-15), "P" + QString::number(i));
    for(;i<body.size();i++)
        painter.drawText(QPoint(body[i]->X()-20, body[i]->Y()-20), "P" + QString::number(i+1));
    painter.drawText(QPoint(onePoint->X()-20, onePoint->Y()-20), "P" + QString::number((i++)+1));
    painter.drawText(QPoint(endPoint->X()-20, endPoint->Y()-15), "P" + QString::number(i+1));
}

bool Linkage::calculate()
{
    size_t i;
    const size_t N = size();
    const size_t posi = data->onepoint_position();
    for(i=0; i<posi; i++)
    {
        if(!operator[](i)->cal_condition())
            break;
    }
    if(i < posi)
    {
        operator[](i)->rev_update();
        return false;
    }
    for(i=N-1; i>=posi; i--)
    {
        if(!operator[](i)->cal_condition())
            break;
    }
    if(i >= posi)
    {
        operator[](i)->rev_update();
        return false;
    }
    return true;

}

void Linkage::changeMotorSpeed(int index, double speed)
{
    PointRot* point = dynamic_cast<PointRot*>(this->operator[](index));
    point->set_condition(unique_ptr<ConditionRot>(new CRot_point_linear(
                                                      this->operator[](index-1), speed, (this->operator[](index-1)->get_position()-point->get_position()).distance() )
                                                ));
    for(int i=index+1;i<size()-1;i++)
    {
        if(i == getOnePointPos())
        {
            if(data->type() == LinkType::L_SLIDER)
            {
                onePoint->set_condition
                    (
                        unique_ptr<ConditionMov>
                        (
                            new CMov_p1((this->operator[](i-1)->get_position()-this->operator[](i)->get_position()).distance(), this->operator[](i-1), this->operator[](i+1), true)
                             // new CMov_p1((p2-p3).distance(), end_point, body[0], false)
                        )
                    );

            }
            else
            {
                onePoint->set_condition
                (
                    unique_ptr<ConditionMov>
                    (
                        new CMov_p2( (this->operator[](i-1)->get_position()-this->operator[](i)->get_position()).distance(), this->operator[](i-1), (this->operator[](i+1)->get_position()-this->operator[](i)->get_position()).distance(), this->operator[](i+1) )
                    )
                );
            }
        }
        else
        {
            double s = getData()->motor_speeds()[i];
            PointRot* p = dynamic_cast<PointRot*>(this->operator[](i));
            p->set_condition(unique_ptr<ConditionRot>(new CRot_point_linear(
                                                              this->operator[](i-1), s, (this->operator[](i-1)->get_position()-point->get_position()).distance() )
                                                        ));
        }

    }
    this->getData()->set_motor_speed(index, speed);
}






void Linkage::rev_update()
{
    startPoint->rev_update();
    endPoint->rev_update();
    for(int i=0; i<body.size();i++)
        body[i]->rev_update();
    onePoint->rev_update();
}

void Linkage::movePoint(int lowLevelIndex, QPoint pos)
{
    Point p(pos.x(), pos.y());
    this->operator[](lowLevelIndex)->move_position(p);
    /*
    if(lowLevelIndex == 0)
    {
        startPoint->move_position(p);
    }
    else if(lowLevelIndex == data->onepoint_position()+1)
    {
        onePoint->move_position(p);
    }
    else if(lowLevelIndex == data->onepoint_position()+2)
    {
        endPoint->move_position(p);
    }
    else
        body[lowLevelIndex-1]->move_position(p);
        */
}

void Linkage::deleteBodyPoint(int index)
{

    if(index == 0 || index == data->onepoint_position() || index == this->size()-1)
        return;
    if(index < data->onepoint_position())
    {
        body[index-1]->reset_condition();
        if(index == data->onepoint_position()-1)
            onePoint->setPoint_L(this->operator[](index-1), this->operator[](index));

        else if(index == 1)
            body[index]->setPoint(this->front(), this->operator[](index));

        else
            body[index]->setPoint(this->operator[](index-1), this->operator[](index));
        data->set_onepoint_position(data->onepoint_position()-1);
        body[index-1]->erase();
        body.erase(body.begin()+index-1);
    }
    if(index > data->onepoint_position())
    {
        body[index-2]->reset_condition();
        if(index == data->onepoint_position()+1)
            onePoint->setPoint_R(this->operator[](index+1), onePoint);
        else if(index == this->size()-2)
            body[index]->setPoint(this->back(), this->operator[](index));
        else
            body[index]->setPoint(this->operator[](index+1), this->operator[](index));
        body[index-2]->erase();
        body.erase(body.begin()+index-2);
    }



}

Linkage *Linkage::copyLinkage()
{
    PointSE* start_point;
    PointSE* end_point;
    QVector<PointRot*> newBody;
    PointMov* one_point;

    start_point = new PointSE( startPoint->get_position() );
    one_point = new PointMov( onePoint->get_position() );
    end_point = new PointSE( endPoint->get_position() );

    for(int i=0;i<body.size();i++)
    {
        double n = getData()->motor_speeds()[i+1];
        newBody.push_back( new PointRot(body[i]->get_position()) );
        if(i > 0)
            newBody[i]->set_condition( unique_ptr<ConditionRot>(new CRot_point_linear( newBody[i-1], n, (newBody[i-1]->get_position()-newBody[i]->get_position()).distance() )) );
        else
            newBody[0]->set_condition( unique_ptr<ConditionRot>(new CRot_point_linear( start_point, n, (start_point->get_position()-newBody[i]->get_position()).distance() )) );
    }
    if(body.size() > 0)
    {
        if(data->type() == LinkType::L_SLIDER)
        {
            if(body.size() != 0)
                one_point->set_condition
                    (
                        unique_ptr<ConditionMov>
                        (
                            new CMov_p1((newBody.last()->get_position()-one_point->get_position()).distance(), newBody[newBody.size()-1], end_point, true)
                             // new CMov_p1((p2-p3).distance(), end_point, body[0], false)
                        )
                    );
        }
        else
        {

                one_point->set_condition
                (
                    unique_ptr<ConditionMov>
                    (
                        new CMov_p2( (newBody.last()->get_position()-one_point->get_position()).distance(), newBody[newBody.size()-1], (one_point->get_position()-end_point->get_position()).distance(), end_point )
                    )
                );

        }
    }
    else
    {
        if(data->type() == LinkType::L_SLIDER)
        {

                one_point->set_condition
                    (
                        unique_ptr<ConditionMov>
                        (
                            new CMov_p1((start_point->get_position()-one_point->get_position()).distance(), start_point, end_point, true)
                             // new CMov_p1((p2-p3).distance(), end_point, body[0], false)
                        )
                    );
        }
        else
        {

                one_point->set_condition
                (
                    unique_ptr<ConditionMov>
                    (
                        new CMov_p2( (start_point->get_position()-one_point->get_position()).distance(), start_point, (one_point->get_position()-end_point->get_position()).distance(), end_point )
                    )
                );

        }
    }

    LinkData* newdata = new LinkData(*data);
    Linkage* link = new Linkage(start_point, end_point, newBody, one_point, newdata);
    return link;
}

LinkData* Linkage::getData()
{
    return data;
}

int Linkage::getOnePointPos()
{
    return data->onepoint_position();
}


const PointBC* Linkage::operator[](size_t i) const
{
    if(i == 0)
        return startPoint;
    if(i < data->onepoint_position())
        return body[i-1];
    if(i == data->onepoint_position())
        return onePoint;
    if(i == body.size() + 2)
        return endPoint;
    return body[i-2];
}

PointBC* Linkage::operator[](size_t i)
{
    if(i == 0)
        return startPoint;
    if(i < data->onepoint_position())
        return body[i-1];
    if(i == data->onepoint_position())
        return onePoint;
    if((int)i == (body.size()+2))
        return endPoint;
    return body[i-2];
}

size_t Linkage::size() const
{
    return body.size()+3;
}

PointSE* Linkage::front()
{	return startPoint;}

PointSE* Linkage::back()
{	return endPoint;}

