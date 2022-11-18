#include "point.h"


Index2::Index2()
{
    index_1 = 0; index_2 = 0;
}

Index2::Index2(size_t i,size_t j)
{
    index_1 = i; index_2 = j;
}

//////////////////////////////////////////

const size_t SimState::SimError = 0x01;
const size_t SimState::CondError = 0x02;
const size_t SimState::Cond2Loop = 0x04;
const size_t SimState::OpenError = 0x08;

const char* SimState::SimError_Word = "The state cannot be simulation\n";
const char* SimState::CondError_Word = "The state cannot be simulation\n";
void SimState::print(size_t flag)
{
    if(flag&SimError)
        puts(SimError_Word);
    if(flag&CondError)
        puts(CondError_Word);
}
//////////////////////////////////////////

void StateObj::set_state(size_t flag)
{ state |= flag; }
void StateObj::clear_state(size_t flag)
{ state &= ~flag;}
void StateObj::reset_state()
{ state = 0; }

size_t StateObj::get_state(size_t flag)
{ return state&flag; }

/////////////////////////////////////////

PointBC::PointBC()
{ }

PointSE::PointSE()
{ }

PointMov::PointMov()
{ }

PointRot::PointRot()
{ }

//////////////////////////////////////////
void PointBC::rev_update()
{    new_point = old_point; }
void PointBC::set_cal_T()
{    is_cal = true;}

bool PointBC::if_cal() const
{	return is_cal; }
const Point PointBC::get_position() const
{   return new_point;}
const Point PointBC::get_remain_position() const
{   return old_point;}
int PointBC::X() const
{    return new_point.X();}
int PointBC::Y() const
{	return new_point.Y();}

void PointBC::update()
{    old_point = new_point;}
void PointBC::set_cal_F()
{    is_cal = false;    }

PointBC::~PointBC()
{}

//////////////////////////////////////////

ConditionSE::~ConditionSE()
{}

ConditionMov::~ConditionMov()
{}

ConditionRot::~ConditionRot()
{}

//////////////////////////////////////////

void PointSE::recalculate_coef()
{
    if(condition)
    {
        condition->recalculate_coef();
    }
}

ConditionSE::ConditionSE(PointSE* np, const PointBC* p0, const PointBC* p1)
{
    pointer = np;
    point_0 = p0;
    point_1 = p1;
    must_cal = false;
}

void ConditionSE::set_cal_T()
{ must_cal = true; }
void ConditionSE::set_cal_F()
{ must_cal = false; }
bool ConditionSE::if_must_cal() const
{  return must_cal; }

const PointBC* ConditionSE::getPoint_0() const
{
    return point_0;
}

const PointBC* ConditionSE::getPoint_1() const
{
    return point_1;
}

CRef_p2_scale::CRef_p2_scale(double l, PointSE* np, const PointBC* p0, const PointBC* p1)
    :ConditionSE(np,p0,p1)
{
    len_x = l;
}

CRef_p2_scale::CRef_p2_scale(PointSE* np, const PointBC* p0, const PointBC* p1)
    :ConditionSE(np,p0,p1)
{
    recalculate_coef();
}

bool CRef_p2_scale::calculate() const
{
    if(!if_must_cal() && (!point_0->if_cal() || !point_1->if_cal()) )
        return false;
    Point& new_point = pointer->new_point;

    const Point& p0 = point_0->get_position();
    const Point& p1 = point_1->get_position();
    new_point = p1-p0;
    new_point.scale(len_x);
    new_point += p0;
    return new_point.check_valid();
}

void CRef_p2_scale::recalculate_coef()
{
    const Point& new_point = pointer->new_point;
    len_x = new_point.line_orthogonal_distance(point_1->get_position() - point_0->get_position(),point_0->get_position());
}

/////////
CRef_p2_fix::CRef_p2_fix(double ly, const CRef_p2_scale* pc)
    :CRef_p2_scale(*pc)
{
    len_y = ly;
}

CRef_p2_fix::CRef_p2_fix(double lx,double ly, PointSE* np,const PointBC* p0, const PointBC* p1)
    :CRef_p2_scale(lx,np,p0,p1)
{
    len_y = ly;
}

CRef_p2_fix::CRef_p2_fix(PointSE* np, const PointBC* p0, const PointBC* p1)
    :CRef_p2_scale(np,p0,p1)
{
    const Point& new_point = pointer->new_point;
    len_y = new_point.line_distance(p1->get_position() - p0->get_position(),p0->get_position());
}

bool CRef_p2_fix::calculate() const
{
    if(!if_must_cal() && (!point_0->if_cal() || !point_1->if_cal()) )
        return false;

    Point& new_point = pointer->new_point;

    const Point& p0 = point_0->get_position();
    const Point& p1 = point_1->get_position();
    Point vector_x = p1-p0;
    Point vector_y = vector_x;
    vector_y.orthogonal();

    vector_x.scale(len_x);
    vector_y.scale(len_y);

    //printf("-len_x = %lf\tlen_y = %lf\n",vector_x.distance(),vector_y.distance());
    new_point = p0  + vector_x + vector_y;
    return new_point.check_valid();
}

void CRef_p2_fix::recalculate_coef()
{
    const Point& new_point = pointer->new_point;
    len_x = new_point.line_orthogonal_distance(point_1->get_position() - point_0->get_position(),point_0->get_position());
    len_y = new_point.line_distance(point_1->get_position() - point_0->get_position(),point_0->get_position());
    //printf("len_x = %lf\tlen_y = %lf\n",len_x,len_y);
}

//////////

CMov_p1::CMov_p1(double l, const PointBC* pl, const PointBC* pd, bool is_l)
{ len = l; point_dire = pd; point_len = pl;  is_left = is_l; }

void CMov_p1::setLen(const PointBC* new_point,const PointBC* old_point)
{
    point_len = new_point;
    len = (new_point->get_position()-old_point->get_position()).distance();
}
void CMov_p1::setDirection(const PointBC* new_point,const PointBC* old_point)
{
    point_dire = new_point;
}


bool CMov_p1::calculate(Point& new_point, const Point& old_position) const
{
    if(!point_len->if_cal() || !point_dire->if_cal())
        return false;


    const Point& c1 = point_len->get_position();

    Point vector_xy = point_dire->get_position()-old_position;
    Point ortho_xy =  c1.To_line_ortho_deri(vector_xy,old_position) * (-1);

    vector_xy.Adjust_Dire(old_position-c1);

    double Len_double = len*len-ortho_xy.distance_square();
    if(Len_double < ACCURACY) return false;
    vector_xy.scale(sqrt(Len_double));

    new_point = c1 + vector_xy + ortho_xy;
    return new_point.check_valid();
}

void CMov_p1::set_is_left(bool is_l)
{
    is_left = is_l;
}

void PointBC::replace_rel_point(PointBC* new_point, PointBC* old_point)
{
    del_rel_point(old_point);
    set_rel_point(new_point);
}

void PointMov::setPoint_L(PointBC* new_point, PointBC* old_point)
{
    PointBC* point = const_cast<PointBC*>(condition->getPoint_L());
    del_rel_point(point);
    condition->setPoint_L(new_point, old_point);

    set_rel_point(new_point);
}

void PointMov::setPoint_R(PointBC* new_point, PointBC* old_point)
{
    PointBC* point = const_cast<PointBC*>(condition->getPoint_R());
    del_rel_point(point);
    condition->setPoint_R(new_point, old_point);

    set_rel_point(new_point);
}

void CMov_p1::setPoint_L(const PointBC* new_point, const PointBC* old_point)
{
    if(is_left)
        setLen(new_point,old_point);
    else
        setDirection(new_point,old_point);
}

void CMov_p1::setPoint_R(const PointBC* new_point, const PointBC* old_point)
{
    if(!is_left)
        setLen(new_point,old_point);
    else
        setDirection(new_point,old_point);
}

const PointBC* CMov_p1::getPoint_L() const
{
    if(is_left)
        return point_len;
    else
        return point_dire;
}

const PointBC* CMov_p1::getPoint_R() const
{
    if(!is_left)
        return point_len;
    else
        return point_dire;
}

///////////////

CMov_p2::CMov_p2(const double l1, const PointBC* p1, const double l2, const PointBC* p2)
{ L1 = l1; L2 = l2; point_1 = p1; point_2 = p2; }

void CMov_p2::setLen
  (
    const PointBC*& point_target, double& len_target,
    const PointBC* new_point, const PointBC* old_point
  )
{
    point_target = new_point;
    len_target = (new_point->get_position()-old_point->get_position()).distance();
}

bool CMov_p2::calculate(Point& new_point, const Point& old_position) const
{
    if(!point_1->if_cal() || !point_2->if_cal())
        return false;

    const Point& c1 = point_1->get_position();
    const Point& c2 = point_2->get_position();
    Point& p1 = new_point;
    //const Point& old_p = old_position;

    Point vector_xy = c2 - c1;
    Point ortho_xy = vector_xy;
    ortho_xy.orthogonal();

    Point check_xy = p1 - c1;
    ortho_xy.Adjust_Dire(check_xy);

    double Len_double = vector_xy.distance_square();
    double Len        = sqrt(Len_double);

    if(L1+L2-Len < ACCURACY) return false;
    if(L2+Len-L1 < ACCURACY) return false;

    double cos_Len = (L1*L1-L2*L2)/2/Len_double+0.5;
    double SS = (L1 + L2 + Len)/2.0;
    double sin_Len = 2.0*sqrt(SS*(SS-Len))*sqrt((SS-L1)*(SS-L2))/Len_double;

    vector_xy *= cos_Len;
    ortho_xy  *= sin_Len;
    p1 = c1 + vector_xy + ortho_xy;
    return new_point.check_valid();
}

void CMov_p2::set_is_left(bool is_l)
{ }

void CMov_p2::setPoint_L(const PointBC* new_point, const PointBC* old_point)
{
    setLen(point_1,L1,new_point,old_point);
}

void CMov_p2::setPoint_R(const PointBC* new_point, const PointBC* old_point)
{
    setLen(point_2,L2,new_point,old_point);
}

const PointBC* CMov_p2::getPoint_L() const
{
    return point_1;
}

const PointBC* CMov_p2::getPoint_R() const
{
    return point_2;
}

////////////////////////////////////////////////

ConditionRot::ConditionRot(const PointBC* point,double angle, double len)
{
    center_point = point;
    step_angle = angle;
    length = len;
}

void PointRot::setPoint(PointBC* new_point, PointBC* old_point)
{
    PointBC* point;
    point = const_cast<PointBC*>(condition->getPoint());
    del_rel_point(point);
    condition->setPoint(new_point);

    set_rel_point(new_point);
}


void ConditionRot::setPoint(const PointBC* new_point)
{
    center_point = new_point;
}

const PointBC* ConditionRot::getPoint() const
{
    return center_point;
}

void ConditionRot::setDirection(bool is_counterwise)
{
    if(is_counterwise && step_angle<0 || !is_counterwise && step_angle>0) step_angle *= -1;
}

/////////////////////

CRot_point_linear::CRot_point_linear(const PointBC* p, double angle, double len): ConditionRot(p,angle,len)
{}

bool CRot_point_linear::calculate(Point& new_point, const Point& old_position) const
{
    if(!center_point->if_cal())
        return false;

    new_point =  center_point->get_position();

    Point vec = old_position-center_point->get_remain_position();
    vec /= step_angle;
    vec.scale(length);
    new_point += vec;
    return new_point.check_valid();;
}

/////////////////////

//public function of PointBC

void PointBC::ini_value(const Point& p)
{
    new_point = p;
    old_point = p;
    state = 0;
    is_cal = true;
}

PointBC::PointBC(const Point& p)
{
    ini_value(p);
}

PointBC::PointBC(const double x, const double y)
{
    ini_value(Point(x,y));
}

//////////

PointSE::PointSE(const Point& p): PointBC(p)
{
    condition.reset();
}

PointSE::PointSE(const double x, const double y): PointBC(Point(x,y))
{
    condition.reset();
}


PointSE::~PointSE()
{
    if(v_rel_points.size() > 0)
    {
        printf("you are not release this PointSE relation with other point\n");
        ////exit(1);
    }
}
//////////

PointMov::PointMov(const Point& p): PointBC(p)
{
    condition.reset();
}

PointMov::PointMov(const double x, const double y): PointBC(Point(x,y))
{
    condition.reset();
}

PointMov::~PointMov()
{
    if(v_rel_points.size() > 0)
    {
        printf("you are not release this PointMov relation with other point\n");
        ////exit(1);
    }
}

PointRot::PointRot(const Point& p): PointBC(p)
{
    condition.reset();
}

PointRot::PointRot(const double x, const double y): PointBC(Point(x,y))
{
    condition.reset();
}

PointRot::~PointRot()
{
    if(v_rel_points.size() > 0)
    {
        printf("you are not release this PointRot relation with other point\n");
        ////exit(1);
    }
}

//inline function of PointBC

inline void PointBC::print_id()
{
    printf("id is the following\n");
    for(int i=0; i<v_rel_points.size(); i++)
        printf("v_rel_points[%d] = %p\n",i,v_rel_points[i]);
    puts("");
}

inline void PointBC::set_rel_point(PointBC* add_point)
{
    /*
    add_point->print_id();
    printf("you want to add v_rel_points into %p\n",this);
    printf("vs is from = %p\n",add_point);
    add_point->print_id();
    */
    for(size_t i=0; i<add_point->v_rel_points.size(); i++)
    {
        //printf("\t=>%p\n",add_point->v_rel_points[i]);
        if(this == add_point->v_rel_points[i])
        {
            printf("You add double condition in the same point\n");
            ////exit(1);
        }
    }
    add_point->v_rel_points.push_back(this);
}

inline void PointBC::del_rel_point(PointBC* del_point)
{
    /*
    del_point->print_id();
    printf("you want to del v_rel_points = %p\n",this);
    printf("vs is from = %p\n",del_point);
    */
    for(size_t i=0; i<del_point->v_rel_points.size(); i++)
    {
        //printf("\t=>%p\n",del_point->v_rel_points[i]);
        if(this == del_point->v_rel_points[i])
        {
            del_point->v_rel_points.erase(del_point->v_rel_points.begin()+i);
            return;
        }
    }
    printf("You erase the null point in the PointBC::del_rel_point\n");
    ////exit(1);
}

//protected function of PointSE
//set_condition reset_condition cal_condition

void PointSE::set_condition(unique_ptr<ConditionSE> cond)
{
    reset_condition();
    if(cond)
    {
        condition = move(cond);

        PointBC* point;
        point = const_cast<PointBC*>(condition->getPoint_0());
        set_rel_point(point);

        point = const_cast<PointBC*>(condition->getPoint_1());
        set_rel_point(point);
        //get_position().print();
        condition->set_cal_T();
        condition->calculate();
        update();
        condition->set_cal_F();
        //get_position().print();
    }
}

void PointSE::erase()
{
    for(size_t i=0; i<v_rel_points.size(); i++)
        v_rel_points[i]->reset_condition();
    //v_rel_points.clear();
}

void PointSE::reset_condition()
{
    if(condition)
    {
        PointBC* point;
        point = const_cast<PointBC*>(condition->getPoint_0());
        del_rel_point(point);

        point = const_cast<PointBC*>(condition->getPoint_1());
        del_rel_point(point);

        condition.reset();
    }
}

bool PointSE::cal_condition()
{
    if(if_cal())
        return true;
    if(!condition)
    {
        set_cal_T();
        clear_state(SimState::SimError);
        return true;
    }
    if(condition->calculate())
    {
        set_cal_T();
        clear_state(SimState::SimError);
        return true;
    }
    set_state(SimState::SimError);
    return false;
}

//protected function of PointMov
//set_condition reset_condition cal_condition

void PointMov::set_condition(unique_ptr<ConditionMov> cond)
{
    reset_condition();
    condition = move(cond);

    PointBC* point;
    point = const_cast<PointBC*>(condition->getPoint_L());
    set_rel_point(point);

    point = const_cast<PointBC*>(condition->getPoint_R());
    set_rel_point(point);

}

void PointMov::erase()
{
    for(size_t i=0; i<v_rel_points.size(); i++)
        v_rel_points[i]->reset_condition();
    //v_rel_points.clear();
}

void PointMov::reset_condition()
{
    if(condition)
    {
        PointBC* point;
        point = const_cast<PointBC*>(condition->getPoint_L());
        del_rel_point(point);
        point = const_cast<PointBC*>(condition->getPoint_R());
        del_rel_point(point);

        //delete condition;
        condition.reset();
    }
}

bool PointMov::cal_condition()
{
    if(!condition)
    {
        printf("MOV condition should not be NULL in PointMov::cal_condition()\n");
        //exit(1);
    }
    if(if_cal())
    {
        set_cal_T();
        clear_state(SimState::SimError);
        return true;
    }
    if(condition->calculate(new_point, this->get_position()))
    {
        set_cal_T();
        clear_state(SimState::SimError);
        return true;
    }
    set_state(SimState::SimError);
    return false;
}

//protected function of PointRot
//set_condition reset_condition cal_condition

void PointRot::set_condition(unique_ptr<ConditionRot> cond)
{
    reset_condition();
    condition = move(cond);

    PointBC* point;
    point = const_cast<PointBC*>(condition->getPoint());
    set_rel_point(point);
}

void PointRot::erase()
{
    for(size_t i=0; i<v_rel_points.size(); i++)
        v_rel_points[i]->reset_condition();
    //v_rel_points.clear();
}

void PointRot::reset_condition()
{
    if(condition)
    {
        PointBC* point;
        point = const_cast<PointBC*>(condition->getPoint());
        del_rel_point(point);

        //delete condition;
        condition.reset();
    }
}

bool PointRot::cal_condition()
{
    if(!condition)
    {
        printf("MOV condition should not be NULL in PointRot::cal_condition()\n");
        //exit(1);
    }
    if(if_cal())
    {
        set_cal_T();
        clear_state(SimState::SimError);
        return true;
    }
    if(condition->calculate(new_point, this->get_position()))
    {
        set_cal_T();
        clear_state(SimState::SimError);
        return true;
    }
    set_state(SimState::SimError);
    return false;
}

//virtual function of PointSE, PointMov, PointRot

int PointSE::get_type() const
{
    if(!condition) return PointBC::FIX;
    else return PointBC::REF;
}

int PointMov::get_type() const
{    return PointBC::MOV; }

int PointRot::get_type() const
{    return PointBC::ROT; }


////////////////////////////////////////////////////////////////////////////////

bool PointSE::check_condition()
{
    bool re_bool = false;
    if(!condition)
    {
        clear_state(SimState::CondError);
        re_bool = true;
    }
    else if
    (
       typeid(CRef_p2_scale) == typeid(*condition) //||
    )
    {
        clear_state(SimState::CondError);
        re_bool = true;
    }
    else if
    (
       typeid(CRef_p2_fix) == typeid(*condition) //||
    )
    {
        clear_state(SimState::CondError);
        re_bool = true;
    }
    if(re_bool)
    {
        const PointBC* p0 = condition->getPoint_0();
        const PointBC* p1 = condition->getPoint_1();

        for(size_t i=0; i<v_rel_points.size(); i++)
            if(v_rel_points[i] == p0 || v_rel_points[i] == p1)
            {
                printf("PointSE condition loop to itself\n");
                re_bool = false;
            }
        if(re_bool) clear_state(SimState::Cond2Loop);
        else 		set_state(SimState::Cond2Loop);
        return re_bool;
    }
    printf("PointSE condition is errror: No such condition\n");
    set_state(SimState::CondError);
    return false;
}

bool PointMov::check_condition()
{
    bool re_bool = false;
    if(!condition)
    {
        set_state(SimState::CondError);
        re_bool = true;
    }
    if
    (
       typeid(CMov_p1) == typeid(*condition) ||
       typeid(CMov_p2) == typeid(*condition)
    )
    {
        clear_state(SimState::CondError);
        re_bool = true;
    }
    if(re_bool)
    {
        const PointBC* p0 = condition->getPoint_L();
        const PointBC* p1 = condition->getPoint_R();

        for(size_t i=0; i<v_rel_points.size(); i++)
            if(v_rel_points[i] == p0 || v_rel_points[i] == p1)
            {
                printf("PointMov condition loop to itself\n");
                re_bool = false;
            }
        if(re_bool) clear_state(SimState::Cond2Loop);
        else 		set_state(SimState::Cond2Loop);
        return re_bool;
    }
    printf("PointMov condition is errror: No such condition\n");
    set_state(SimState::CondError);
    return false;
}


bool PointRot::check_condition()
{
    bool re_bool = false;
    if(!condition)
    {
        set_state(SimState::CondError);
        re_bool = true;
    }
    else if
    (
       typeid(CRot_point_linear) == typeid(*condition) //||
    )
    {
        clear_state(SimState::CondError);
        re_bool = true;
    }
    if(re_bool)
    {
        const PointBC* p = condition->getPoint();

        for(size_t i=0; i<v_rel_points.size(); i++)
            if(v_rel_points[i] == p)
            {
                printf("PointRot condition loop to itself\n");
                re_bool = false;
            }
        if(re_bool) clear_state(SimState::Cond2Loop);
        else 		set_state(SimState::Cond2Loop);
        return re_bool;
    }
    printf("PointRot condition is errror: No such condition\n");
    set_state(SimState::CondError);
    return false;
}

PointBC* PointSE::copy_pointbc()
{
    return new PointSE(*this);
}


PointBC* PointMov::copy_pointbc()
{
    return new PointMov(*this);
}


PointBC* PointRot::copy_pointbc()
{
    return new PointRot(*this);
}

/////////////////////////////////////////////////////////////////////


void CMov_p1::recalculate_coef(const Point& target_point)
{
    len = ( target_point-point_len->get_position() ).distance();
}

void CMov_p2::recalculate_coef(const Point& target_point)
{
    L1 = ( target_point-point_1->get_position() ).distance();
    L2 = ( target_point-point_2->get_position() ).distance();
}

void CRot_point_linear::recalculate_coef(const Point& target_point)
{
    length = ( target_point-center_point->get_position() ).distance();
}

void PointSE::recalculate_condition_coef()
{
    if(condition)
    {
        condition->set_cal_T();
        condition->calculate();
        update();
        condition->set_cal_F();
    }
}

void PointMov::recalculate_condition_coef()
{
    condition->recalculate_coef(get_position());
}

void PointRot::recalculate_condition_coef()
{
    condition->recalculate_coef(get_position());
}

void PointMov::move_position(const Point& next_position)
{
    new_point = next_position;
    update();
    condition->recalculate_coef(next_position);
    for(size_t i=0; i<v_rel_points.size(); i++)
        v_rel_points[i]->recalculate_condition_coef();
}

void PointRot::move_position(const Point& next_position)
{
    new_point = next_position;
    update();
    condition->recalculate_coef(next_position);
    for(size_t i=0; i<v_rel_points.size(); i++)
        v_rel_points[i]->recalculate_condition_coef();
}

void PointSE::move_position(const Point& next_position)
{
    if(!condition)
    {
        new_point = next_position;
        update();
        for(size_t i=0; i<v_rel_points.size(); i++)
            v_rel_points[i]->recalculate_condition_coef();
    }
}

///////////////////////////////////////////////////////////////////

bool PointSE::check_answer() const
{
    return condition->check_answer();
}

bool CRef_p2_scale::check_answer() const
{
    return true;
}

bool CRef_p2_fix::check_answer() const
{
    return true;
}

bool CMov_p1::check_answer(const Point& new_point) const
{
    if(is_left)
    {
        const Point& c1 = point_len->get_position();
        if(! Point::check_double(new_point.distance(c1), len))
            return false;
        return true;
    }
    else
    {
        const Point& c1 = point_dire->get_position();
        if(! Point::check_double(new_point.distance(c1), len))
            return false;
        return true;
    }
}

bool CMov_p2::check_answer(const Point& new_point) const
{
    const Point& c1 = point_1->get_position();
    const Point& c2 = point_2->get_position();
    if(! Point::check_double(new_point.distance(c1), L1))
        return false;
    if(! Point::check_double(new_point.distance(c2), L2))
        return false;
    //printf("Point n  x y = %d %d\n",new_point.X(),new_point.Y());
    //printf("Point c1 x y = %d %d\n",c1.X(),c1.Y());
    //printf("Point c2 x y = %d %d\n",c2.X(),c2.Y());
    //printf("len x y = %lf %lf\n\n",L1,L2);
    return true;
}

bool CRot_point_linear::check_answer(const Point& new_point) const
{
    return true;
}

bool PointMov::check_answer() const
{
    return condition->check_answer(new_point);
}

bool PointRot::check_answer() const
{
    return condition->check_answer(new_point);
}

///////////////////////////////////////////////////////////////////

void PointSE::save_file(FILE* fpw) const
{
    Data_IO::save_point(new_point,fpw);
    if(!condition)
        Data_IO::print_next(fpw);
    else
        condition->save_file(fpw);
}

void PointMov::save_file(FILE* fpw) const
{
    Data_IO::save_point(new_point,fpw);
    condition->save_file(fpw);
}

void PointRot::save_file(FILE* fpw) const
{
    Data_IO::save_point(new_point,fpw);
    condition->save_file(fpw);
}


//////////////////////////////////////////////////////////////////////////////////////

inline bool Data_IO::get_double(double& temp,FILE* fpr)
{
    if(fscanf(fpr,"%lf",&temp) != 1)
        return false;
    return true;
}
inline void Data_IO::save_double(double temp,FILE* fpw)
{
    fprintf(fpw,"%lf ",temp);
}

inline bool Data_IO::get_int(int& temp,FILE* fpr)
{
    if(fscanf(fpr,"%d",&temp) != 1)
        return false;
    return true;
}
inline void Data_IO::save_int(int temp,FILE* fpw)
{
    fprintf(fpw,"%d ",temp);
}

inline bool Data_IO::get_index(size_t& temp,FILE* fpr)
{
    if(fscanf(fpr,"%ld",&temp) != 1)
        return false;
    return true;
}
inline void Data_IO::save_index(size_t temp,FILE* fpw)
{
    fprintf(fpw,"%ld ",temp);
}

inline bool Data_IO::get_bool(bool& temp,FILE* fpr)
{
    int temp_int = -1;
    if(fscanf(fpr,"%d",&temp_int) != 1)
        return false;
    if(temp_int == 0)
        temp = false;
    else if(temp_int == 1)
        temp = true;
    else
        return false;
    return true;
}

inline void Data_IO::save_bool(bool temp,FILE* fpw)
{
    if(temp) fprintf(fpw,"1 ");
    else     fprintf(fpw,"0 ");
}

inline bool Data_IO::get_point(Point& temp,FILE* fpr)
{
    if(fscanf(fpr,"%lf %lf",&temp.x,&temp.y) != 2)
        return false;
    return true;
}

inline void Data_IO::save_point(const Point& temp,FILE* fpw)
{
    fprintf(fpw,"%lf %lf",temp.x,temp.y);
}

inline void Data_IO::print_next(FILE* fpw)
{
    fprintf(fpw,"\n");
}

bool Index2::get_index(Index2& temp, FILE* fpr)
{
    if( fscanf(fpr,"%d %d",&temp.index_1,&temp.index_2) != 2)
        return false;
    return true;
}

bool PointBC::get_index(Index2& temp, FILE* fpr)
{
    return Index2::get_index(temp, fpr);
}

void Index2::save_index(FILE* fpw) const
{
    fprintf(fpw,"%d %d ",index_1,index_2);
}

size_t Index2::operator[](int i) const
{
    if(i == 0) return index_1;
    return index_2;
}

//////////////////////////////////////////////////////////////////////////////////////

bool PointSE::open_file(FILE* fpr,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    Point temp;
    if(!Data_IO::get_point(temp,fpr))
        re_bool = false;
    if(!re_bool)
    {
        set_state(SimState::OpenError);
        return re_bool;
    }

    ini_value(temp);
    ConditionSE* temp_pointer = NULL;
    re_bool = ConditionSE::new_element(fpr,&temp_pointer,Table);
    if(!re_bool)
    {
        set_state(SimState::OpenError);
        return re_bool;
    }
    set_condition( unique_ptr<ConditionSE>(temp_pointer) );
    clear_state(SimState::OpenError);
    return re_bool;
}

bool ConditionSE::new_element(FILE* fpr,ConditionSE** re_pointer,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    re_pointer = NULL;
    int id = -1;
    if(!Data_IO::get_int(id,fpr))
    {
        printf("there is a id error in the ConditionRot");
        return false;
    }
    switch(id)
    {
        case Condition::FIX_POSITION:
        {
            re_pointer = NULL;
            break;
        }
        case Condition::CREF_P2_SCALE:
        {
            re_bool = CRef_p2_scale::new_element(fpr, re_pointer,Table);
            break;
        }
        case Condition::CREF_P2_FIX:
        {
            re_bool = CRef_p2_fix::new_element(fpr, re_pointer,Table);
            break;
        }
        default:
            re_bool = false;
            break;
    }
    if(re_bool)
        return re_bool;
    else
    {
        printf("Read file error in the ConditionSE id = %d\n",id);
        return re_bool;
    }
}

void CRef_p2_scale::save_file(FILE* fpw,bool is_id = true) const
{
    if(is_id) Data_IO::save_int(Condition::CREF_P2_SCALE,fpw);
    pointer->save_index(fpw);
    point_0->save_index(fpw);
    point_1->save_index(fpw);
    Data_IO::save_double(len_x,fpw);
}

bool CRef_p2_scale::new_element(FILE* fpr,ConditionSE** re_pointer,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    //////////////////////////////
    PointSE* pointer; //to access the past position data
    const PointBC* point_0;
    const PointBC* point_1;

    double len_x;
    //////////////////////////////

    Index2 P;
    re_bool = PointBC::get_index(P,fpr);	if(!re_bool) return false;
    pointer = dynamic_cast<PointSE*>(Table[P[0]][P[1]]);
    if(!pointer) return false;

    re_bool = PointBC::get_index(P,fpr);	if(!re_bool) return false;
    point_0 = Table[P[0]][P[1]];

    re_bool = PointBC::get_index(P,fpr);	if(!re_bool) return false;
    point_1 = Table[P[0]][P[1]];

    re_bool = Data_IO::get_double(len_x,fpr);	if(!re_bool) return false;
    //////////////////////////////
    *re_pointer = new CRef_p2_scale(len_x,pointer,point_0,point_1);
    return true;
}

void CRef_p2_fix::save_file(FILE* fpw,bool is_id = true) const
{
    if(is_id) Data_IO::save_int(Condition::CREF_P2_FIX,fpw);
    CRef_p2_scale::save_file(fpw,false);
    Data_IO::save_double(len_y,fpw);
}

bool CRef_p2_fix::new_element(FILE* fpr,ConditionSE** re_pointer,const vector<vector<PointBC*>>& Table)
{
    ConditionSE* temp;
    double len_y;
    if(!CRef_p2_scale::new_element(fpr,&temp,Table)) return false;
    if(!Data_IO::get_double(len_y,fpr)) return false;
    *re_pointer = new CRef_p2_fix(len_y,dynamic_cast<CRef_p2_scale*>(temp));
    delete temp;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////


bool PointMov::open_file(FILE* fpr,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    Point temp;
    if(!Data_IO::get_point(temp,fpr))
        re_bool = false;
    if(!re_bool)
    {
        set_state(SimState::OpenError);
        return re_bool;
    }

    ini_value(temp);
    ConditionMov* temp_pointer = NULL;
    re_bool = ConditionMov::new_element(fpr,&temp_pointer,Table);
    if(!re_bool)
    {
        set_state(SimState::OpenError);
        return re_bool;
    }
    set_condition( unique_ptr<ConditionMov>(temp_pointer) );
    clear_state(SimState::OpenError);
    return re_bool;
}

bool ConditionMov::new_element(FILE* fpr,ConditionMov** re_pointer,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    re_pointer = NULL;
    int id = -1;
    if(!Data_IO::get_int(id,fpr))
    {
        printf("there is a id error in the ConditionRot");
        return false;
    }
    switch(id)
    {
        case Condition::CMOV_P1:
        {
            re_bool = CMov_p1::new_element(fpr, re_pointer,Table);
            break;
        }
        case Condition::CMOV_P2:
        {
            re_bool = CMov_p2::new_element(fpr, re_pointer,Table);
            break;
        }
        default:
            re_bool = false;
            break;
    }
    if(re_bool)
        return re_bool;
    else
    {
        printf("Read file error in the ConditionSE id = %d\n",id);
        return re_bool;
    }
}

void CMov_p1::save_file(FILE* fpw,bool is_id = true) const
{
    if(is_id) Data_IO::save_int(Condition::CMOV_P1,fpw);
    point_len->save_index(fpw);
    point_dire->save_index(fpw);
    Data_IO::save_double(len,fpw);
    Data_IO::save_bool(is_left,fpw);
}

bool CMov_p1::new_element(FILE* fpr,ConditionMov** re_pointer,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    //////////////////////////////
    double len;
    const PointBC* point_len;
    const PointBC* point_dire;
    bool is_left;
    //////////////////////////////
    Index2 P;
    re_bool = PointBC::get_index(P,fpr);	if(!re_bool) return false;
    point_len = Table[P[0]][P[1]];

    re_bool = PointBC::get_index(P,fpr);	if(!re_bool) return false;
    point_dire = Table[P[0]][P[1]];

    re_bool = Data_IO::get_double(len,fpr);	if(!re_bool) return false;
    re_bool = Data_IO::get_bool(is_left,fpr);	if(!re_bool) return false;
    //////////////////////////////
    *re_pointer = new CMov_p1(len,point_len,point_dire,is_left);
    return true;
}

void CMov_p2::save_file(FILE* fpw,bool is_id = true) const
{
    if(is_id) Data_IO::save_int(Condition::CMOV_P1,fpw);
    point_1->save_index(fpw);
    point_2->save_index(fpw);
    Data_IO::save_double(L1,fpw);
    Data_IO::save_double(L2,fpw);
}

bool CMov_p2::new_element(FILE* fpr,ConditionMov** re_pointer,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    //////////////////////////////
    double L1;
    double L2;
    const PointBC* point_1;
    const PointBC* point_2;
    //////////////////////////////
    Index2 P;
    re_bool = PointBC::get_index(P,fpr);	if(!re_bool) return false;
    point_1 = Table[P[0]][P[1]];

    re_bool = PointBC::get_index(P,fpr);	if(!re_bool) return false;
    point_2 = Table[P[0]][P[1]];

    re_bool = Data_IO::get_double(L1,fpr);	if(!re_bool) return false;
    re_bool = Data_IO::get_double(L2,fpr);	if(!re_bool) return false;
    //////////////////////////////
    *re_pointer = new CMov_p2(L1,point_1,L2,point_2);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////


bool PointRot::open_file(FILE* fpr,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    Point temp;
    if(!Data_IO::get_point(temp,fpr))
        re_bool = false;
    if(!re_bool)
    {
        set_state(SimState::OpenError);
        return re_bool;
    }

    ini_value(temp);
    ConditionRot* temp_pointer = NULL;
    re_bool = ConditionRot::new_element(fpr,&temp_pointer,Table);
    if(!re_bool)
    {
        set_state(SimState::OpenError);
        return re_bool;
    }
    set_condition( unique_ptr<ConditionRot>(temp_pointer) );
    clear_state(SimState::OpenError);
    return re_bool;
}

bool ConditionRot::new_element(FILE* fpr,ConditionRot** re_pointer,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = true;
    re_pointer = NULL;
    int id = -1;
    if(!Data_IO::get_int(id,fpr))
    {
        printf("there is a id error in the ConditionRot");
        return false;
    }
    switch(id)
    {
        case Condition::CROT_POINT_LINEAR:
        {
            re_bool = CRot_point_linear::new_element(fpr, re_pointer,Table);
            break;
        }
        default:
            re_bool = false;
            break;
    }
    if(re_bool)
        return re_bool;
    else
    {
        printf("Read file error in the ConditionRot id = %d\n",id);
        return re_bool;
    }
}

void ConditionRot::set_save_file(FILE* fpw) const
{
    center_point->save_index(fpw);
    Data_IO::save_double(step_angle,fpw);
    Data_IO::save_double(length,fpw);
}

void CRot_point_linear::save_file(FILE* fpw,bool is_id = true) const
{
    if(is_id) Data_IO::save_int(Condition::CROT_POINT_LINEAR,fpw);
    ConditionRot::set_save_file(fpw);
}

bool ConditionRot::ini_ConditionRot(FILE* fpr,const PointBC** center_point,double& step_angle,double& length,const vector<vector<PointBC*>>& Table)
{
    bool re_bool = false;

    Index2 P;
    re_bool = PointBC::get_index(P,fpr);	if(!re_bool) return false;
    *center_point = Table[P[0]][P[1]];

    re_bool = Data_IO::get_double(step_angle,fpr);	if(!re_bool) return false;
    re_bool = Data_IO::get_double(length,fpr);	if(!re_bool) return false;

    return true;
}

bool CRot_point_linear::new_element(FILE* fpr,ConditionRot** re_pointer,const vector<vector<PointBC*>>& Table)
{
    const PointBC* center_point;
    double step_angle;
    double length;
    if(!ConditionRot::ini_ConditionRot(fpr,&center_point,step_angle,length,Table)) return false;
    ////////////////////////////
    *re_pointer = new CRot_point_linear(center_point, step_angle, length);
    return true;
}






