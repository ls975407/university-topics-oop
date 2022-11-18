#ifndef POINT_H
#define POINT_H

#include "point_base.h"
#include <vector>
#include <typeinfo>
#include <cstring>
#include <memory>
using namespace std;

class SimState
{
public:
    static const size_t SimError;
    static const size_t CondError;
    static const size_t Cond2Loop;
    static const size_t OpenError;

    static const char* SimError_Word;
    static const char* CondError_Word;
    static void print(size_t flag);
};

class Data_IO
{
public:
    static bool get_double(double&,FILE*);
    static void save_double(double,FILE*);

    static bool get_int(int&,FILE*);
    static void save_int(int,FILE*);

    static bool get_index(size_t&,FILE*);
    static void save_index(size_t,FILE*);

    static bool get_bool(bool&,FILE*);
    static void save_bool(bool,FILE*);

    static bool get_point(Point&,FILE*);
    static void save_point(const Point&,FILE*);

    static void print_next(FILE*);
};

class Index2
{
protected:
    size_t index_1;
    size_t index_2;
public:
    Index2();
    Index2(size_t,size_t);
    static bool get_index(Index2&,FILE*);
    void save_index(FILE*) const;
    size_t operator[](int i) const;
};

class ConditionMov;
class ConditionSE;
class ConditionRot;

class StateObj
{
protected:
    size_t state;
protected:
    void set_state(size_t flag);
    void clear_state(size_t flag);
    void reset_state();
public:
    size_t get_state(size_t flag=-1);
};

class PointBC: public StateObj, public Index2
{
public:
    static bool get_index(Index2&,FILE*);
//member data
private:
    Point old_point;
public:
    enum{ FIX, MOV, ROT, REF};
    Point new_point;
protected:
    bool is_cal;
    vector<PointBC*> v_rel_points;
//member function
//virtual function
public:
    virtual int get_type() const = 0;
    virtual bool cal_condition() = 0;
    virtual bool check_condition() = 0;
    virtual bool check_answer() const = 0;
    virtual PointBC* copy_pointbc() = 0;
    virtual void recalculate_condition_coef() = 0;
    virtual ~PointBC();
public:
    virtual void save_file(FILE*) const = 0;
    virtual bool open_file(FILE*,const vector<vector<PointBC*>>&) = 0;


protected:
    void set_cal_T();
    void print_id();

public:
    PointBC();
    PointBC(const Point& p);
    PointBC(const double x, const double y);
    bool if_cal() const;
    const Point get_position() const;
    const Point get_remain_position() const;
    int X() const;
    int Y() const;

    void ini_value(const Point& p);

    void update();
    void rev_update();
    void set_cal_F();
//motion function
public:
    virtual void move_position(const Point&) = 0;
//setting function
    void set_rel_point(PointBC*);
    void del_rel_point(PointBC*);
    void replace_rel_point(PointBC*,PointBC*);
public:
    virtual void reset_condition() = 0;
    virtual void erase() = 0;
};

/////////////////////////////////////////////////////////

class PointSE: public PointBC
{
//member data
private:
    //ConditionSE* condition;
    shared_ptr<ConditionSE> condition;
//member function
public:
    PointSE();
    PointSE(const Point& p);
    PointSE(const double x, const double y);
    ~PointSE();
//virtual function
public:
    //void set_condition(class ConditionSE* cond);
    void set_condition(unique_ptr<ConditionSE> cond);
    void reset_condition(); //virtaul function
    void erase(); //virtual function
    bool check_condition();
    bool check_answer() const;
    void recalculate_condition_coef();
    void recalculate_coef();
    PointBC* copy_pointbc();
    bool cal_condition();
    int get_type() const;
public:
    void save_file(FILE*) const;
    bool open_file(FILE*,const vector<vector<PointBC*>>&);

//motion function
public:
    void move_position(const Point&);
//reset relationship
    //no setting function
};

/////////////////////////////////////////////////////////

class PointMov: public PointBC
{
//member data
private:
    //ConditionMov* condition;
    shared_ptr<ConditionMov> condition;
//member function
public:
    PointMov();
    PointMov(const Point& p);
    PointMov(const double x, const double y);
    ~PointMov();
    void set_condition(unique_ptr<ConditionMov> cond);
    void reset_condition(); //virtaul function
    void erase(); //virtual function
    bool check_condition();
    bool check_answer() const;
    void recalculate_condition_coef();
    PointBC* copy_pointbc();
    bool cal_condition();
public:
    void save_file(FILE*) const;
    bool open_file(FILE*,const vector<vector<PointBC*>>&);
//virtual function
public:
    int get_type() const;
//motion function
public:
    void move_position(const Point&);
//reset relationship
    void setPoint_L(PointBC*, PointBC*);
    void setPoint_R(PointBC*, PointBC*);
};

class PointRot: public PointBC
{
//member data
private:
    //ConditionRot* condition;
    shared_ptr<ConditionRot> condition;
//member function
public:
    PointRot();
    PointRot(const Point& p);
    PointRot(const double x, const double y);
    ~PointRot();
    void set_condition(unique_ptr<ConditionRot> cond);
    void reset_condition(); //virtaul function
    void erase(); //virtual function
    bool check_condition();
    bool check_answer() const;
    void recalculate_condition_coef();
    PointBC* copy_pointbc();
    bool cal_condition();
public:
    void save_file(FILE*) const;
    bool open_file(FILE*,const vector<vector<PointBC*>>&);
//virtual function
public:
    int get_type() const;
//motion function
public:
    void move_position(const Point&);
//reset relationship
    void setPoint(PointBC*, PointBC*);
};

//////////////////////////////////////////////////
enum Condition
{
    FIX_POSITION=1,
    CREF_P2_SCALE,
    CREF_P2_FIX,
    ///////////////////
    CMOV_P1=11,
    CMOV_P2,
    ///////////////////
    CROT_POINT_LINEAR=21
};

class ConditionSE
{
private:
    bool must_cal;
protected:
    PointSE* pointer; //to access the past position data
    const PointBC* point_0;
    const PointBC* point_1;
public:
    virtual bool calculate() const = 0;
    virtual void recalculate_coef() = 0;
    virtual bool check_answer() const = 0;
    ConditionSE(PointSE*, const PointBC*, const PointBC*);
    virtual ~ConditionSE();
    const PointBC* getPoint_0() const;
    const PointBC* getPoint_1() const;
public:
    virtual void save_file(FILE*,bool=true) const = 0;
    static bool new_element(FILE* ,ConditionSE**,const vector<vector<PointBC*>>&);
public:
    void set_cal_T();
    void set_cal_F();
    bool if_must_cal() const;
};

class ConditionMov
{
public:
    virtual bool calculate(Point& new_point, const Point& old_position) const = 0;
    virtual ~ConditionMov();
public:
    virtual void set_is_left(bool is_l) = 0;
    virtual void setPoint_L(const PointBC*,const PointBC*) = 0;
    virtual void setPoint_R(const PointBC*,const PointBC*) = 0;
    virtual const PointBC* getPoint_L() const = 0;
    virtual const PointBC* getPoint_R() const = 0;
    virtual void recalculate_coef(const Point&) = 0;
    virtual bool check_answer(const Point&) const = 0;
public:
    virtual void save_file(FILE*,bool=true) const = 0;
    static bool new_element(FILE* ,ConditionMov**,const vector<vector<PointBC*>>&);
};

class ConditionRot
{
protected:
    const PointBC* center_point;
    double step_angle;
    double length;
    static bool ini_ConditionRot(FILE* fpr,const PointBC** center_point,double& step_angle,double& length,const vector<vector<PointBC*>>& Table);
public:
    void setPoint(const PointBC*);
    const PointBC* getPoint() const;
    void setDirection(bool is_counterwise);
public:
    ConditionRot(const PointBC*,double,double);
    virtual bool calculate(Point& new_point, const Point& old_position) const = 0;
    virtual ~ConditionRot();
    virtual void recalculate_coef(const Point&) = 0;
    virtual bool check_answer(const Point&) const = 0;
public:
    virtual void save_file(FILE*,bool=true) const = 0;
    void set_save_file(FILE*) const;
    static bool new_element(FILE* ,ConditionRot**,const vector<vector<PointBC*>>&);
};

//////////////////////////////////////////////

class CRef_p2_scale: public ConditionSE
{
protected:
    CRef_p2_scale(double l, PointSE* np, const PointBC* p0, const PointBC* p1);
protected:
    double len_x;
public:
    CRef_p2_scale(PointSE* np, const PointBC* p0, const PointBC* p1);
    bool calculate() const;
    void recalculate_coef();
    bool check_answer() const;
public:
    void save_file(FILE*,bool) const;
    static bool new_element(FILE* ,ConditionSE**,const vector<vector<PointBC*>>&);
};

class CRef_p2_fix: public CRef_p2_scale
{
protected:
    CRef_p2_fix(double lx,double ly, PointSE* np, const PointBC* p0, const PointBC* p1);
    CRef_p2_fix(double ly, const CRef_p2_scale* pc);
private:
    double len_y;
public:
    CRef_p2_fix(PointSE* np, const PointBC* p0, const PointBC* p1);
    bool calculate() const;
    void recalculate_coef();
    bool check_answer() const;
public:
    void save_file(FILE*,bool) const;
    static bool new_element(FILE* ,ConditionSE**,const vector<vector<PointBC*>>&);
};

//////////////////////////////////////////////

class CMov_p1: public ConditionMov
{
private:
    double len;
    const PointBC* point_len;
    const PointBC* point_dire;
    bool is_left;
private:
    void setLen(const PointBC* new_point,const PointBC* old_point);
    void setDirection(const PointBC* new_point,const PointBC* old_point);
public:
    CMov_p1(double l, const PointBC* pl, const PointBC* pd, bool is_l);
    bool calculate(Point& new_point, const Point& old_position) const;
    void set_is_left(bool is_l);
    void setPoint_L(const PointBC*,const PointBC*);
    void setPoint_R(const PointBC*,const PointBC*);
    const PointBC* getPoint_L() const;
    const PointBC* getPoint_R() const;
    void recalculate_coef(const Point&);
    bool check_answer(const Point&) const;
public:
    void save_file(FILE*,bool) const;
    static bool new_element(FILE* ,ConditionMov**,const vector<vector<PointBC*>>&);
};

class CMov_p2: public ConditionMov
{
private:
    double L1;
    double L2;
    const PointBC* point_1;
    const PointBC* point_2;
private:
    void setLen
      (
        const PointBC*& point_target, double& len_target,
        const PointBC* new_point, const PointBC* old_point
      );
public:
    CMov_p2(const double l1, const PointBC* p1, const double l2, const PointBC* p2);
    bool calculate(Point& new_point, const Point& old_position) const;
    void set_is_left(bool is_l);
    void setPoint_L(const PointBC*,const PointBC*);
    void setPoint_R(const PointBC*,const PointBC*);
    const PointBC* getPoint_L() const;
    const PointBC* getPoint_R() const;
    void recalculate_coef(const Point&);
    bool check_answer(const Point&) const;
public:
    void save_file(FILE*,bool) const;
    static bool new_element(FILE* ,ConditionMov**,const vector<vector<PointBC*>>&);
};

//////////////////////////////////////////////

class CRot_point_linear: public ConditionRot
{
public:
    CRot_point_linear(const PointBC* p, double angle, double len);
    bool calculate(Point& new_point, const Point& old_position) const;
    void recalculate_coef(const Point&);
    bool check_answer(const Point&) const;
public:
    void save_file(FILE*,bool) const;
    static bool new_element(FILE* ,ConditionRot**,const vector<vector<PointBC*>>&);
};




#endif
