#ifndef HJ_FUNC_OPT_H_
#define HJ_FUNC_OPT_H_

#include <boost/property_tree/ptree.hpp>
#include <jtflib/function/function.h>

namespace jtf {

class opt
{
public:
  static opt* create(const char *name);

  opt():cb_(0){}
  virtual ~opt(){}

  class callbacks
  {
  public:
    virtual ~callbacks(){}
    //! @return non-zero means break the optimization
    virtual int at_point(const double *x) = 0;
  };
  virtual void set_callbacks(callbacks *cb) { cb_ = cb; }

  virtual int set_f(jtf::function::functionN1_t<double,int32_t> &f) = 0;
  virtual int set_c(jtf::function::functionN1_t<double,int32_t> *c) {return 1;}
  virtual int solve(double *x, boost::property_tree::ptree &pt) = 0;
protected:
  callbacks *cb_;
};

}

#endif
