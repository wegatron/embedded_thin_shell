#ifndef HJ_FUNC_TO_OPT_H_
#define HJ_FUNC_TO_OPT_H_

#include <jtflib/function/function.h>
#include <hjlib/math_func/math_func.h>

class func2opt : public jtf::function::functionN1_t<double,int32_t>
{
public:
  func2opt(const std::shared_ptr<const hj::math_func::math_func_t<double, int32_t> > &f);

  virtual size_t dim(void) const;
  virtual int val(const double *x, double &v);
  virtual int gra(const double *x, double *g);
  virtual int hes(const double *x, size_t &nnz, size_t &format, double *h,
                  int32_t *ptr, int32_t *idx, double alpha = 1);

  virtual int gra(const double *x, size_t &nnz, double *g, int32_t *idx)
  { assert(0); return -1; }
  virtual int hes_block(const double *x, double *h, double alpha=1)
  { assert(0); return -1; }
private:
  const std::shared_ptr<const hj::math_func::math_func_t<double, int32_t> > f_;
  std::shared_ptr<hj::math_func::coo_pat<int32_t> > cp_[3];
};

#endif
