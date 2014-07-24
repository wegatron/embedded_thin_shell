#include <iostream>

#include <zjucad/matrix/matrix.h>
// #include <hjlib/util/hrclock.h>


#include "func2opt.h"

using namespace std;
using namespace hj::math_func;
using namespace zjucad::matrix;

func2opt::func2opt(const std::shared_ptr<const hj::math_func::math_func_t<double, int32_t> > &f)
  :f_(f)
{
  assert(f_->nf() == 1);
  for(size_t k = 0; k < 3; ++k)
    cp_[k].reset(patt<int32_t>(*f_, k));
  assert(cp_[0]->is_dense());
  assert(cp_[1]->is_dense());
  assert(!cp_[2]->is_dense());
}

size_t func2opt::dim(void) const
{
  return f_->nx();
}

int func2opt::val(const double *x, double &v) const
{
  f_->eval(0, x, coo2val(*cp_[0], &v));
}

int func2opt::gra(const double *x, double *g)
{
    // static hj::util::high_resolution_clock hrc;
    // double beg = hrc.ms();
  f_->eval(1, x, coo2val(*cp_[1], g));
    // cout << "\ngra: " << hrc.ms()-beg << endl;
}

int func2opt::hes(const double *x, size_t &nnz, size_t &format, double *h,
                  int32_t *ptr, int32_t *idx, double alpha)
{
  if(fabs(alpha- 1.0)) {
    cout << "no support for alpha." << endl;
    return -1;
  }
  format = 1;
  if(h == 0 && ptr == 0 && idx == 0) {// query nnz
    nnz = cp_[2]->nnz();
    return 0;
  }
  if(h == 0 && ptr != 0 && idx != 0) {// query patten
    int32_t leading[] = {0};
    coo2csc(*cp_[2], ptr, idx, leading);
    return 0;
  }
  if(h != 0 && ptr != 0 && idx != 0) {// accumulate
    // static hj::util::high_resolution_clock hrc;
    // double beg = hrc.ms();
    f_->eval(2, x, coo2val(*cp_[2], h));
    // cout << "\nhes: " << hrc.ms()-beg << endl;
    return 0;
  }
}
