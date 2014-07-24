#ifndef HJ_DEF_CTRL_PENALTY_H_
#define HJ_DEF_CTRL_PENALTY_H_

#include <hjlib/math_func/math_func.h>
#include <zjucad/matrix/matrix.h>


#include <hjlib/util/hrclock.h>


typedef double  FLT;
typedef int32_t INT;

typedef zjucad::matrix::matrix<FLT> matrixf_t;
typedef zjucad::matrix::matrix<INT> matrixi_t;
typedef hj::math_func::math_func_t<FLT, INT> func_t;

using namespace std;
using namespace zjucad::matrix;
using namespace hj::math_func;



class position_difference_ : public func_t
{
public:
  //! ref to internal
  position_difference_(size_t n, const matrixi_t &idx, const matrixf_t &pos, double weight)
    :nodes_(pos), w_(weight) {
    if(idx.size() != pos.size()) { // per point
      nx_ = n*pos.size(1);
      idx_.resize(idx.size()*nodes_.size(1));
      for(size_t i = 0; i < idx.size(); ++i) {
        for(size_t d = 0; d < nodes_.size(1); ++d)
          idx_[i*nodes_.size(1)+d] = idx[i]*nodes_.size(1)+d;
      }
    }
    else { // per variable
      nx_ = n;
      idx_ = idx;
    }
  }

  typedef FLT value_type;
  typedef INT int_type;

  virtual size_t nx(void) const { return nx_; }

  virtual size_t nf(void) const { return nodes_.size(); }

    virtual int eval(size_t k, const value_type *x,
                   const coo2val_t<value_type, int_type> &cv,
                   hj::math_func::func_ctx *ctx = 0) const
  {
    if(k == 0) {
      for(int_type i = 0; i < idx_.size(); ++i) {
        int_type c[] = {i};
        cv[c] += (x[idx_[i]]-nodes_[i])*w_;
      }
    }

    if(k == 1) {
      for(int_type i = 0; i < idx_.size(); ++i) {
        int_type c[] = {i, idx_[i]};
        cv[c] += w_;
      }
    }
    return 0;
  }
    virtual int patt(size_t k, coo_set<int_type> &cs, const coo_l2g &l2g, hj::math_func::func_ctx *ctx = 0) const
  {
    if(k == 1) {
      for(int_type i = 0; i < idx_.size(); ++i) {
        int_type c[] = {i, idx_[i]};
        l2g.add(cs, c);
      }
    }
    return 0;
  }
    virtual size_t nnz(size_t k) const {
    if(k == 0)
      return -1;
    if(k == 1)
      return idx_.size();
  }
public:
  matrixi_t idx_;
  const matrixf_t &nodes_;
  size_t nx_;
  const double w_;
};


position_difference_ *build_dc_pen_diff_(size_t n, const matrixi_t &idx, const matrixf_t &pos, double weight);


#endif
