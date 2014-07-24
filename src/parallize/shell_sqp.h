#ifndef __CJ_SHELL_SQP_H__
#define __CJ_SHELL_SQP_H__

#include <jtflib/function/function.h>
#include <zjucad/matrix/matrix.h>
#include <boost/property_tree/ptree.hpp>
#include "linear_solver.h"

class shell_sqp {
    typedef jtf::function::functionN1_t<double, int32_t> jtf_func;
public :
    shell_sqp(const shared_ptr<jtf_func>  &f,            //total energy
              const shared_ptr<jtf_func>  &f_spring,     //spring
              const shared_ptr<jtf_func>  &f_bend,       //bending
              const shared_ptr<jtf_func>  &f_pos,        //position constraints
              const double                w1,
              const double                w2,
              const double                w3,
              const double                *x);
    virtual int solve(double *x, boost::property_tree::ptree &pt);

protected :
    std::shared_ptr<jtf_func>           f_;
    std::shared_ptr<jtf_func>           fs_, fb_, fp_;
    const double                        ws_, wb_, wp_;
    hj::sparse::csc<double, int32_t>    Hs_, Hb_, Hp_;
    Eigen::SparseMatrix<double>         const_hes_;

    std::unique_ptr<cj::linear_solver>  slv_;
    hj::sparse::csc<double, int32_t>    H_, corrected_H_;
    zjucad::matrix::matrix<double>      g_, s_, D_, tmp_;
};

#endif
