#ifndef __CJ_SQP_H__
#define __CJ_SQP_H__

#include <jtflib/function/function.h>
#include <zjucad/matrix/matrix.h>
#include <boost/property_tree/ptree.hpp>
//#include <zjucad/optimizer/linear_solver.h>
#include "linear_solver.h"


/**
 * TODO : replace the linear_solver with my own version
 * TIPS : my linear solver should has same interface with zjcuad version
 * TIPS : implement linear solver using PETsc
 */

namespace cj {

class SQP {
public :
    SQP();
    virtual int set_f(const jtf::function::functionN1_t<double, int32_t> &f);
    virtual int solve(double *x, boost::property_tree::ptree &pt);

protected :
    const jtf::function::functionN1_t<double, int32_t> *f_;
    std::unique_ptr<linear_solver> slv_;
    hj::sparse::csc<double, int32_t> H_, corrected_H_;
    zjucad::matrix::matrix<double> g_, s_, D_, tmp_;
};


int optimize(const jtf::function::functionN1_t<double,int32_t> &f,
             zjucad::matrix::matrix<double>                    &x,
             boost::property_tree::ptree                       &pt);

}











#endif
