#ifndef __CJ_DEFORMER_H_
#define __CJ_DEFORMER_H_

#include <zjucad/matrix/matrix.h>
#include <hjlib/sparse/sparse.h>
#include <hjlib/math_func/math_func.h>
#include <boost/property_tree/ptree.hpp>
#include "func2opt.h"
#include "pos_pen.h"

using namespace std;

class deformer {

public :
    typedef zjucad::matrix::matrix<size_t> matrixst;
    typedef zjucad::matrix::matrix<double> matrixd;
    typedef std::vector<std::shared_ptr<hj::math_func::math_func>> funcs_t;

    deformer(const matrixst &t, const matrixd &r);
    deformer(const matrixst &t, const matrixd &r,
             const matrixd  &init_emb_node,
             const map<size_t, vector<pair<size_t, string>>> &regions);
    deformer(const matrixst &t, const matrixd &r,
             const matrixd  &init_emb_node,
             const vector<vector<pair<size_t, double>>>  &w_mat);

    bool deform(matrixd &x);
    bool deform(matrixd &x, const matrixd &curr_emb_node);
    void reset(matrixd &curr) {
        curr = r_;
    }

protected :
    std::shared_ptr<funcs_t>     shell_funcs_;
    shared_ptr<vector<double>>   w_;
    std::shared_ptr<func2opt>    for_opt_;
    boost::property_tree::ptree  pt_;
    matrixst                     t_;
    matrixd                      r_;
    vector<vector<pair<size_t, double>>> clt_w_;
    shared_ptr<position_difference>      pos_diff_;
};

typedef shared_ptr<deformer> pDeformer;
#endif
