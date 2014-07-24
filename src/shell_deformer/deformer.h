#ifndef __CJ_DEFORMER_H_
#define __CJ_DEFORMER_H_

#include <zjucad/matrix/matrix.h>
#include <hjlib/sparse/sparse.h>
#include <hjlib/math_func/math_func.h>
#include <boost/property_tree/ptree.hpp>
#include "func2opt.h"
#include "pos_pen.h"
#include "spring_potential.h"
#include <Eigen/CholmodSupport>
#include <Eigen/SPQRSupport>

class shell_sqp;

typedef zjucad::matrix::matrix<size_t> matrixst;
typedef zjucad::matrix::matrix<double> matrixd;
typedef std::vector<std::shared_ptr<hj::math_func::math_func>> funcs_t;

class deformer {
public :
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
    shared_ptr<shell_sqp>   sqp_solver_;
};

typedef shared_ptr<deformer> pDeformer;

class quad_deformer {

public :
    quad_deformer(const matrixst &tris, const matrixd &nods,
                  const matrixd  &init_emd_nods,
                  const map<size_t, vector<pair<size_t, string>>> &regions);
    int deform(matrixd &x, const matrixd &curr_emb_nods);
    void reset(matrixd &curr) { curr = nods_; }

private :
    shared_ptr<funcs_t>                  shell_funcs_;
    shared_ptr<vector<double>>           w_;
    shared_ptr<func2opt>                 for_opt_;

    shared_ptr<position_difference>      pos_diff_;
    shared_ptr<spring_potential>         spring_;

    matrix<size_t>                       tris_;
    matrix<double>                       nods_;
    vector<vector<pair<size_t, double>>> clt_w_;

    Eigen::SparseMatrix<double>                                   HES_;
//    Eigen::CholmodSimplicialLDLT<Eigen::SparseMatrix<double>>     solver_;
    Eigen::SPQR<Eigen::SparseMatrix<double>> solver_;
};


#endif
