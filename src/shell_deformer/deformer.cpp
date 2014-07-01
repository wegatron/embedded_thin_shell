#include "deformer.h"

#include <fstream>
#include <hjlib/shell/shell.h>
#include <zjucad/ptree/ptree.h>
#include <hjlib/math_func/func_aux.h>
#include <hjlib/math_func/operation.h>
#include <hjlib/util/hrclock.h>
#include <jtflib/optimizer/optimizer.h>
// #include "../parallize/linear_solver.h"
// #include "../parallize/SQP.h"
#include "cluster.h"


using namespace std;
using namespace zjucad::matrix;
using namespace hj::math_func;

typedef std::vector<std::shared_ptr<hj::math_func::math_func>> funcs_t;
typedef zjucad::matrix::matrix<size_t> matrixst;
typedef zjucad::matrix::matrix<double> matrixd;
typedef hj::math_func::math_func_t<double, int32_t> func_t;


//#define __CJ_SQP__
#define CJ_LINEAR_TYPE "PETsc"


deformer::deformer(const matrixst &t, const matrixd &r)
    :t_(t), r_(r), shell_funcs_(new funcs_t(2))
{
    {
        shared_ptr<hj::math_func::math_func> f
          (hj::shell::build_dihedral_angle_change(r_, t_));
        (*shell_funcs_)[1].reset(new sumsqr<FLT, INT>(f));
    }
    {
        shared_ptr<hj::math_func::math_func> f
                (hj::shell::build_edge_length_square2(r_, t_, 'M'));
        (*shell_funcs_)[0].reset(new hj::math_func::sum<double, int32_t>(f));
    }

    w_ = shared_ptr<vector<FLT>>(new vector<FLT>(2));
    (*w_)[0] = 1e1;
    (*w_)[1] = 1e-4;
    shared_ptr<func_t> all_funcs(new fcat<FLT, INT, funcs_t>(shell_funcs_));
    shared_ptr<func_t> energy(new hj::math_func::sum<FLT, INT>(all_funcs, w_));
    for_opt_.reset(new func2opt(energy));

    pt_.put("package.value", "jtf");
    pt_.put("alg.value", "SQP");
    pt_.put("iter.value", 1);
    pt_.put("GS_LM_mu.value", 1e-14);

#ifdef __CJ_SQP__
    pt_.put("linear_solver/type.value", CJ_LINEAR_TYPE);
#else
    pt_.put("linear_solver/type.value", "direct");
    pt_.put("linear_solver/name.value", "cholmod");
#endif
}


deformer::deformer(const matrixst &t, const matrixd &r,
                   const matrixd  &init_emb_nodes,
                   const map<size_t, vector<pair<size_t, string>>> &regions)
    :t_(t), r_(r), shell_funcs_(new funcs_t(3))
{
    calc_point_weight(init_emb_nodes, regions, clt_w_);
    {
        shared_ptr<hj::math_func::math_func> f
          (hj::shell::build_dihedral_angle_change(r_, t_));
        (*shell_funcs_)[2].reset(new sumsqr<FLT, INT>(f));
    }
    {
        shared_ptr<hj::math_func::math_func> f
                (hj::shell::build_edge_length_square2(r_, t_, 'M'));
        (*shell_funcs_)[1].reset(new hj::math_func::sum<double, int32_t>(f));
    }
    {
        pos_diff_.reset(build_pos_pen_diff(init_emb_nodes, clt_w_));
        shared_ptr<hj::math_func::math_func> f(pos_diff_);
        (*shell_funcs_)[0].reset(new hj::math_func::sumsqr<FLT, INT>(f));
    }

//    {
//        matrix<double> idx(r_.size(2), 1);
//        for (size_t i = 0; i < idx.size(); ++i)
//            idx[i] = i;

//        pos_diff_.reset(build_dc_pen_diff(r_.size(2), idx, init_emb_nodes));
//        shared_ptr<hj::math_func::math_func> f(pos_diff_);
//        (*shell_funcs_)[0].reset(new hj::math_func::sumsqr<FLT, INT>(f));
//    }

    w_.reset(new vector<FLT>(3));
    (*w_)[0] = 1e4;
    (*w_)[1] = 1e1;
    (*w_)[2] = 1e-4;

    shared_ptr<func_t> all_funcs(new fcat<FLT, INT, funcs_t>(shell_funcs_));
    shared_ptr<func_t> energy(new hj::math_func::sum<FLT, INT>(all_funcs, w_));
    for_opt_.reset(new func2opt(energy));

    pt_.put("package.value", "jtf");
    pt_.put("alg.value", "SQP");
    pt_.put("iter.value", 1);
    pt_.put("GS_LM_mu.value", 1e-14);

#ifdef __CJ_SQP__
    pt_.put("linear_solver/type.value", CJ_LINEAR_TYPE);
#else
    pt_.put("linear_solver/type.value", "direct");
    pt_.put("linear_solver/name.value", "cholmod");
#endif

}

deformer::deformer(const matrixst &t, const matrixd &r,
                   const matrixd  &init_emb_nodes,
                   const vector<vector<pair<size_t, double>>> &w_mat)
    :t_(t), r_(r), shell_funcs_(new funcs_t(3)), clt_w_(w_mat)
{
    {
        shared_ptr<hj::math_func::math_func> f
          (hj::shell::build_dihedral_angle_change(r_, t_));
        (*shell_funcs_)[2].reset(new sumsqr<FLT, INT>(f));
    }
    {
        shared_ptr<hj::math_func::math_func> f
                (hj::shell::build_edge_length_square2(r_, t_, 'M'));
        (*shell_funcs_)[1].reset(new hj::math_func::sum<double, int32_t>(f));
    }
    {
        pos_diff_.reset(build_pos_pen_diff(init_emb_nodes, clt_w_));
        shared_ptr<hj::math_func::math_func> f(pos_diff_);
        (*shell_funcs_)[0].reset(new hj::math_func::sumsqr<FLT, INT>(f));
    }

    w_.reset(new vector<FLT>(3));
    (*w_)[0] = 1e4;
    (*w_)[1] = 1e1;
    (*w_)[2] = 1e-4;

    shared_ptr<func_t> all_funcs(new fcat<FLT, INT, funcs_t>(shell_funcs_));
    shared_ptr<func_t> energy(new hj::math_func::sum<FLT, INT>(all_funcs, w_));
    for_opt_.reset(new func2opt(energy));

    pt_.put("package.value", "jtf");
    pt_.put("alg.value", "SQP");
    pt_.put("iter.value", 1);
    pt_.put("GS_LM_mu.value", 1e-14);

#ifdef __CJ_SQP__
    pt_.put("linear_solver/type.value", CJ_LINEAR_TYPE);
#else
    pt_.put("linear_solver/type.value", "direct");
    pt_.put("linear_solver/name.value", "cholmod");
#endif
}

bool deformer::deform(matrixd &x)
{
    try {
        static hj::util::high_resolution_clock hrc;
        double start = hrc.ms();
#ifdef __CJ_SQP__
        cj::optimize(*for_opt_, x, pt_);
#else
        jtf::optimize(*for_opt_, x, pt_, nullptr, nullptr, nullptr);
#endif
        cout << "\nopt: " << hrc.ms() - start << endl;
    } catch ( std::exception &e ) {
        cerr << e.what() << endl;
        zjucad::show_usage_info(cerr, pt_);
    }
    return true;
}


bool deformer::deform(matrixd &x, const matrixd &curr_emb_node)
{
    pos_diff_->nodes_ = curr_emb_node;
    try {
        static hj::util::high_resolution_clock hrc;
        double start = hrc.ms();
#ifdef __CJ_SQP__
        cj::optimize(*for_opt_, x, pt_);
#else
        jtf::optimize(*for_opt_, x, pt_, nullptr, nullptr, nullptr);
#endif
        cout << "\nopt: " << hrc.ms() - start << endl;
    } catch ( std::exception &e ) {
        cerr << e.what() << endl;
        zjucad::show_usage_info(cerr, pt_);
    }
    return true;
}
