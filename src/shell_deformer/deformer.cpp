#include "deformer.h"

#include <fstream>
#include <hjlib/shell/shell.h>
#include <zjucad/ptree/ptree.h>
#include <hjlib/math_func/func_aux.h>
#include <hjlib/math_func/operation.h>
// #include <hjlib/util/hrclock.h>
#include <jtflib/optimizer/optimizer.h>
#include "../parallize/SQP.h"
#include "const_bending.h"
#include "cluster.h"
#include "../parallize/shell_sqp.h"
#include "sparse_transition.h"
#include <zjucad/matrix/io.h>

using namespace std;
using namespace zjucad::matrix;
using namespace hj::math_func;

typedef std::vector<std::shared_ptr<hj::math_func::math_func>> funcs_t;
typedef zjucad::matrix::matrix<size_t> matrixst;
typedef zjucad::matrix::matrix<double> matrixd;
typedef hj::math_func::math_func_t<double, int32_t> func_t;

#define __CJ_SQP__
#define CJ_LINEAR_TYPE "PETsc"
#define CONST_BENDING_HESSIAN

deformer::deformer(const matrixst &t, const matrixd &r)
    :t_(t), r_(r), shell_funcs_(new funcs_t(1))
{
//    {
//        shared_ptr<hj::math_func::math_func> f
//                (hj::shell::build_edge_length_square2(r_, t_, 'M'));
//        (*shell_funcs_)[0].reset(new hj::math_func::sum<double, int32_t>(f));
//    }

#ifdef CONST_BENDING_HESSIAN
    {
        shared_ptr<hj::math_func::math_func> f
                (build_quad_bending_energy(r_, t_));
        (*shell_funcs_)[0].reset(f.get());
    }
#else
    {
        shared_ptr<hj::math_func::math_func> f
          (hj::shell::build_dihedral_angle_change(r_, t_));
        (*shell_funcs_)[1].reset(new sumsqr<FLT, INT>(f));
    }
#endif

    w_ = shared_ptr<vector<FLT>>(new vector<FLT>(1));
    (*w_)[0] = 1;//1e1;
//    (*w_)[1] = 1;//1e-4
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
    shared_ptr<func2opt> fs, fp, fb;
    {
        pos_diff_.reset(build_pos_pen_diff(init_emb_nodes, clt_w_));
        shared_ptr<hj::math_func::math_func> f(pos_diff_);
        shared_ptr<hj::math_func::sumsqr<FLT, INT>> F
                            (new hj::math_func::sumsqr<FLT, INT>(f));
        fp.reset(new func2opt(F));
        (*shell_funcs_)[0] = F;
    }
    {
        shared_ptr<hj::math_func::math_func> f
                (hj::shell::build_edge_length_square2(r_, t_, 'M'));
        shared_ptr<hj::math_func::sum<FLT, INT>> F
                            (new hj::math_func::sum<FLT, INT>(f));
        fs.reset(new func2opt(F));
        (*shell_funcs_)[1] = F;
    }
#ifdef CONST_BENDING_HESSIAN
    {
        shared_ptr<func_t> f
                (build_quad_bending_energy(r_, t_));
        fb.reset(new func2opt(f));
        (*shell_funcs_)[2] = f;
    }
#else
    {
        shared_ptr<hj::math_func::math_func> f
          (hj::shell::build_dihedral_angle_change(r_, t_));
        (*shell_funcs_)[2].reset(new sumsqr<FLT, INT>(f));
    }
#endif

    w_.reset(new vector<FLT>(3));
    (*w_)[0] = 1e4;
    (*w_)[1] = 1e1;
    (*w_)[2] = 1e-7;

    shared_ptr<func_t> all_funcs(new fcat<FLT, INT, funcs_t>(shell_funcs_));
    shared_ptr<func_t> energy(new hj::math_func::sum<FLT, INT>(all_funcs, w_));
    for_opt_.reset(new func2opt(energy));

    sqp_solver_.reset(new shell_sqp(for_opt_, fs, fb, fp,
                                        (*w_)[1], (*w_)[2], (*w_)[0], &init_emb_nodes[0]));

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
        // static hj::util::high_resolution_clock hrc;
        // double start = hrc.ms();
#ifdef __CJ_SQP__
        cj::optimize(*for_opt_, x, pt_);
#else
        jtf::optimize(*for_opt_, x, pt_, nullptr, nullptr, nullptr);
        // cout << "\nopt: " << hrc.ms() - start << endl;
#endif
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
        // static hj::util::high_resolution_clock hrc;
        // double start = hrc.ms();
#ifdef __CJ_SQP__
//        cj::optimize(*for_opt_, x, pt_);
        sqp_solver_->solve(&x[0], pt_);
#else
        jtf::optimize(*for_opt_, x, pt_, nullptr, nullptr, nullptr);
        // cout << "\nopt: " << hrc.ms() - start << endl;
#endif
    } catch ( std::exception &e ) {
        cerr << e.what() << endl;
        zjucad::show_usage_info(cerr, pt_);
    }
    return true;
}

quad_deformer::quad_deformer(const matrixst &tris, const matrixd &nods,
                             const matrixd  &init_emd_nods,
                             const map<size_t, vector<pair<size_t, string>>> &regions)
    : nods_(nods), tris_(tris), shell_funcs_(new funcs_t(2))
{
    cout << "[INFO] the program is initializing a shell deformer\n";
    calc_point_weight(init_emd_nods, regions, clt_w_);
    {
        pos_diff_.reset(build_pos_pen_diff(init_emd_nods, clt_w_));
        shared_ptr<hj::math_func::math_func> f(pos_diff_);
        (*shell_funcs_)[0].reset(new hj::math_func::sumsqr<FLT, INT>(f));
    }
//    {
//        spring_.reset(build_spring_energy(nods_, tris_));
//        (*shell_funcs_)[1] = spring_;
//    }
    {
        (*shell_funcs_)[1].reset(build_quad_bending_energy(nods_, tris_));
    }

    w_.reset(new vector<FLT>(2));
    (*w_)[0] = 1e4; // 1e4;
    (*w_)[1] = 1e-7; // 1e1;
//    (*w_)[2] = 1e-7 //1e-7;

    shared_ptr<func_t> all_funcs(new fcat<FLT, INT, funcs_t>(shell_funcs_));
    shared_ptr<func_t> energy(new hj::math_func::sum<FLT, INT>(all_funcs, w_));
    for_opt_.reset(new func2opt(energy));

    size_t hes_nnz = 0;
    size_t format = -1;
    size_t dim = nods_.size();
    hj::sparse::csc<double, int32_t> H;
    for_opt_->hes(&nods_[0], hes_nnz, format, 0, 0, 0);
    H.resize(dim, dim, hes_nnz);
    for_opt_->hes(&nods_[0], hes_nnz, format, 0, &H.ptr()[0], &H.idx()[0]);
    H.val()(colon()) = 0;
    for_opt_->hes(&nods_[0], hes_nnz, format, &H.val()[0], &H.ptr()[0], &H.idx()[0]);

    vector<Eigen::Triplet<double>> vec;
    for (size_t j = 0; j < nods_.size(); ++j) {
        for (size_t cnt = H.ptr()[j]; cnt < H.ptr()[j + 1]; ++cnt) {
            size_t i = H.idx()[cnt];
            vec.push_back(Eigen::Triplet<double>(i, j, H.val()[cnt]));
        }
    }
    HES_.resize(nods_.size(), nods_.size());
    HES_.setFromTriplets(vec.begin(), vec.end());
    HES_.makeCompressed();

    //init solver and pre factorize
    solver_.compute(HES_);
    if ( solver_.info() != Success )
        cout << "[INFO] decomposition failed!\n";
    cout << "[INFO] shell deformer initialization done!\n";

    //    exit(0);
}

int quad_deformer::deform(matrixd &x, const matrixd &curr_emb_nods)
{
    hj::util::high_resolution_clock hrc;
    double start = hrc.ms();
    pos_diff_->nodes_ = curr_emb_nods;
    VectorXd x_(x.size()), g(x.size()), s(x.size());
    std::copy(x.begin(), x.end(), x_.data());

    for (size_t iter = 0; iter < 1; ++iter) {
//        spring_->update_d(x_);
        for_opt_->gra(x_.data(), g.data());
        s = solver_.solve(g);
        if ( solver_.info() != Success )
            cout << "[INFO] solving failed\n";
        x_ += -s;
    }

    std::copy(x_.data(), x_.data() + x_.size(), x.begin());
    cout << "[INFO] time cost for shell deformation : " << hrc.ms() - start << endl;
    return 0;
}
































