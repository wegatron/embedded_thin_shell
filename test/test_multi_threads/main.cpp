#include <iostream>
#include <metis.h>
#include <jtflib/mesh/io.h>
#include <src/conf_para.h>
#include <zjucad/matrix/matrix.h>
#include <boost/unordered_set.hpp>
#include "src/parallize/metis_decomposition.h"
#include "src/shell_deformer/deformer.h"
#include "common/mesh_operation.h"
#include "common/vtk.h"


#include "src/parallize/linear_solver.h"

using namespace std;
using namespace zjucad::matrix;

//int main(int argc, char *argv[])
//{
//    hj::sparse::csc<double, int32_t> A;
//    A.resize(3, 3, 9);
//    A.ptr().resize(4, 1);
//    A.val().resize(9, 1);
//    A.idx().resize(9, 1);

//    A.ptr()[0] = 0; A.ptr()[1] = 3; A.ptr()[2] = 6; A.ptr()[3] = 9;

//    A.val()[0] = 0.25; A.val()[1] = 0.01; A.val()[2] = 0.2;
//    A.val()[3] = 0.01; A.val()[4] = 0.05; A.val()[5] = 0.3;
//    A.val()[6] = 0.2; A.val()[7] = 0.3; A.val()[8] = 0.4;

//    A.idx()[0] = 0; A.idx()[1] = 1; A.idx()[2] = 2;
//    A.idx()[3] = 0; A.idx()[4] = 1; A.idx()[5] = 2;
//    A.idx()[6] = 0; A.idx()[7] = 1; A.idx()[8] = 2;

//    Eigen::Matrix<double, 3, 3> B;
//    B << 0.25,     0.01,   0.2,
//         0.01,    0.05,    0.3,
//         0.2,   0.3,    0.4;

//    EigenSolver<MatrixXd> es(B);
//    cout << es.eigenvalues() << endl;

//    boost::property_tree::ptree pt;
//    pt.put("linear_solver/type.value", "Jacobi");
//    cj::linear_solver* solver = cj::linear_solver::create(A, pt);
//    double x[3], b[3];
//    x[0] = 1; x[1] = 2; x[2] = 3;
//    b[0] = 0.46;  b[1] = 0.36; b[2] = 0.9;
//    solver->solve(&b[0], &x[0], 1, pt);

//    for (int i = 0; i < 3; ++i)
//        cout << x[i] << " ";
//    cout << endl;

//    return 0;


//}

int main(int argc, char *argv[])
{
    matrix<size_t> shell_mesh;
    matrix<double> shell_nodes;
    string inputPath = __POJ_BASE_PATH "dat/obj/cylinder/shell_mesh_000.obj";
    jtf::mesh::load_obj(inputPath.c_str(), shell_mesh, shell_nodes);

//    string inputPath = "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/obj/test/sprial.obj";
//    jtf::mesh::load_obj(inputPath.c_str(), shell_mesh, shell_nodes);


    vector<pPatch> _model;
    vector<pDeformer> _solver(__NODES_NUMBER);

    /* init _model */
    shared_ptr<decomposition> pDec(new decomposition(shell_mesh, shell_nodes, __NODES_NUMBER));
    pDec->generatePatches(_model);

//    char *outdir = "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result_1/";
//    pDec->writeAllPatches(outdir, "cylinder", _model);
//    for (size_t i = 0; i < _model.size(); ++i)
//        cout << _model[i]->mesh_.size(2) << endl;

    cout << "[INFO] load mesh done\n";

    /* init _solver */
#pragma omp parallel for
    for (size_t i = 0; i < _solver.size(); ++i)
        _solver[i].reset(new deformer(_model[i]->mesh_,  _model[i]->nodes_,
                                      _model[i]->nodes_, _model[i]->cons_));


    char emd_shell_path[200];
    matrix<size_t> emd_mesh;
    matrix<double> emd_nodes;

    size_t frame = 200;
    size_t curr = 0;

    while ( frame-- ) {
        cout << "\n\nthis is the " << curr << " frame.\n";
        sprintf(emd_shell_path, __POJ_BASE_PATH "dat/obj/cylinder/embd_mesh_%03d.obj", curr);
        jtf::mesh::load_obj(emd_shell_path, emd_mesh, emd_nodes);
        cout << "[INFO] load embedded mesh succeed.\n";

#pragma omp parallel for
        for (size_t i = 0; i < _solver.size(); ++i) {
            //load the reference embedded mesh of current patch
            matrix<double> emd_ref_nodes;
            emd_ref_nodes.resize(3, _model[i]->nodes_.size(2));
            for (size_t j = 0; j < _model[i]->nodes_.size(2); ++j) {
                // assert(_model[i]->s2g_.find(j) != nullptr);
                emd_ref_nodes(colon(), j) = emd_nodes(colon(), _model[i]->s2g_[j]);
            }
            _solver[i]->deform(_model[i]->nodes_, emd_ref_nodes);
        }

#pragma omp parallel for
        for (size_t i = 0; i < _model.size(); ++i)
            for (size_t j = 0; j < _model[i]->nodes_.size(2); ++j)
                shell_nodes(colon(), _model[i]->s2g_[j]) = _model[i]->nodes_(colon(), j);

        stringstream ss;
        ss << __POJ_BASE_PATH "result/test_multi_threads/cylinder_1threads_" << curr << ".vtk";
        ++curr;
        ofstream os(ss.str());
        tri2vtk(os, &shell_nodes[0], shell_nodes.size(2), &shell_mesh[0], shell_mesh.size(2));
    }

    cout << "[INFO] all done!\n";
    return 0;
}


