#include <iostream>

#include <jtflib/mesh/io.h>
#include <hjlib/shell/shell.h>
#include <src/parallize/domain_decomposition.h>
#include <src/shell_deformer/cluster.h>
#include <src/shell_deformer/deformer.h>
#include <src/conf_para.h>
#include <common/vtk.h>
#include <omp.h>

using namespace zjucad::matrix;
using namespace std;


int main()
{
    /**
      * (1)load the initial surface.
      * (2)partition for position constraint.
      * (3)decompose the surface.
      * (4)get vector of patches.
      * (5)optimize the independent patches.
      * (6)assemble.
      **/

    matrix<size_t> shell_mesh;
    matrix<double> shell_nodes;
    string ref_shell_path = __POJ_BASE_PATH "dat/obj/cylinder/shell_mesh_000.obj";
    jtf::mesh::load_obj(ref_shell_path.c_str(), shell_mesh, shell_nodes);

    vector<vector<pair<size_t, double>>> w_mat;
    shared_ptr<cluster_machine> pCM(new cluster_machine(shell_mesh,
                                                        shell_nodes,
                                                        __CLUSTER_RADIUS));
    pCM->partition(__REGION_COUNT);
    calc_point_weight(shell_nodes, pCM->regions_, w_mat);

    assert(w_mat.size() == __REGION_COUNT);

//    for (size_t i = 0; i < w_mat.size(); ++i) {
//        for (size_t j = 0; j < w_mat[i].size(); ++j) {
//            cout << "(" << w_mat[i][j].first << "," << w_mat[i][j].second << ")";
//        }
//        cout << endl;
//    }
//    return 0;



    std::vector<tri_mesh_ptr> _model;
    std::vector<shared_ptr<deformer>> _solver(__NODES_NUMBER);
    gen_patches(shell_mesh, shell_nodes, __NODES_NUMBER, w_mat, _model);

//    for (size_t i = 0; i < _model.size(); ++i) {
//        cout << "[INFO] the program is writing " << i << " patch\n";
//        stringstream oss;
//        oss << "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result_1/o_patches_" << i << ".vtk";
//        char *path = oss.str().c_str();
//        printf("%s\n", path);
//        write_one_patch(oss.str(), _model, i);
//    }

//    cout << "[INFO] domain decompostion done.\n";
//    return 0;

    for (size_t i = 0; i < __NODES_NUMBER; ++i)
        _solver[i].reset(new deformer(_model[i]->mesh, _model[i]->nodes,
                                      _model[i]->nodes, _model[i]->cons));


    char emd_shell_path[200];
    matrix<size_t> emd_mesh;
    matrix<double> emd_nodes;

    size_t frame = 200;
    size_t curr = 0;

    while ( frame-- ) {
        cout << "this is the " << curr << " frame.\n";
        sprintf(emd_shell_path, __POJ_BASE_PATH "dat/obj/cylinder/embd_mesh_%03d.obj", curr);
        jtf::mesh::load_obj(emd_shell_path, emd_mesh, emd_nodes);
        cout << "[INFO] load embedded mesh succeed.\n";

#pragma omp parallel for
        for (size_t i = 0; i < __NODES_NUMBER; ++i) {
            //load the reference embedded mesh of current patch
            matrix<double> emd_ref_nodes;
            emd_ref_nodes.resize(3, _model[i]->nodes.size(2));
            for (size_t j = 0; j < _model[i]->nodes.size(2); ++j) {

              // assert(_model[i]->s2g.find(j) != nullptr);
                emd_ref_nodes(colon(), j) = emd_nodes(colon(), _model[i]->s2g[j]);
            }
            _solver[i]->deform(_model[i]->nodes, emd_ref_nodes);
        }

#pragma omp parallel for
        for (size_t i = 0; i < __NODES_NUMBER; ++i) {
            for (size_t j = 0; j < _model[i]->interior.size(); ++j) {
                size_t pid = _model[i]->interior[j];
                shell_nodes(colon(), pid) = _model[i]->nodes(colon(), _model[i]->g2s[pid]);
            }
        }

        ostringstream oss;
        oss << __POJ_BASE_PATH << "result/test_node_cpu/cylinder_2threads_" << curr << ".vtk";
        ++curr;
        ofstream os(oss.str());
        tri2vtk(os, &shell_nodes[0], shell_nodes.size(2), &shell_mesh[0], shell_mesh.size(2));
    }

    cout << "[INFO] all done!\n";
    return 0;
}
