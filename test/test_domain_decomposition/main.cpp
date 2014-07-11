#include <iostream>
#include <sstream>
#include <jtflib/mesh/io.h>
#include <jtflib/mesh/util.h>

#include "src/parallize/domain_decomposition.h"
#include "src/shell_deformer/cluster.h"
#include "src/conf_para.h"
#include "common/mesh_operation.h"
#include "common/vtk.h"

using namespace std;
using namespace zjucad::matrix;
using jtf::mesh::face2tet_adjacent;

int main(int argc, char *argv[])
{
    matrix<size_t> tet_mesh, shell_mesh;
    matrix<double> tet_nodes, shell_nodes;

    jtf::mesh::tet_mesh_read_from_zjumat(__POJ_BASE_PATH "dat/sphere/orig.tet",
                                         &tet_nodes, &tet_mesh);

    shared_ptr<face2tet_adjacent> f2t(face2tet_adjacent::create(tet_mesh));
    jtf::mesh::get_outside_face(*f2t, shell_mesh);
    shell_nodes = tet_nodes;
    remove_extra_node(shell_mesh, shell_nodes);

    {
        char *out = string(__POJ_BASE_PATH "result/test_domain_decomposition/tri_surface.vtk").c_str();
        std::ofstream os(out);
        tri2vtk(os, &shell_nodes[0], shell_nodes.size(2), &shell_mesh[0], shell_mesh.size(2));
    }

    cluster_machine handle(shell_mesh, shell_nodes, __CLUSTER_RADIUS);
    handle.partition(__REGION_COUNT);
    vector<vector<pair<size_t, double>>> region_info;
    calc_point_weight(shell_nodes, handle.regions_, region_info);

    vector<tri_mesh_ptr> patches;
    gen_patches(shell_mesh, shell_nodes, __NODES_NUMBER,
                region_info, patches);

    for (size_t i = 0; i < patches.size(); ++i) {
         ostringstream oss;
         oss << __POJ_BASE_PATH << "result/test_domain_decomposition/patches_" << i << ".vtk";

         char *path = oss.str().c_str();
         printf("%s\n", path);


         write_one_patch(oss.str(), patches, i);
    }



    cout << "[INFO] done!\n";
	return 0;
}
