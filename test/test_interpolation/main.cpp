#include <jtflib/mesh/io.h>
#include <zjucad/matrix/io.h>
#include <hjlib/sparse/operation.h>
#include "common/vtk.h"
#include "interpolator/interpolator.h"

using namespace std;
using namespace zjucad::matrix;


int main(int argc, char *argv[])
{
    matrix<size_t> tet_mesh, shell_mesh;
    matrix<double> tet_nodes, shell_nodes, shell_normal, xq;

    jtf::mesh::tet_mesh_read_from_vtk("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/orig.vtk", &tet_nodes, &tet_mesh);

    gen_outside_shell(tet_mesh, tet_nodes, shell_mesh, shell_nodes, shell_normal, 1, 0.003);

    size_t row = tet_nodes.size(2);
    size_t col = shell_nodes.size(2);

    hj::sparse::spm_csc<double> B(row, col);
    tet_embed(tet_nodes, tet_mesh, shell_nodes, B);

    matrix<double> B_(row, col);
    for (size_t j = 0; j < col; ++j)
        for (size_t i = 0; i < row; ++i)
            B_(i, j) = B(i, j);

    xq = tet_nodes * B_;

    {
        std::ofstream os("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result/orig.vtk");
        tet2vtk(os, &tet_nodes[0], tet_nodes.size(2), &tet_mesh[0], tet_mesh.size(2));
    }
    {
        std::ofstream os("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result/sub.vtk");
        tri2vtk(os, &shell_nodes[0], shell_nodes.size(2), &shell_mesh[0], shell_mesh.size(2));
    }
    {
        std::ofstream os("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result/embed.vtk");
        tri2vtk(os, &xq[0], xq.size(2), &shell_mesh[0], shell_mesh.size(2));
    }

    cout << "[INFO]done!\n";
    return 0;
}
