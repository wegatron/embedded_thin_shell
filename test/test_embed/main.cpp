#include <iostream>
#include <cstring>
#include <jtflib/mesh/mesh.h>
#include <jtflib/mesh/io.h>
#include <zjucad/matrix/io.h>
#include "interpolator/interpolator.h"
#include "common/vtk.h"

using namespace zjucad::matrix;
using namespace std;


int main(int argc, char *argv[])
{
    matrix<size_t> tet, shell;
    matrix<double> tet_nodes, shell_nodes, shell_normal;

    char model[20], in_addr[100];
    cout << "MODEL : ";
    scanf("%s", model);
    sprintf(in_addr, "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/%s/orig.tet", model);
    jtf::mesh::tet_mesh_read_from_zjumat(in_addr, &tet_nodes, &tet);

    gen_outside_shell(tet, tet_nodes, shell, shell_nodes, shell_normal);

    std::ofstream ios("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result/interior.vtk");
    std::ofstream oos("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result/exterior.vtk");
    tet2vtk(oos, &tet_nodes[0], tet_nodes.size(2), &tet[0], tet.size(2));
    tri2vtk(ios, &shell_nodes[0], shell_nodes.size(2), &shell[0], shell.size(2));

    cout << "[INFO]DONE!\n";
    return 0;
}
