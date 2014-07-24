#include <iostream>

#include <jtflib/mesh/io.h>
#include <jtflib/mesh/mesh.h>
#include <sxxlib/tri_sub/tri_subdivision.h>
#include "common/vtk.h"
#include "common/mesh_operation.h"

using namespace std;
using namespace zjucad::matrix;
using jtf::mesh::edge2cell_adjacent;


int main(int argc, char *argv[])
{
    matrix<size_t> tet_mesh, shell;
    matrix<double> tet_nodes, shell_nodes;

    jtf::mesh::tet_mesh_read_from_zjumat("/home/chenjiong/Desktop/cylinder.tet", &tet_nodes, &tet_mesh);

    using jtf::mesh::face2tet_adjacent;
    using jtf::mesh::edge2cell_adjacent;
    boost::shared_ptr<face2tet_adjacent> f2t(face2tet_adjacent::create(tet_mesh));

    jtf::mesh::get_outside_face(*f2t, shell, true);
    shell_nodes = tet_nodes;

    remove_extra_node(shell, shell_nodes);

    matrix<size_t> new_face;
    matrix<double> new_nodes;
    std::vector<size_t> add_node;
    int N = 2;
    while ( N-- ) {
        shared_ptr<edge2cell_adjacent> e2c(edge2cell_adjacent::create(shell, false));
        std::vector<std::pair<size_t, size_t>> edges(e2c->edges_);
        for (auto it = edges.begin(); it != edges.end(); ++it)
            if ( it->first > it->second )
                std::swap(it->first, it->second);
        sxx::sub_edge(shell_nodes, shell, edges, new_nodes, new_face, add_node);
        shell = new_face;
        shell_nodes = new_nodes;
    }


    cout << "face : " << shell.size(2) << endl;
    cout << "nodes : " << shell_nodes.size(2) << endl;

    std::ofstream out("/home/chenjiong/Desktop/shell.vtk");
    tri2vtk(out, &shell_nodes[0], shell_nodes.size(2), &shell[0], shell.size(2));
    cout << "[INFO] done!\n";
	return 0;
}
