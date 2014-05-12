#include <iostream>
#include <cstring>
#include <jtflib/mesh/mesh.h>
#include <jtflib/mesh/io.h>
#include <zjucad/matrix/io.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "shell_deformer/cluster.h"
#include "common/vtk.h"
#include "src/conf_para.h"
#include "src/interpolator/interpolator.h"

using namespace std;
using namespace zjucad::matrix;
using jtf::mesh::face2tet_adjacent;

int main(int argc, char *arg[])
{
    matrix<size_t> tet_mesh, tri_mesh;
    matrix<double> tet_nodes, tri_nodes;
    matrix<double> shell_normal;

    char model[20], in_addr[100], out_addr[100];
    cout << "model : ";
    scanf("%s", model);
    sprintf(in_addr, "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/%s/init.tet", model);

#ifndef VTK_FORMAT
    if ( jtf::mesh::tet_mesh_read_from_zjumat(in_addr, &tet_nodes, &tet_mesh) )
        return __LINE__;
#else
    if ( jtf::mesh::tet_mesh_read_from_vtk(in_addr, &tet_nodes, &tet_mesh) )
        return __LINE__;
#endif

    gen_outside_shell(tet_mesh, tet_nodes, tri_mesh, tri_nodes, shell_normal, __SUBDIVISION_TIME, __EMBED_DEPTH);
//    boost::shared_ptr<face2tet_adjacent> f2c(face2tet_adjacent::create(tet_mesh));
//    jtf::mesh::get_outside_face(*f2c, tri_mesh);
//    tri_nodes = tet_nodes;

    cluster_machine handle(tri_mesh, tri_nodes, 10);
    tri_mesh = handle.mesh_;
    tri_nodes = handle.nodes_;

    handle.partition(20);

    std::map<size_t, std::vector<std::pair<size_t, double>>> region_dis;
    calc_dist_to_center(tri_nodes, handle.regions_, region_dis);

    int cnt = 0;
    for (auto it = handle.regions_.begin(); it != handle.regions_.end(); ++it) {
        cout << "REGIOIN " << cnt << endl;
        matrix<size_t> sub(it->second.size());
        for (size_t i = 0; i < it->second.size(); ++i) {
            sub[i] = it->second[i].first;
            cout << it->second[i].first << " : ";
            cout << it->second[i].second << "    ";
            cout << region_dis[it->first][i].first << " ";
            cout << region_dis[it->first][i].second << " ";
            cout << handle.prime_[sub[i]].dis << endl;
        }
        sprintf(out_addr, "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result/sub_%d.vtk", cnt++);
        std::ofstream os(out_addr);
        point2vtk(os, &tri_nodes[0], tri_nodes.size(2), &sub[0], sub.size());
    }
    std::ofstream os("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result/triangle_mesh.vtk");
    tri2vtk(os, &tri_nodes[0], tri_nodes.size(2), &tri_mesh[0], tri_mesh.size(2));

    cout << "[INFO]DONE!" << endl;
    return 0;
}





//int main()
//{
//    string temp("123.23123.");
//    std::vector<string> str;
//    boost::algorithm::split(str, temp, boost::is_any_of("."), boost::token_compress_on);

//    cout << str.size();
////    for (size_t i = 0; i < str.size(); ++i)
////        cout << str[i] << " ";

//    return 0;
//}

