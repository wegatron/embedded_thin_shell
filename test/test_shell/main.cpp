#include <iostream>
#include <jtflib/mesh/io.h>
#include "shell_deformer/deformer.h"
#include "common/vtk.h"


#include <jtflib/mesh/mesh.h>


using namespace std;
using namespace zjucad::matrix;
using namespace jtf::mesh;


int main(int argc, char *argv[])
{
    matrix<size_t> t0;
    matrix<double> r0;
    jtf::mesh::load_obj("/home/chenjiong/usr/WorkSpace/embedded_shell/dat/cloth/cube1.obj", t0, r0);

    deformer df(t0, r0);
    for (int cnt = 0; cnt < 2; ++cnt) {
        char outfile[200];
        sprintf(outfile, "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/result/shell_def_%d.vtk", cnt);
        std::ofstream os(outfile);
        tri2vtk(os, &r0[0], r0.size(2), &t0[0], t0.size(2));

        df.deform(r0);
    }
    cout << "[INFO]DONE!\n";
    return 0;
}



//int main(int argc, char *argv[])
//{
//    matrix<size_t> tris;
//    matrix<double> nods;

//    jtf::mesh::load_obj("/home/chenjiong/usr/WorkSpace/embedded_shell/dat/cloth/cube.obj", tris, nods);

//    shared_ptr<one_ring_point_at_point> p2p(one_ring_point_at_point::create(tris));
//    shared_ptr<edge2cell_adjacent> e2c(edge2cell_adjacent::create(tris, false));
//    p2p->sort_into_loop(tris, *e2c);

////    for (auto it = p2p->p2p_.begin(); it != p2p->p2p_.end(); ++it) {
////        cout << it->first << " : " << endl;
////        for (size_t n = 0; n < it->second.size(); ++n)
////            cout << it->second[n] << " ";
////        cout << endl;
////    }


//    cout << "HERE\n";
//    shared_ptr<one_ring_face_at_point> f2p(new one_ring_face_at_point);
//    f2p->add_all_faces(tris, *e2c);
//    f2p->sort_int_loop(tris, nods);
//    for (auto it = f2p->p2f_.begin(); it != f2p->p2f_.end(); ++it) {
//        cout << it->first << " : " << endl;
//        for (size_t n = 0; n < it->second.size(); ++n)
//            cout << it->second[n] << " ";
//        cout << endl;
//    }

//    cout << "DONE\n";

//    return 0;
//}
