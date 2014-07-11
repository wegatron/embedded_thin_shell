#include <iostream>

#include <jtflib/mesh/io.h>

#include "shell_deformer/deformer.h"
#include "common/vtk.h"
#include "conf_para.h"

using namespace std;
using namespace zjucad::matrix;


int main(int argc, char *argv[])
{
    matrix<size_t> t0, t;
    matrix<double> r0, x;
    jtf::mesh::load_obj(__POJ_BASE_PATH "dat/cloth/ref.obj", t0, r0);
    jtf::mesh::load_obj(__POJ_BASE_PATH "dat/cloth/def.obj", t, x);

    deformer df(t0, r0);
    int cnt = 0;
    while ( true ) {
        if ( ++cnt == 300 )
            break;
        char outfile[200];
        sprintf(outfile, __POJ_BASE_PATH "result/shell_def_%d.vtk", cnt);
        std::ofstream os(outfile);
        tri2vtk(os, &x[0], x.size(2), &t[0], t.size(2));

        df.deform(x);
    }
    cout << "[INFO]DONE!\n";
    return 0;
}
