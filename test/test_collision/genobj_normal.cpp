#include <Objmesh.h>
#include <jtflib/mesh/util.h>

#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
  if (argc != 5) {
    cout << "usage: gen_obj_normal [prefix] [suffix] [count] [bits]" << endl;
    //                             1          2        3       4
  }
  UTILITY::Objmesh obj;
  int count = atoi(argv[3]);
  int bits = atoi(argv[4]);
  char file_name[100];
  char type_str[100];
  for (int i=0; i< count; ++i) {
    sprintf(type_str, "%%s%%0%dd%%s", bits);
    sprintf(file_name, type_str, argv[1], i, argv[2]);
    cout << "gen normal for file:" << file_name << endl;
    obj.load(file_name);
    VectorXd nodes_e = obj.getVerts();
    VectorXi cell_e = obj.getFaces();

    zjucad::matrix::matrix<size_t> cell;
    zjucad::matrix::matrix<double> nodes;
    zjucad::matrix::matrix<double> normal;
    cell.resize(3, obj.getFaces().size()/3);
    nodes.resize(3, obj.getVerts().size()/3);
    normal.resize(3, obj.getVerts().size()/3);
    for (int i=0; i<shell_nodes.size(2); ++i) {
      nodes(0,i) = nodes_e[i*3];
      nodes(1,i) = odes_e[i*3+1];
      nodes(2,i) = nodes_e[i*3+2];

      normal(0,i) = normal_e[i*3];
      normal(1,i) = normal_e[i*3+1];
      normal(2,i) = normal_e[i*3+2];
    }


    jtf::mesh::cal_point_normal(cell, nodes, normal);

    export_obj(filename, cell, nodes, normal);
    //sprintf(filename, "%s
  }
  return 0;
}
