#include <Timer.h>
#include <FullStVKSimulator.h>

using namespace UTILITY;
using namespace SIMULATOR;


void collision_plane_x(const VVec3d &nodes, VectorXd &v, VectorXd &u, double kd, double plane_x,
                    bool up)
{
  for (int i=0; i<nodes.size(); ++i)
    {
      if (up)
        {
          if (nodes[i][0] + u[i*3] > plane_x)
            {
              u[i*3] = plane_x - nodes[i][0];
              v[i*3] = -kd*v[i*3];
            }
        }
      else
        {
            if (nodes[i][0] + u[i*3] < plane_x)
            {
              u[i*3] = plane_x - nodes[i][0];
              v[i*3] = -kd*v[i*3];
            }
        }
    }
}

void case_beam()
{
  // open init json file
  const string ini_file = "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/beam/simu_full.ini";
  JsonFilePaser jsonf;
  bool succ = jsonf.open(ini_file);
  assert(succ);

  pTetMesh tet_mesh = pTetMesh(new TetMesh());
  { // init tetrahedron mesh.
    string vol_file;
    succ = jsonf.readFilePath("vol_file",vol_file);
    assert(succ);
    succ = tet_mesh->load(vol_file);
    assert(succ);

    string mtl_file;
    jsonf.readFilePath("elastic_mtl",mtl_file);
    succ = tet_mesh->loadElasticMtl(mtl_file);
    assert(succ);
  }

  pSimulator simulator = pSimulator(new FullStVKSimulator());
  { // init simulator
    succ = simulator->init(ini_file);
    assert(succ);
    simulator->setVolMesh(tet_mesh);
    simulator->precompute();
  }

  { // load and set fixed nodes
    vector<int> nodes;
    succ = UTILITY::loadVec("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/beam/model/con_nodes2.bou", nodes,UTILITY::TEXT);
    assert(succ);
    cout << "num of fixed nodes: " << nodes.size() << endl;
    simulator->setConNodes(nodes);
    VectorXd uc(nodes.size()*3);
    uc.setZero();
    simulator->setUc(uc);
  }
  
  // simulation for 200 steps, record displacements
  vector<VectorXd> record_u;
  { // set external forces, then simulate 50 steps
    const VVec3d &nodes = tet_mesh->nodes();
    VectorXd fext(nodes.size()*3);
    fext.setZero();
    for (int i=0; i<nodes.size(); ++i)
      {
        fext[i*3] = 8;
      }
    simulator->setExtForce(fext);
    for (int i = 0; i < 5000; ++i){
      cout << "step " << i << endl;
      simulator->forward();
      VectorXd &v = simulator->getV();
      VectorXd &u = simulator->getModifyFullDisp();
      collision_plane_x(tet_mesh->nodes(), v, u, 0, 0.6, true);
      if(i%10 == 0)
        record_u.push_back(simulator->getFullDisp());
    }
  }

  { // save as vtk files.
    succ = tet_mesh->writeVTK("/home/wegatron/tempt/test_beam", record_u);
    if (!succ)
      {
        cout << "error output" << endl;
      }
    assert(succ);
  }

  cout << "[INFO]DONE!\n";

}


// a simple example demonstrate the usage of the solid simulator.
int main(int argc, char *argv[]){
  case_beam();
  return 0;
}
