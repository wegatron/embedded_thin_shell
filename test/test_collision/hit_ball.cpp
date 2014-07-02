#include <sstream>
#include <Timer.h>
#include <FullStVKSimulator.h>
#include <zjucad/matrix/matrix.h>
#include <common/vtk.h>
#include <jtflib/mesh/util.h>
#include <Objmesh.h>
#include "interpolator/interpolator.h"
#include "shell_deformer/cluster.h"
#include "shell_deformer/deformer.h"
#include "conf_para.h"
#include "objio.h"
#include "move_ball.h"

using namespace UTILITY;
using namespace SIMULATOR;

void cacu_gravity(const pTetMesh tet_mesh, VectorXd& fext, int axis_index, bool isnegtive=true)
{
  const VVec4i& tets = tet_mesh->tets();
  std::vector<double> rho = tet_mesh->material()._rho;
  int tet_size = tets.size();
  assert(tet_size == rho.size());
  if (tet_size != rho.size()) { cerr << "error" << endl; exit(0); }
  for (int i=0; i<tet_size; ++i) {
    double tmp_g = 25.5*rho[i]*tet_mesh->volume(i);
    if (isnegtive) {
      fext[tets[i][0]*3+axis_index] -= tmp_g;
      fext[tets[i][1]*3+axis_index] -= tmp_g;
      fext[tets[i][2]*3+axis_index] -= tmp_g;
      fext[tets[i][3]*3+axis_index] -= tmp_g;
    } else {
      fext[tets[i][0]*3+axis_index] += tmp_g;
      fext[tets[i][1]*3+axis_index] += tmp_g;
      fext[tets[i][2]*3+axis_index] += tmp_g;
      fext[tets[i][3]*3+axis_index] += tmp_g;
    }
  }
}

void collision_ball(const VVec3d &nodes,  VectorXd &u, VectorXd &extforce, const MovingBall &ball, double k)
{
  extforce.resize(u.size());
  extforce.setZero();
  double r = ball.getR();
  for (int i=0; i<nodes.size(); ++i) {
    Vector3d normal = nodes[i] + u.segment<3>(i*3) - ball.getCenter();
    double diff = r - normal.norm();
    if (diff > 0) { // collision
        extforce.segment<3>(i*3) = k * diff * 1.0/normal.norm() * normal;
    }
  }
}
/**
 * front true is collision in the front face of that axis
 */
void collision_plane(const VVec3d &nodes, VectorXd &v, VectorXd &u, double kd, int plane_index, double plane_height, bool front)
{
  if (front) {
    for (int i=0; i<nodes.size(); ++i)
      {
        if (nodes[i][plane_index] + u[i*3+plane_index] < plane_height)
          {
            u[i*3+plane_index] = plane_height - nodes[i][plane_index];
            v[i*3+plane_index] = -kd*v[i*3+plane_index];
            // v[i*3+plane_index] = 0;
          }
      }
  }
  else {
    for (int i=0; i<nodes.size(); ++i) {
      if (nodes[i][plane_index] + u[i*3+plane_index] > plane_height)
        {
          u[i*3+plane_index] = plane_height - nodes[i][plane_index];
          v[i*3+plane_index] = -kd*v[i*3+plane_index];
          // v[i*3+plane_index] = 0;
        }
    }
  }
}

void case_common(const char *ini_file)
{
  // open init json file
  JsonFilePaser jsonf;
  int steps = 1;
  int output_steps = 1;
  int plane_index = 2;
  bool front_collision = true;
  double plane_height = 0;
  double kd = 0;
  string output_file;

  bool succ = jsonf.open(ini_file);
  assert(succ);

  {// output setting
    succ &= jsonf.read("steps", steps);
    succ &= jsonf.read("output_steps", output_steps);
    succ &= jsonf.read("output_file", output_file);
  }
  assert(succ);

  {// colision setting
    succ &= jsonf.read("plane_index", plane_index);
    succ &= jsonf.read("plane_height", plane_height);
    succ &= jsonf.read("front_collision", front_collision);
    succ &= jsonf.read("kd", kd);
  }
  assert(succ);

  pTetMesh tet_mesh = pTetMesh(new TetMesh());
  { // init tetrahedron mesh.
    string vol_file;
    succ = jsonf.readFilePath("vol_file",vol_file);
    assert(succ);
    succ = tet_mesh->load(vol_file);
    cout << "tet size:" << tet_mesh->tets().size() << endl;
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

  string con_file;
  if( jsonf.readFilePath("con_nodes", con_file) )
    { // load and set fixed nodes
      vector<int> nodes;
      succ = UTILITY::loadVec(con_file.c_str(), nodes,UTILITY::TEXT);
      assert(succ);
      cout << "num of fixed nodes: " << nodes.size() << endl;
      simulator->setConNodes(nodes);
      VectorXd uc(nodes.size()*3);
      uc.setZero();
      simulator->setUc(uc);
    }

  // simulation for 200 steps, record displacements
  vector<VectorXd> record_u;
  cout << "nodes size:" << tet_mesh->nodes().size() << endl;
  const VVec3d &nodes = tet_mesh->nodes();
  const VVec4i &tets = tet_mesh->tets();
  zjucad::matrix::matrix<size_t> tet_cell(4, tets.size());
  zjucad::matrix::matrix<double> tet_nodes(3, nodes.size());
  for (int i=0; i<tets.size(); ++i)  {
    tet_cell(0,i) = tets[i][0];
    tet_cell(1,i) = tets[i][1];
    tet_cell(2,i) = tets[i][2];
    tet_cell(3,i) = tets[i][3];
  }
  for (int i=0; i<nodes.size(); ++i) {
    tet_nodes(0,i) = nodes[i][0];
    tet_nodes(1,i) = nodes[i][1];
    tet_nodes(2,i) = nodes[i][2];
  }

  // zjucad::matrix::matrix<size_t> shell_cell;
  // zjucad::matrix::matrix<double> shell_nodes;
  // zjucad::matrix::matrix<double> shell_normal;
  // gen_outside_shell(tet_cell, tet_nodes, shell_cell, shell_nodes, shell_normal, __SUBDIVISION_TIME, __EMBED_DEPTH);

  // size_t row = tet_nodes.size(2);
  // size_t col = shell_nodes.size(2);
  // hj::sparse::spm_csc<double> B(row, col);
  // tet_embed(tet_nodes, tet_cell, shell_nodes, B);
  // matrix<double> B_(row, col);
  // for (size_t j = 0; j < col; ++j)
  //   for (size_t i = 0; i < row; ++i)
  //     B_(i, j) = B(i, j);

  // cluster_machine handle(shell_cell, shell_nodes, __CLUSTER_RADIUS);
  // handle.partition(__REGION_COUNT);

  // deformer shell_deformer(shell_cell, shell_nodes, shell_nodes, handle.regions_);

  // matrix<double> dx(tet_nodes.size(1), tet_nodes.size(2)),
  //   q(tet_nodes.size(1), tet_nodes.size(2)),
  //   xq(shell_nodes.size(1), shell_nodes.size(2));

  Timer timer;
  { // set external forces, then simulate 50 steps
    const VVec3d &nodes = tet_mesh->nodes();
    VectorXd gravity(nodes.size()*3);
    gravity.setZero();
    cacu_gravity(tet_mesh, gravity, plane_index, front_collision);
    simulator->setExtForce(gravity);
    record_u.push_back(simulator->getFullDisp());
    UTILITY::Objmesh obj;
    obj.load("/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/for_test_drop/hit_ball/model/ball.obj");
    string out_file = "/home/wegatron/workspace/embedded_thin_shell/branches/chenjiong/dat/for_test_drop/hit_ball/result/ball";
    Vector3d tmp_move;
    tmp_move << 0,0, -0.6;
    move_obj(obj,tmp_move);
    MovingBall ball;
    ball.setBounceV(2.0);
    ball.setDeltaV(0.01);
    ball.setDecDiff(0.38);
    ball.setBounceDiff(0.6);
    ball.setAxisIndex(2);
    ball.setV(-2.0);
    ball.setCenter(-0.548927, -0.583168, 0.4089);
    ball.setR(0.2);
    for (int i = 0; i < steps; ++i){
      simulator->forward();
      VectorXd &v = simulator->getV();
      VectorXd &u = simulator->getModifyFullDisp();
      ball.stepForward(simulator->getTimestep());
      //void collision_plane(const VVec3d &nodes, VectorXd &v, VectorXd &u, double kd, int plane_index, double plane_height, bool front)
      collision_plane(tet_mesh->nodes(), v, u, kd, plane_index, plane_height, front_collision);
      VectorXd extforce;
      collision_ball(tet_mesh->nodes(), u, extforce, ball, 10000);
      simulator->setExtForce(extforce+gravity);
      // collision_plane(tet_mesh->nodes(), v, u, kd,
      if(i%output_steps == 0) {
        cout << "step:" << i << endl;
        VectorXd disp = simulator->getFullDisp();
        // std::copy(disp.data(), disp.data() + disp.size(), dx.begin());
        // q = tet_nodes + dx;
        // xq = q * B_;
        // shell_deformer.deform(shell_nodes, xq);
        // jtf::mesh::cal_point_normal(shell_cell, shell_nodes, shell_normal);
        // std::stringstream ss;
        // ss << output_file << i/output_steps << ".obj";
        // cout << "export obj" << endl;
        // export_obj(ss.str(),shell_cell, shell_nodes, shell_normal);
        record_u.push_back(disp);
      }
    }
  }

  timer.stop("total simulation time is (seconds):");
  { // save as vtk files.
    succ = tet_mesh->writeVTK(output_file.c_str(), record_u);
    assert(succ);
  }
  cout << "[INFO]DONE!\n";
}

// a simple example demonstrate the usage of the solid simulator.
int main(int argc, char *argv[]){
  if (argc != 2) {
    cout << "usage: test_drop [ini_file]" << endl;
  } else {
    case_common(argv[1]);
  }
  return 0;
}
