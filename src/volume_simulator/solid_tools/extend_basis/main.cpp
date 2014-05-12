#include <SparseMatrixTools.h>
#include <ElasticForceTetFullStVK.h>
#include <SparseGenEigenSolver.h>
#include <MassMatrix.h>
#include <SparseMatrixTools.h>
#include <ExtendModalBasis.h>
#include <JsonFilePaser.h>
#include <MatrixIO.h>

using namespace UTILITY;
using namespace SIMULATOR;
using namespace EIGEN3EXT;

int main(int argc, char *argv[]){

  // check
  if (argc < 2){
	ERROR_LOG("ussage: extend_basis ini_json_file_name");
	return -1;
  }
  JsonFilePaser jsonf;
  if(!jsonf.open(argv[1])){
	ERROR_LOG("failed to open json file: " << argv[1]);
	return -1;
  }

  // load data
  INFO_LOG("loading data....");
  pTetMesh tet_mesh = pTetMesh(new TetMesh());
  string tet_file, elastic_mtl;
  if(jsonf.readFilePath("vol_file", tet_file)){
	if(!tet_mesh->load(tet_file)){
	  ERROR_LOG("failed to load tet mesh file: " << tet_file);
	  return -1;
	}else if(jsonf.readFilePath("elastic_mtl", elastic_mtl)){
	  const bool s = tet_mesh->loadElasticMtl(elastic_mtl);
	  ERROR_LOG_COND("failed to load the elastic material from: "<<elastic_mtl,s);
	}
  }

  set<int> fixed_nodes;
  jsonf.read("fixed_nodes", fixed_nodes);
  int num_linear_modes = fixed_nodes.size()>0 ? 15:20;
  jsonf.read("num_linear_modes",num_linear_modes,num_linear_modes);

  /// compute full K, M
  INFO_LOG("computing full K, M....");
  ElasticForceTetFullStVK ela(tet_mesh);
  ela.prepare();
  VectorXd x0;
  tet_mesh->nodes(x0);
  SparseMatrix<double> K = ela.K(x0)*(-1.0f);

  MassMatrix mass;
  DiagonalMatrix<double,-1> diagM;
  mass.compute(diagM,*tet_mesh);

  /// remove fixed nodes
  INFO_LOG("remove fixed dofs in K, M....");
  SparseMatrix<double> P;
  EIGEN3EXT::genReshapeMatrix(K.rows(),3,fixed_nodes,P);
  K = P*(K*P.transpose());
  const SparseMatrix<double> M = P*(diagM*P.transpose());

  /// solve general eigen value problem for W, lambda
  INFO_LOG("solving general eigen value problem K*x = la*M*x....");
  MatrixXd W;
  VectorXd lambda;
  const SparseMatrix<double> Klower = getLower(K);
  if(!EigenSparseGenEigenSolver::solve(Klower,M,W,lambda,num_linear_modes)){
  	ERROR_LOG("failed to solve the general eigen value problem.");
  	return -1;
  }
  if (fixed_nodes.size() > 0){
	W = P.transpose()*W;
  }

  // extend basis W to obtain B
  INFO_LOG("extending basis....");
  MatrixXd B;
  if (fixed_nodes.size() > 0){
	ExtendModalBasis::construct(W, B);
  }else{
	ExtendModalBasis::construct(W, x0, B);
  }
  
  // save
  string save_b;
  jsonf.readFilePath("subspace_basis",save_b,string("B.b"),false);
  INFO_LOG("save extended basis to: "<< save_b);
  if(!write(save_b, B)){
	ERROR_LOG("failed to save the extended basis to: " << save_b);
  }

  return 0;
}
