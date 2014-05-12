#include "DataModel.h"
#include <SubspaceSimulator.h>
#include <FullStVKSimulator.h>
#include <MatrixIO.h>
using namespace SIMULATOR;

DataModel::DataModel(pTetMeshEmbeding embeding):_volObj(embeding){

  assert(embeding);
  _simulator = pSimulator(new FullStVKSimulator());
}

pSimulator DataModel::createSimulator(const string filename)const{
  
  string simulator_name = "full_stvk";
  JsonFilePaser jsonf;
  if (jsonf.open(filename)){
	jsonf.read("simulator", simulator_name, string("full_stvk"));
  }

  pSimulator sim;
  if ("subspace" == simulator_name){
	pReducedElasticModel elas_m = pReducedElasticModel(new DirectReductionElasticModel());
	sim = pSimulator(new SubspaceSimulator(elas_m,string("subspace")));
  }else if ("cubature" == simulator_name){
	pReducedElasticModel elas_m = pReducedElasticModel(new CubaturedElasticModel());
	sim = pSimulator(new SubspaceSimulator(elas_m,string("cubature")));
  }else{
	sim = pSimulator(new FullStVKSimulator());
  }

  return sim;
}

bool DataModel::loadSetting(const string filename){

  JsonFilePaser jsonf;
  if (!jsonf.open(filename)){
	ERROR_LOG("failed to open: " << filename);
	return false;
  }

  _simulator = createSimulator(filename);

  bool succ = true;
  string mtlfile;
  if (jsonf.readFilePath("elastic_mtl",mtlfile)){
	if (_volObj && _volObj->getTetMesh()){
	  succ = _volObj->getTetMesh()->loadElasticMtl(mtlfile);
	}
  }

  // string fixed_node_file;
  // if (jsonf.readFilePath("fixed_nodes", fixed_node_file)){
  // 	succ &= loadFixedNodes(fixed_node_file);
  // }
	  
  if (_simulator){
	succ &= _simulator->init(filename);
  }

  print();
  return succ;
}

void DataModel::prepareSimulation(){

  if(_simulator){
	_simulator->setVolMesh(_volObj->getTetMesh());
	const bool succ = _simulator->precompute();
	ERROR_LOG_COND("the precomputation is failed.",succ);
  }
}

void DataModel::setForces(const int nodeId,const double force[3]){

  if(_simulator){
	_simulator->setExtForceOfNode(nodeId,force);
	this->simulate();
  }
}

void DataModel::getSubUc(const vector<set<int> > &groups,const VectorXd &full_u,Matrix<double,3,-1> &sub_u)const{

  int nodes = 0;
  BOOST_FOREACH(const set<int>& s, groups)
	nodes += s.size();

  sub_u.resize(3,nodes);
  int index = 0;
  BOOST_FOREACH(const set<int>& s, groups){
	BOOST_FOREACH(const int i, s){
	  assert_in(i*3,0,full_u.size()-3);
	  sub_u.col(index) = full_u.segment<3>(i*3);
	  index++;
	}
  }
}

void DataModel::updateUc(const Matrix<double,3,-1> &uc,const int group_id){

  _partialCon.updatePc(uc,group_id);
  if(uc.size() > 0){
	const int n = _partialCon.getPc().size();
	_simulator->setUc(Map<VectorXd>(const_cast<double*>(&(_partialCon.getPc()(0,0))),n));
  }
}

void DataModel::resetPartialCon(){

  Matrix<double,3,-1> pc;
  getSubUc(_partialCon.getConNodesSet(),getU(),pc);
  _partialCon.updatePc(pc);
  static vector<int> con_nodes;
  static VectorXd con_uc;
  _partialCon.getPartialCon(con_nodes, con_uc);
  _simulator->setConNodes(con_nodes);
  _simulator->setUc(con_uc);
}

bool DataModel::simulate(){

  bool succ = false;
  if(_simulator){
	succ = _simulator->forward();
	_volObj->interpolate(getU());
  }

  { // recording.
	// static vector<VectorXd> record_u, record_f;
	// static int i = 0;
	// i ++;
	// cout << "recording: "<< i  << endl;
	// if (i < 300){
	//   record_u.push_back(getU());
	//   static VectorXd f;
	//   _simulator->computeElasticForce(getU(), f);
	//   record_f.push_back(f);
	// }else{
	//   EIGEN3EXT::write("training_u.b", record_u);
	//   EIGEN3EXT::write("training_f.b", record_f);
	//   exit(0);
	// }
  }
  
  return succ;
}
