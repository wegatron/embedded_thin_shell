#ifndef _DATAMODEL_H_
#define _DATAMODEL_H_

#include <boost/shared_ptr.hpp>
#include <QObject>
#include <Log.h>
#include <AuxTools.h>
#include <MatrixIO.h>
#include <TetMeshEmbeding.h>
#include <JsonFilePaser.h>
#include <Simulator.h>
#include <PartialConstraints.h>

#include <zjucad/matrix/matrix.h>
#include <jtflib/mesh/util.h>
#include "shell_deformer/deformer.h"

using namespace std;
using namespace UTILITY;
using namespace zjucad::matrix;

namespace SIMULATOR{
  
  /**
   * @class DataModel contains all the data for simulation.
   * 
   */
  class DataModel:public QObject{
	
	Q_OBJECT

  public:
	DataModel(pTetMeshEmbeding embeding);
	const string simulatorName()const{
	  if (_simulator) return _simulator->name();
	  return "no simulator.";
	}
	pSimulator createSimulator(const string filename)const;
	bool loadSetting(const string filename);

	// set fixed nodes
	void addConNodes(const vector<int> &sel_ids){
	  _partialCon.addConNodes(sel_ids);
	  resetPartialCon();
	}
	void removeConNodes(const vector<int> &sel_ids){
	  _partialCon.rmConNodes(sel_ids);
	  resetPartialCon();
	}
	const vector<set<int> > &getConNodes()const{
	  return _partialCon.getConNodesSet();
	}
	void updateUc(const Matrix<double,3,-1> &uc,const int group_id);

	// perturbation
	void setForces(const int nodeId,const double force[3]);

	// get data
	const pObjmesh_const getObjMesh()const{
	  return _volObj->getObjMesh();
	}
	const pTetMesh_const getVolMesh()const{
	  return _volObj->getTetMesh();
	}
	const VectorXd &getU()const{
	  assert(_simulator);
	  return _simulator->getFullDisp();
	}
	const Matrix<double,3,-1> getUc(const int group)const{
	  assert_in(group,0,_partialCon.numGroup()-1);
	  return _partialCon.getPc(group);
	}

	// io
	void print()const{}

  public slots:
	void prepareSimulation();
	bool simulate();
	void reset(){
      if(_simulator && shell_deformer_)  {
          _simulator->reset();
          shell_deformer_->reset(shell_nodes_);
          jtf::mesh::cal_point_normal(shell_mesh_, shell_nodes_, shell_normal_);
      }
	}

  protected:
	void resetPartialCon();
	void getSubUc(const vector<set<int> > &groups,const VectorXd &full_u,Matrix<double,3,-1> &sub_u)const;
	
  private:
	PartialConstraints _partialCon;
	pSimulator _simulator;
	pTetMeshEmbeding _volObj;


//********MY MODIFY***************************************
public :
    matrix<size_t>         tet_mesh_, shell_mesh_;
    matrix<double>         tet_nodes_, shell_nodes_;
    matrix<double>         shell_normal_;
    shared_ptr<deformer>   shell_deformer_;
    matrix<double>         B_;
    map<size_t, vector<pair<size_t, string>>> regions_;
    matrix<double> dx_, q_, xq_;
//********************************************************


  };
  
  typedef boost::shared_ptr<DataModel> pDataModel;
  
}//end of namespace

#endif /*_DATAMODEL_H_*/
