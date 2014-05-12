#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <boost/shared_ptr.hpp>
#include <set>
#include <eigen3/Eigen/Dense>
#include <TetMesh.h>

using namespace std;
using namespace Eigen;
using namespace UTILITY;

namespace SIMULATOR{

  /**
   * @class Simulator interface for the solid simulation.
   * 
   */
  class Simulator{
	
  public:
	virtual string name()const{
	  return "un-known-simulator";
	}
	virtual bool init(const string filename) = 0;
	virtual void setVolMesh(pTetMesh_const tetMesh) = 0;
	virtual bool precompute(){}
	virtual void reset() = 0;

	virtual void setConNodes(const vector<int> &con_nodes) = 0;
	virtual void setUc(const VectorXd &uc) = 0;
	virtual void removeAllConNodes() = 0;

	virtual void setExtForceOfNode(const int nodeId,const double f[3]) = 0;
	virtual void setExtForce(const VectorXd &f_ext) = 0;
	virtual void clearExtForce() = 0;

	virtual bool forward() = 0;

	virtual const VectorXd &getFullDisp()const = 0;
	virtual bool computeElasticForce(const VectorXd &u,VectorXd &f)const{
	  return false;
	}

  };
  typedef boost::shared_ptr<Simulator> pSimulator;
  
}//end of namespace

#endif /*_SIMULATOR_H_*/
