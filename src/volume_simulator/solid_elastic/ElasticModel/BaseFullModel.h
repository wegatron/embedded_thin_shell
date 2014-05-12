#ifndef _BASEFULLMODEL_H_
#define _BASEFULLMODEL_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
using namespace Eigen;
using namespace std;

namespace SIMULATOR{

  typedef vector<Eigen::Triplet<double> > VecT;
  
  /**
   * @class BaseFullModel interface for computing internal forces, stiffness
   * matrix, mass matrix in full space, which are used in full simulator.
   */
  class BaseFullModel{
	
  public:
	// initialize from file.
	virtual bool init(const std::string init_filename) = 0;
	virtual bool prepare() = 0;

	// compute the internal forces.
	virtual bool evaluateF(const Eigen::VectorXd &u, Eigen::VectorXd &f) = 0;

	// compute the stiffness matrix or its triplet. both in full format.
	virtual bool evaluateK(const Eigen::VectorXd &u, SparseMatrix<double> &K_full)=0;
	virtual bool evaluateK_triplet(const Eigen::VectorXd &u, VecT &K_full_t)=0;

	// compute the mass matrix or its triplet. both in full format.
	virtual bool evaluateM(SparseMatrix<double> &M_full)=0;
	virtual bool evaluateM_triplet(VecT &M_full_t)=0;

	// dimension of the full space.
	virtual int dimension()const = 0;
  };
  
  typedef boost::shared_ptr<BaseFullModel> pBaseFullModel;
  
}//end of namespace

#endif /*_BASEFULLMODEL_H_*/
