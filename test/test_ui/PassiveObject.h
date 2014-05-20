#ifndef _PASSIVEOBJECT_H_
#define _PASSIVEOBJECT_H_

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <SelfRenderEle.h>
#include <MeshRender.h>
#include <Manipulatoion.h>
#include <TetMesh.h>
using namespace Eigen;
using namespace UTILITY;
using namespace QGLVEXT;

namespace SIMULATOR{
  
  /**
   * @class PassiveObject abstract class for passive objects.
   * 
   */
  class PassiveObject:public LocalframeManipulatoionExt{
	
  public:
        PassiveObject(pQGLViewerExt view);

	bool load(const string obj_file){
	  bool succ = obj->load(obj_file);
	  initial_center = obj->getCenter();
	  return succ;
	}
	void setCollisionPenalty(double p){
	  assert_gt(p,0.0f);
	  collision_force_penalty = p;
	}
	virtual void draw()const;

	int totalEleNum()const{
	  return 1;
	}
	void drawWithNames()const;
	void select(const vector<int> &sel_ids);
	Matrix<double,3,-1> &getCurrentPositions(){
	  return current_center;
	}
	const Matrix<double,3,-1> &getCurrentPositions()const{
	  return current_center;
	}

	virtual void setScale(const double scale){ 
	  assert_gt(scale,0.0f);
	  this->scale = scale;
	}
	virtual void move(const double x,const double y,const double z){
	  current_center(0,0) =initial_center[0] + x;
	  current_center(1,0)=initial_center[1] + y;
	  current_center(2,0) =initial_center[2] + z;
	}
	virtual void collision(pTetMesh_const tet_mesh, const VectorXd &u, VectorXd &coll_forces){
	  if (tet_mesh){
		coll_forces.resize(tet_mesh->nodes().size()*3);
		coll_forces.setZero();
	  }else{
		coll_forces.resize(0);
	  }
	}

  protected:
	virtual void drawMesh()const{
	  UTILITY::draw(obj,mtl);
	}

  protected:
	Vector3d initial_center;
	Matrix<double,3,-1> current_center;
	double scale;
	pObjmesh obj;
	bool dragged;
	double collision_force_penalty;
        ObjMtl mtl;
  };
  typedef boost::shared_ptr<PassiveObject> pPassiveObject;

  class PassiveBall:public PassiveObject{
	
  public:
	PassiveBall(pQGLViewerExt view, const int slice = 100):
	  PassiveObject(view),slice_num(slice){}
	void collision(pTetMesh_const tet_mesh, const VectorXd &u, VectorXd &coll_forces);

  protected:
	void drawMesh()const{
	  PassiveObject::drawMesh();
	  // gluSphere(gluNewQuadric(), 1.0, slice_num, slice_num);///@bug failed to draw.
	}
	double radius()const{
	  return 0.3;
	}

  private:
	int slice_num;
  };
  
}//end of namespace

#endif /*_PASSIVEOBJECT_H_*/
