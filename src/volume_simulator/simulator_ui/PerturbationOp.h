#ifndef _PERTURBATIONOP_H_
#define _PERTURBATIONOP_H_

#include <boost/shared_ptr.hpp>
#include <QObject>
#include <Perturbator.h>
#include <SelectCtrl.h>
#include <Selectable.h>
#include <DragCtrl.h>
#include "DataModel.h"
using namespace UTILITY;
using namespace QGLVEXT;

namespace SIMULATOR{

  /**
   * @class PerturbationOp control the perturbate operation.
   * @see Utility/Perturbator.h
   */
  class Perturbation:public Selectable, public DragObserver,
					 public SelfRenderEle, public DragHook{
	
  public:
	Perturbation(pQGLViewerExt viewer,pDataModel dm):_viewer(viewer),_dataModel(dm){
      	  _perturbator.setPerturCompilance(1.0f);
	}
	void setPerturCompilance(const double p){
	  assert_gt(p,0.0);
	  _perturbator.setPerturCompilance(p);
	}
	int totalEleNum ()const{
	  const pTetMesh_const tetmesh = _dataModel->getVolMesh();
	  assert(tetmesh);
	  return tetmesh->nodes().size();
	}
	void drawWithNames ()const{
	  pTetMesh_const tetmesh = _dataModel->getVolMesh();

	  assert(tetmesh);
	  glFlush();
	  const VectorXd &u = _dataModel->getU();		
	  assert_eq(u.size(),tetmesh->nodes().size()*3);
	  for (int i=0; i<tetmesh->nodes().size(); i++){

		const Vector3d &v = tetmesh->nodes()[i];
		glPushName(i);
		glBegin(GL_POINTS);
		glVertex3d(v[0]+u[i*3+0], v[1]+u[i*3+1], v[2]+u[i*3+2]);
		glEnd();
		glPopName();
	  }
	}
	
	// observer
	void selectDragEle(int nodeId){
	  _perturbator.setPerturbNodeId(nodeId);
	}
	void startDrag (double x,double y,double z){
	  _perturbator.setStartPosition(x,y,z);
	}
	void dragTo (double x,double y,double z){

	  _perturbator.dragTo(x,y,z);
	  double force[3];
	  _perturbator.getPerturbForce(force);
	  _dataModel->setForces(_perturbator.getPerturbNodeId(),force);
	  if(_viewer) _viewer->update();
	}
	void stopDrag (double x,double y,double z){
	  double force[3] = {0,0,0};
	  _dataModel->setForces(_perturbator.getPerturbNodeId(),force);
	  _perturbator.clear();
	}
	
	// render
	void draw()const{

	  if(_dataModel){

		const pTetMesh_const tetmesh = _dataModel->getVolMesh();
		const int i = _perturbator.getPerturbNodeId();
		if(tetmesh && i >= 0){

		  assert_in(i,0,tetmesh->nodes().size()-1);
		  Vector3d n = tetmesh->nodes()[i];
		  const VectorXd &u = _dataModel->getU();
		  if(u.size()>0){
			n[0] += u[i*3+0];
			n[1] += u[i*3+1];
			n[2] += u[i*3+2];
		  }

		  glDisable(GL_LIGHTING);
		  glColor3d(1.0,0.0f,0.0f);
		  glPointSize(8.0f);
		  glBegin(GL_POINTS);
		  glVertex3d(n[0],n[1],n[2]);
		  glEnd();
		}
	  }
	}

	// hook
	void getDragedPoint(double point[3])const{

	  assert(_dataModel);
	  assert(_dataModel->getVolMesh());
	  const VVec3d &nodes = _dataModel->getVolMesh()->nodes();
	  const int n = _perturbator.getPerturbNodeId();
	  const VectorXd &u = _dataModel->getU();
	  assert_eq(u.size(),nodes.size()*3);
	  assert_in(n,0,nodes.size()-1);

	  point[0] = nodes[n][0]+u[n*3+0];
	  point[1] = nodes[n][1]+u[n*3+1];
	  point[2] = nodes[n][2]+u[n*3+2];
	}
	
  private:
	pQGLViewerExt _viewer;
	pDataModel _dataModel;
	Perturbator _perturbator;
  };
  typedef boost::shared_ptr<Perturbation> pPerturbation;

  class PerturbationCtrl:public QObject{

	Q_OBJECT
	
  public: 
	PerturbationCtrl(pQGLViewerExt view,pDataModel dm){

	  _perturb = pPerturbation(new Perturbation(view,dm));
	  _dragCtrl = pDragCtrl(new DragCtrl(view,_perturb));
	  _dragCtrl->setObserver(_perturb);
	  _dragCtrl->setDragHook(_perturb);
	  _dragCtrl->setKeyMouse(Qt::ShiftModifier,Qt::LeftButton);
	  if (view != NULL){
		_perturb->setRenderPriority(FIRST_RENDER);
		view->addSelfRenderEle(_perturb);
	  }
	}
	void setPerturCompilance(const double p){
	  assert_gt(p,0.0);
	  if (_perturb)	_perturb->setPerturCompilance(p);
	}

  private:
	pPerturbation _perturb;
	pDragCtrl _dragCtrl;
  };
  typedef boost::shared_ptr<PerturbationCtrl> pPerturbationCtrl;

}//end of namespace

#endif /*_PERTURBATIONOP_H_*/
