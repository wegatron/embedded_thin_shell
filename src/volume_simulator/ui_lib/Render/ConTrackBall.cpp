#include <WindowsHeaders.h>
#include <GL/glew.h>
#include <math.h>
#include "ConTrackBall.h"
using namespace qglviewer;
using namespace QGLVEXT;

ConTrackBall::ConTrackBall(pQGLViewerExt viewer):viewer(viewer){

  assert (viewer != NULL);

  p_AxisTorus = pAxisTorus(new AxisTorus());
  constraint = new WorldConstraint();
  constrained_axi = 0;
  m_show = false;
  m_hit = false;

  constraint->setRotationConstraintType(AxisPlaneConstraint::FREE);
  viewer->camera()->frame()->setConstraint(constraint);

  connect( viewer, SIGNAL(mousePressSignal(QMouseEvent *)),this, SLOT(press(QMouseEvent *)) );
  connect( viewer, SIGNAL(mouseReleaseSignal(QMouseEvent *)),this, SLOT(release(QMouseEvent *)) );
  connect( viewer, SIGNAL(selectedIds(vector<int>)),this, SLOT(selectAxises(vector<int>)));
}

void ConTrackBall::selectAxises(const vector<int> sel_group_ids){

  if (m_show){
	if (sel_group_ids.size() <= 0 ){
	  m_hit = false;
	  constrained_axi = -1;
	  p_AxisTorus->selectAxis(constrained_axi);
	}else{
	  m_hit = true;
	  constrained_axi = sel_group_ids[0];
	  p_AxisTorus->selectAxis(constrained_axi);
	  viewer->update();
	}
	//if (viewer->selectedName() > 0)
	//{
	//	m_hit = true;
	//	int hit = viewer->selectedName();
	//	constrained_axi = hit %3;
	//	p_AxisTorus->selectAxis(constrained_axi);
	//	viewer->update();
	//}
  }
}

void ConTrackBall::checkIfGrabsMouse(int x,int y,const Camera*const camera){
  if (m_show)
	{
	  viewer->setSelector(p_AxisTorus);
	  QRect select_rect(x,y,5,5);
	  viewer->select (select_rect);
	}
}

void ConTrackBall::press(QMouseEvent* e){

  if (m_show && m_hit){
	constraint->setRotationConstraintType(AxisPlaneConstraint::AXIS);
	Vec dir(0.0,0.0,0.0);
	dir[constrained_axi] = 1.0;
	constraint->setRotationConstraintDirection(dir);
  }
}

void ConTrackBall::release(QMouseEvent* e){
  if (m_show)
	{
	  constraint->setRotationConstraintType(AxisPlaneConstraint::FREE);
	  constrained_axi = -1;
	  p_AxisTorus->selectAxis(constrained_axi);
	}
}

void ConTrackBall::ShowConTrackBall(){

  m_show = !m_show;
  if (m_show){
	viewer->addSelfRenderEle(p_AxisTorus);
  }else{
	viewer->removeSelfRenderEle(p_AxisTorus);
  }
}
