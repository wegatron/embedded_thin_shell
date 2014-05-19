#include <WindowsHeaders.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <DragGroupSelDraw.h>
#include "DragNodeOp.h"
using namespace SIMULATOR;
using namespace UTILITY;

DragNodeOp::DragNodeOp(pQGLViewerExt viewer, pDataModel dm):
  viewer(viewer),data_model(dm){

  draggedGroupId = -1;
  initial_dragged_point.setZero();
  initial_displacement.setZero();
  const double dragged_color[4] = {0.0f, 0.0f, 0.6f, 1.0f};
  con_node_render.setColor(dragged_color);
}

void DragNodeOp::drawWithNames ()const{

  pTetMesh_const vol_mesh = data_model->getVolMesh();
  if( vol_mesh != NULL ){
	
  	const vector<set<int> > con_groups = data_model->getConNodes();
  	const VectorXd &vol_u = data_model->getU();
	DragGroupSelDraw::drawAllGroupsWithPoints(vol_mesh,con_groups,&vol_u[0]);
  }
}

void DragNodeOp::draw()const{
  
  if (data_model!=NULL&&hasDraggedGroup()){
  	pTetMesh_const vol_mesh = data_model->getVolMesh();
	const set<int> dragged_nodes = data_model->getConNodes()[draggedGroupId];
  	const VectorXd &vol_u = data_model->getU();
  	if (vol_u.size() >= 3)
  	  con_node_render.draw(vol_mesh, dragged_nodes, &vol_u[0], DRAW_POINT);
  }
}

void DragNodeOp::getDragedPoint(double point[3])const{

  point[0] = initial_dragged_point.col(0)[0];
  point[1] = initial_dragged_point.col(0)[1];
  point[2] = initial_dragged_point.col(0)[2];
}

void DragNodeOp::selectDragEle(int sel_group_id){

  if(data_model){
	assert_in(sel_group_id,0,data_model->getConNodes().size()-1);
	draggedGroupId = sel_group_id;
	pTetMesh_const restVolMesh = data_model->getVolMesh();
	initial_displacement = data_model->getUc(sel_group_id);
  }
}

void DragNodeOp::startDrag (double x,double y,double z){

  initial_dragged_point[0] = x;
  initial_dragged_point[1] = y;
  initial_dragged_point[2] = z;
}

void DragNodeOp::dragTo(double x,double y,double z){
  if(data_model){

	double disp[3];
	disp[0] = x-initial_dragged_point[0];
	disp[1] = y-initial_dragged_point[1];
	disp[2] = z-initial_dragged_point[2];

	Matrix<double,3,-1> u = initial_displacement;
	for (int i = 0; i < u.cols(); ++i){
	  u.col(i)[0] += disp[0];
	  u.col(i)[1] += disp[1];
	  u.col(i)[2] += disp[2];
	}
	data_model->updateUc(u,draggedGroupId);
  }
  if(viewer)
	viewer->update();
}
