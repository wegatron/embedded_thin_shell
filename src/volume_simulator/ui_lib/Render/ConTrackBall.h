#ifndef _CONTRACKBALL_H_
#define _CONTRACKBALL_H_

#include <boost/shared_ptr.hpp>
#include <QGLViewerExt.h>
#include <SelfRenderEle.h>
#include <AxisTorus.h>

namespace QGLVEXT{

  /**
   * @class ConTrackBall constrained track ball. the rotation will be
   * constrained to selected axis.
   * 
   */
  class ConTrackBall: public QObject, public MouseGrabber{

	Q_OBJECT

  public:
  	ConTrackBall(pQGLViewerExt viewer);
  	void checkIfGrabsMouse(int x, int y, const Camera* const camera);

  public slots:
  	void selectAxises(const vector<int> sel_group_ids);
  	void press(QMouseEvent* e);
  	void release(QMouseEvent* e);
  	void ShowConTrackBall();

  private:
  	pQGLViewerExt viewer;
  	pAxisTorus p_AxisTorus;
  	AxisPlaneConstraint* constraint;

#endif /*_CONTRACKBALL_H_*/

  	AxisPlaneConstraint::Type grabbed_type;
  	short constrained_axi; // 0: x, 1: y, 2: z, -1:none.
	bool m_show;
	bool m_hit;
  };
  
  typedef boost::shared_ptr<ConTrackBall> pConTrackBall;
  
}//end of namespace