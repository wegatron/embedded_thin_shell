#ifndef _AXISTORUS_H_
#define _AXISTORUS_H_

#include <boost/shared_ptr.hpp>
#include <SelfRenderEle.h>
#include <Selectable.h>
namespace QGLVEXT{
  
  /**
   * @class AxisTorus
   * 
   */
  class AxisTorus:public SelfRenderEle, public Selectable{
	
  public: 
	AxisTorus(){
	  selected_axis = -1;
	}
	void draw()const;
	int totalEleNum ()const{
	  return 3;
	}
	void drawWithNames ()const;
	void selectAxis(int a){
	  selected_axis = a;
	}
	
  protected:
	void drawTorus(float x, float y, float z,
				   float r, float g, float b,
				   const float scalor)const;
	
  private:
	int selected_axis;
	
  };
  
  typedef boost::shared_ptr<AxisTorus> pAxisTorus;
  
}//end of namespace

#endif /* _AXISTORUS_H_ */
