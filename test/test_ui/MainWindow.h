#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtGui/QApplication>
#include <FileDialog.h>
#include <QGLViewerExt.h>
#include <VolObjCtrl.h>
#include <AniCtrl.h>
#include "PerturbationOp.h"
#include "DataModel.h"
#include "DataModelRender.h"
#include "SelectionOp.h"
#include "DragNodeOp.h"
#include <boost/shared_ptr.hpp>
using namespace QGLVEXT;
#include <test/test_ui/ui_simulator.h>

namespace SIMULATOR{
  
  class MainWindow: public QMainWindow{
	
	Q_OBJECT
	
  public:
	MainWindow(QWidget *parent=0,Qt::WFlags flags=0);
	void createComponents();
	void createConnections();
	void paserCommandLine();
	void loadInitFile(const string filename);

  public slots:
	void loadInitFile();
	
  private:
	Ui::MainWindow _mainwindow;
	pQGLViewerExt _viewer;
	pFileDialog _fileDialog;
	pPerturbationCtrl _perturb;

        pLocalframeManipulatoionCtrl manipulation_passive_obj;
	pPassiveObject _passiveObject;
	pDataModel _dataModel;
	pVolObjCtrl _volObjCtrl;
	pSimSelectionCtrl _selCtrl;
	pDragNodeCtrl _DragCtrl;
	pDataModelRenderCtrl _renderCtrl;
	pAniCtrl _aniCtrl;

	string init_filename;
  };
  
  typedef boost::shared_ptr<MainWindow> pMainWindow;
  
}//end of namespace

#endif /*_MAINWINDOW_H_*/

