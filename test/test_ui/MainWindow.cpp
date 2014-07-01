#include "MainWindow.h"
using namespace SIMULATOR;

MainWindow::MainWindow(QWidget *parent,Qt::WFlags flags):QMainWindow(parent,flags){
	  
  createComponents();
  createConnections();
  paserCommandLine();
}

void MainWindow::createComponents(){

  _mainwindow.setupUi(this);
  _viewer = _mainwindow.left_view;
  _fileDialog = pFileDialog(new FileDialog(this));
  _viewer->show3DGrid();
  // _viewer->startAnimation();

  // data model	  
  pTetMeshEmbeding embeding = pTetMeshEmbeding(new TetMeshEmbeding());
  _volObjCtrl = pVolObjCtrl(new VolObjCtrl(this,embeding));
  _dataModel = pDataModel(new DataModel(embeding));

  // render
  _renderCtrl = pDataModelRenderCtrl(new DataModelRenderCtrl(_viewer,_dataModel));

  // selection, drag
  _selCtrl = pSimSelectionCtrl(new SimSelectionCtrl(_viewer,_dataModel));
  _perturb = pPerturbationCtrl(new PerturbationCtrl(_viewer,_dataModel));
  _DragCtrl = pDragNodeCtrl(new DragNodeCtrl(_viewer, _dataModel));

  // passive objects
  _passiveObject = pPassiveObject(new PassiveBall(_viewer));
  _passiveObject->move(-0.3,0,1.0);
  manipulation_passive_obj = pLocalframeManipulatoionCtrl(new LocalframeManipulatoionCtrl(_viewer, _passiveObject));
  manipulation_passive_obj->setEnable(true);
  _viewer->addSelfRenderEle(_passiveObject);
  _dataModel->setPassiveObject(_passiveObject);

  _selCtrl->setEnable(true);
}

void MainWindow::createConnections(){
  
  connect(_mainwindow.actionLoadInitFile, SIGNAL(triggered()),this,SLOT(loadInitFile()));

  connect(_mainwindow.actionLoadObj, SIGNAL(triggered()), _volObjCtrl.get(), SLOT(loadObjMesh()));
  connect(_mainwindow.actionLoadVol, SIGNAL(triggered()), _volObjCtrl.get(), SLOT(loadVolMesh()));
  connect(_volObjCtrl.get(),SIGNAL(resetSceneMsg(double,double,double,double,double,double)),
		  _viewer,SLOT(resetSceneBoundBox(double,double,double,double,double,double)));
  connect(_mainwindow.actionPrepareSimulation,SIGNAL(triggered()),
		  _dataModel.get(),SLOT(prepareSimulation()));
  
  // selection
  connect(_mainwindow.actionSelectNodes, SIGNAL(triggered()), _selCtrl.get(), SLOT(selectNodes()));
  connect(_mainwindow.actionPrintSelected, SIGNAL(triggered()), _selCtrl.get(), SLOT(togglePrintSelEle()));

  // render
  connect(_mainwindow.actionObjMesh,SIGNAL(triggered()),_renderCtrl.get(),SLOT(toggleShowObj())); 
  connect(_mainwindow.actionVolMesh,SIGNAL(triggered()),_renderCtrl.get(),SLOT(toggleShowVol()));
  connect(_mainwindow.actionConNodes,SIGNAL(triggered()),_renderCtrl.get(),SLOT(toggleShowConNodes())); 
  connect(_mainwindow.actionShellMesh,SIGNAL(triggered()),_renderCtrl.get(),SLOT(toggleShowShell()));


  // simulation
  connect(_mainwindow.actionSimulate,SIGNAL(triggered()),_dataModel.get(),SLOT(simulate()));
  connect(_viewer,SIGNAL(updateAnimation()),_dataModel.get(),SLOT(simulate()));
  connect(_mainwindow.actionPauseSimulation,SIGNAL(triggered()),_viewer,SLOT(pauseAnimation()));
  connect(_mainwindow.actionReset,SIGNAL(triggered()),_dataModel.get(),SLOT(reset()));

}

void MainWindow::paserCommandLine(){
  QStringList cmdline_args = QCoreApplication::arguments();
  if( cmdline_args.size() >= 2 ){
	const string init_filename = cmdline_args.at(1).toStdString();
	loadInitFile(init_filename);
  }
}

void MainWindow::loadInitFile(const string filename){

  this->init_filename = filename;
  if(boost::filesystem::exists(filename)){

	DEBUG_LOG("begin to load vol_obj");
	bool succ = _volObjCtrl->initialize(filename);
        DEBUG_LOG("begin to init _dataModel");
	succ &= _dataModel->loadSetting(filename);
	_dataModel->prepareSimulation();
        
	JsonFilePaser jsonf;
	jsonf.open(filename);
	double collision_k_stiffness, collision_k_limit, con_penalty;
	jsonf.read("con_penalty", con_penalty, 1.0);
	_perturb->setPerturCompilance(con_penalty);
	INFO_LOG("con_penalty: "<<con_penalty );
	_fileDialog->warning(succ);

        
        
        string passive_obj_file;
	if (_passiveObject&&jsonf.readFilePath("passive_object", passive_obj_file, true)){
          _passiveObject->load(passive_obj_file);
          double collision_penalty = 1.0f;
          if (jsonf.read("collision_penalty", collision_penalty)){
            _passiveObject->setCollisionPenalty(collision_penalty);
          }
          if (jsonf.read("collision_k_stiffness", collision_k_stiffness)){
            _passiveObject->setKStiffness(collision_k_stiffness);
          }
          if (jsonf.read("collision_k_limit", collision_k_limit)){
            _passiveObject->setKLimit(collision_k_limit);
          }

	}
  }else{

	ERROR_LOG("file " << filename <<" is not exist!" );
	_fileDialog->warning(false);
  }
}

void MainWindow::loadInitFile(){

  const string filename = _fileDialog->load("ini");
  if(filename.size() >0) loadInitFile(filename);
}
