#include <stdlib.h>
#include <QtGui/QApplication>
#include <Log.h>
#include <QInputEventRecorderCmd.h>
#include "MainWindow.h"

using namespace SIMULATOR;
using namespace UTILITY;

int main(int argc, char *argv[]){

  QApplication application(argc,argv);
  QMainWindow w;
  MainWindow m_mainw(&w);
  
  // auto record and replay
  pQInputEventRecorderCmd recorder;
  if(argc >= 3){
  	recorder = pQInputEventRecorderCmd(new QInputEventRecorderCmd(&w));
  	if (3 == argc)
  	  recorder->setCmd(argv[argc-1]);
  	else if(4 == argc)
  	  recorder->setCmd(argv[argc-2],argv[argc-1]);
  }

  m_mainw.show();
  return  application.exec();
}
