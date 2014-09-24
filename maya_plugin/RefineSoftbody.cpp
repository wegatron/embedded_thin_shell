#include <assert.h>

#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <QtGui/QFileDialog>
#include <maya/MObject.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

#include "maya_op.h"
#include "ani_mesh.h"
#include "moc_RefineSoftbody.h"
#include "maya_plug_setting.h"

MObject AniMesh::time;
MObject AniMesh::outputMesh;
MTypeId AniMesh::id( 0x80000 );

using namespace std;
RefineSoftbody::RefineSoftbody(QWidget *parent): QWidget(parent)
{
	browseButton = createButton(tr("&Browse..."),SLOT(browse()));
	executeButton= createButton(tr("&Execute"),SLOT(extcute()));

	startFrameEdit = new QLineEdit("0");
	endFrameEdit= new QLineEdit("100");
	// directoryComboBox = createComboBox(QDir::currentPath());
	
	directoryComboBox = createComboBox("C:\\MinGW\\msys\\home\\wegatron\\workspace\\embedded_thin_shell\\branches\\zhangshengwei\\dat\\maya_plugin");
	
	subEdit = new QLineEdit("1");
	startFrameLable = new QLabel(tr("Start Frame:"));
	endFrameLable = new QLabel(tr("End Frame:"));
	directoryLabel = new QLabel(tr("VRML File directory:"));
	sublable = new QLabel(tr("sub(int):"));

	QGridLayout *mainLayout = new QGridLayout;
	mainLayout->addWidget(startFrameLable, 0, 0);
	mainLayout->addWidget(startFrameEdit, 0, 1);

	mainLayout->addWidget(endFrameLable, 1, 0);
	mainLayout->addWidget(endFrameEdit, 1, 1);
	mainLayout->addWidget(directoryLabel, 2, 0);
	mainLayout->addWidget(directoryComboBox, 2, 1);
	mainLayout->addWidget(browseButton, 2, 2);
	mainLayout->addWidget(sublable, 3, 0, 1, 2);
	mainLayout->addWidget(subEdit, 3, 1);
	mainLayout->addWidget(executeButton,4,1);
	setLayout(mainLayout);

	setWindowTitle(tr("RefineSoftbody"));
	resize(700, 250);
}

void RefineSoftbody::browse()
{
	
	//QString directory = QFileDialog::getExistingDirectory(this, tr("Find Vrml Files"), QDir::currentPath());
	//if (!directory.isEmpty()) {
	//	if (directoryComboBox->findText(directory) == -1)
	//		directoryComboBox->addItem(directory);
	//	directoryComboBox->setCurrentIndex(directoryComboBox->findText(directory));
	//}

	QString default_dir("C:\\MinGW\\msys\\home\\wegatron\\workspace\\embedded_thin_shell\\branches\\zhangshengwei\\dat\\maya_plugin");
	QString directory = QFileDialog::getExistingDirectory(this, tr("Find Vrml Files"), default_dir);
	if (!directory.isEmpty()) {
		if (directoryComboBox->findText(directory) == -1)
			directoryComboBox->addItem(directory);
		directoryComboBox->setCurrentIndex(directoryComboBox->findText(directory));
	}
}
void RefineSoftbody::extcute()
{

  QString startFrame = startFrameEdit->text();
  QString endFrame = endFrameEdit->text();
  QString vrmlDirectory = directoryComboBox->currentText();
  //QString filepath=directoryComboBox->currentText();
  int sub= subEdit->text().toInt();
  string strstartFrame = startFrame.toStdString();
  string strendFrame = endFrame.toStdString();
  std::string strvrmlDirectory = vrmlDirectory.toStdString();

  string strExpPerfix = vrmlDirectory.toStdString().append("\\shell_ori");
  string strImpPerfix = vrmlDirectory.toStdString().append("\\shell_out");
  setImportPrefix(strImpPerfix);
  int start=atoi(strstartFrame.c_str());
  int end = atoi(strendFrame.c_str());

  ExportSelect2Vrml(0,0,strExpPerfix.c_str());
  ExportSelect2Vrml(start, end,strExpPerfix.c_str());
  
  // remove old files and generate new files
  int res = -1;
  HANDLE hThread = DoRefine(strExpPerfix, strImpPerfix, sub, start, end, res);
  LoadUpdate(strImpPerfix, start, &res);
  if(res > 0){
    std::ofstream log(LOG_PATH, ofstream::app);
    log << "exec cmd error! in frame: " << start << endl;
    log.close();
    return;
  }
  ImportVrml();
  for (int c_frame= start+1; c_frame<=end; ++c_frame) {
    LoadUpdate(strImpPerfix, c_frame, &res);
    if(res>0){
      std::ofstream log(LOG_PATH, ofstream::app);
      log << "exec cmd error! in frame: " << c_frame << endl;
      log.close();
      return;
    }
  }
  assert(res == 0);
}

static void updateComboBox(QComboBox *comboBox)
{
	if (comboBox->findText(comboBox->currentText()) == -1)
		comboBox->addItem(comboBox->currentText());
}


QPushButton *RefineSoftbody::createButton(const QString &text,const char *member)
{
	QPushButton *button = new QPushButton(text);
	connect(button, SIGNAL(clicked()), this, member);
	return button;
}

QComboBox *RefineSoftbody::createComboBox(const QString &text)
{
	QComboBox *comboBox = new QComboBox;
	comboBox->setEditable(true);
	comboBox->addItem(text);
	comboBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
	return comboBox;
}




/*********************************************************************************
                             RefineSoftbodyCmd  class
*********************************************************************************/

QPointer<RefineSoftbody>	RefineSoftbodyCmd::window;

const MString			RefineSoftbodyCmd::commandName("RefineSoftbody");


//	Destroy the button window, if it still exists.
void RefineSoftbodyCmd::cleanup()
{
	if (!window.isNull()) delete window;
}


MStatus RefineSoftbodyCmd::doIt(const MArgList& /* args */)
{
	
	
	QPushButton* browsBtton;
	QPushButton* exeCuteButton;
	if (window.isNull()) {
		window = new RefineSoftbody();
		window->show();
	}
	else {
		window->showNormal();
		window->raise();
		browsBtton = window->browseButton;
		exeCuteButton= window->executeButton;
		browsBtton->connect(browsBtton,SIGNAL(clicked(bool)),browsBtton,SLOT(window->browse()));
		exeCuteButton->connect(exeCuteButton,SIGNAL(clicked(bool)),exeCuteButton,SLOT(window->extcute()));
	}
	return MS::kSuccess;
}

MStatus initializePlugin(MObject plugin)
{
	MStatus		st;
	MFnPlugin	pluginFn(plugin, "Autodesk, Inc.", "1.0", "Any", &st);

	if (!st) {
		MGlobal::displayError(
			MString("helixQtCmd - could not initialize plugin: ")+ st.errorString());
		return st;
	}

	//	Register the command.
	st = pluginFn.registerNode("AniMesh", AniMesh::id,AniMesh::creator, AniMesh::initialize);
	st = pluginFn.registerCommand(RefineSoftbodyCmd::commandName, RefineSoftbodyCmd::creator);

	if (!st) {
		MGlobal::displayError(
			MString("RefineSoftbodyCmd - could not register '")
			+ RefineSoftbodyCmd::commandName + "' command: "
			+ st.errorString()
			);
		return st;
	}

	return st;
}


MStatus uninitializePlugin(MObject plugin)
{
	MStatus		st;
	MFnPlugin	pluginFn(plugin, "Autodesk, Inc.", "1.0", "Any", &st);

	if (!st) {
		MGlobal::displayError(
			MString("RefineSoftbodyCmd - could not uninitialize plugin: ")
			+ st.errorString()
			);
		return st;
	}

	//	Make sure that there is no UI left hanging around.
	RefineSoftbodyCmd::cleanup();

	//	Deregister the command.
	st = pluginFn.deregisterCommand(RefineSoftbodyCmd::commandName);
	st = pluginFn.deregisterNode(AniMesh::id);
	if (!st) {
		MGlobal::displayError(
			MString("RefineSoftbodyCmd - could not deregister '")
			+ RefineSoftbodyCmd::commandName + "' command: "
			+ st.errorString()
			);
		return st;
	}

	return st;
}
