#ifndef REFINESOFTBODY_H
#define REFINESOFTBODY_H

#include<QtCore/QPointer>
#include <maya/MPxCommand.h>
#include  <maya/MStatus.h>
#include <QtGui/QMainWindow>
#include <QtGui/QWidget>
#include <QtCore/QDir>
#include <QtGui/QComboBox>
#include <QtGui/QLineEdit>
#include <QtGui/QGridLayout>
#include <QtGui/QPushButton>
#include <QtGui/QLabel>
#include <string.h>

QT_BEGIN_NAMESPACE
class QComboBox;
class QLineEdit;
class QLabel;
class QGridLayout;
class QPushButton;
class QTableWidget;
class QTableWidgetItem;
QT_END_NAMESPACE

	//! [0]
class RefineSoftbody : public QWidget
{
	Q_OBJECT

public:
	RefineSoftbody(QWidget *parent = 0);
	QStringList findFiles(const QStringList &files, const QString &text);
	//QPushButton *createButton(const QString &text);
	QPushButton *createButton(const QString &text, const char *member);
	QComboBox *createComboBox(const QString &text = QString());

	QLineEdit *startFrameEdit;
	QLineEdit *endFrameEdit;
	QComboBox* directoryComboBox;
	QLineEdit *subEdit;
	QLabel *startFrameLable;
	QLabel *endFrameLable;
	QLabel *directoryLabel;
	QLabel *sublable;
	QPushButton *browseButton;
	QPushButton *executeButton;
	//QTableWidget *filesTable;
	

	QDir currentDir;
public slots:
		void browse();
		void extcute();

//
//private:
//	QStringList findFiles(const QStringList &files, const QString &text);
//	QPushButton *createButton(const QString &text, const char *member);
//	QComboBox *createComboBox(const QString &text = QString());
//
//	QLineEdit *startFrameEdit;
//	QLineEdit *endFrameEdit;
//	QComboBox* directoryComboBox;
//	QLineEdit *subEdit;
//	QLabel *startFrameLable;
//	QLabel *endFrameLable;
//	QLabel *directoryLabel;
//	QLabel *sublable;
//	QPushButton *browseButton;
//	QPushButton *executeButton;
//	//QTableWidget *filesTable;
//
//	QDir currentDir;
};
class RefineSoftbodyCmd : public MPxCommand
{
public:
	static void		cleanup();
	static void*	creator()		{ return new RefineSoftbodyCmd(); }

	MStatus			doIt(const MArgList& args);

	static QPointer<RefineSoftbody>	window;
	static const MString			commandName;
};
#endif