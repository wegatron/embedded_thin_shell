#ifndef EXECUTOR_H
#define EXECUTOR_H

#include <maya/MPxCommand.h>

class MArgList;

class Importer : public MPxCommand
{
public:
	Importer() {};
	~Importer(){};
    MStatus     doIt( const MArgList& );
    static      void* Execute();
};

class Exporter : public MPxCommand
{
public:
	Exporter() {}
	~Exporter() {}
	// export select frame from arg0 to arg1
	MStatus doIt(const MArgList &);
	static void* Execute();
};

class Refine : public MPxCommand
{
public:
	Refine() {}
	~Refine() {}
	MStatus doIt(const MArgList &);
	static void* Execute();
};

#endif /* EXECUTOR_H */