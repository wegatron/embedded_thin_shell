#include "executor.h"

#include <maya/MGlobal.h>
#include <maya/MArgList.h>
#include "maya_op.h"

//QLineEdit RefineSoftbody::*startFrameEdit;
//QLineEdit RefineSoftbody::*endFrameEdit;
/// test import comannd
MStatus Importer::doIt( const MArgList& ){
  return ImportVrml();
}

void* Importer::Execute() {
  return new Importer();
}

/// test exporter command
MStatus Exporter::doIt( const MArgList& args){
  MStatus stat;
  int start_frame = args.asInt(0, &stat);
  int end_frame = args.asInt(1, &stat);
  return ExportSelect2Vrml(start_frame, end_frame, EXPORT_PREFIX);
}

void* Exporter::Execute() {
  return new Exporter();
}

/// test refine command
MStatus Refine::doIt( const MArgList& ){
  // get frame 0 data
  // init embedded thin shell simulator
  // for i=1 to last
  //     get data of frame i
  //   refine export
  // import all vrml2 file
  return MS::kSuccess;
}

void* Refine::Execute() {
  return new Refine();
}
