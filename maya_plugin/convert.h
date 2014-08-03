#ifndef CONVERT_H
#define CONVERT_H

#include <vector>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>

namespace MAYA_PLUGIN {
  void Convert (const MPointArray &point_array, std::vector<double> &points_v);
}

#endif /* CONVERT_H */
