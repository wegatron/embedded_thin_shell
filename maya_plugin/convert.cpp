#include "convert.h"

void MAYA_PLUGIN::Convert(const MPointArray &points_array, std::vector<double> &points_v)
{
  points_v.clear();
  for(size_t i=0; i<points_array.length(); ++i) {
    points_v.push_back(points_array[i].x/points_array[i].w);
    points_v.push_back(points_array[i].y/points_array[i].w);
    points_v.push_back(points_array[i].z/points_array[i].w);
  }
}
