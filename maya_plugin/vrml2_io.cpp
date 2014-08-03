#include <stdio.h>
#include <string.h>
#include "vrml2_io.h"

using namespace std;

int ExportVrml2 (const string& filename, const std::vector<size_t> &face_v, const std::vector<double> &points_v)
{
  FILE *file_wrl;

  if ((file_wrl = fopen(filename.c_str(),"w"))==NULL)//creat dest file(**.obj)
    {printf("Cannot create the destination vrml_file, please check the filename and path.\n");exit(1);}

  fprintf(file_wrl,"#VRML V2.0 utf8\n");
  fprintf(file_wrl,"Shape {\n");
  fprintf(file_wrl,"	geometry IndexedFaceSet	{\n");
  fprintf(file_wrl,"	coord Coordinate {\n");
  fprintf(file_wrl,"	point [\n");
  for(vector<double>::const_iterator it = points_v.begin();it != points_v.end();it+=3)
    {
      fprintf(file_wrl,"%.6lf %.6lf %.6lf\n",*it,*(it+1),*(it+2));
    }
  fprintf(file_wrl,"]\n");
  fprintf(file_wrl,"}\n");
  fprintf(file_wrl,"coordIndex [\n");
  for(vector<size_t>::const_iterator it = face_v.begin();it != face_v.end();it+=3)
    {
      fprintf(file_wrl,"%d %d %d -1\n",*it,*(it+1),*(it+2));
    }
  fprintf(file_wrl,"]\n");
  fprintf(file_wrl,"}\n");
  fprintf(file_wrl,"}\n");
  fclose(file_wrl);
  return 0;
}

int ImportVrml2 (const string& filename, std::vector<size_t> &face_v, std::vector<double> &points_v)
{
  FILE *file_wrl;
  double x,y,z;
  int a,b,c,d;
  char str[100];
  if ((file_wrl = fopen(filename.c_str(),"r"))==NULL)//creat dest file(**.obj)
    {printf("Cannot create the destination vrml_file, please check the filename and path.\n");exit(1);}
  while(fscanf(file_wrl,"%s",str) == 1)
    {
      if(strcmp(str,"point") == 0)
        {
          fscanf(file_wrl,"%s",str);
          while(fscanf(file_wrl,"%lf %lf %lf",&x,&y,&z) == 3)
            {
              points_v.push_back(x);
              points_v.push_back(y);
              points_v.push_back(z);
            }
        }
      if(strcmp(str,"coordIndex") == 0)
        {
          fscanf(file_wrl,"%s",str);
          while(fscanf(file_wrl,"%d %d %d %d",&a,&b,&c,&d) == 4)
            {
              face_v.push_back(a);
              face_v.push_back(b);
              face_v.push_back(c);
            }
        }

    }
  fclose(file_wrl);
  return 0;
}
