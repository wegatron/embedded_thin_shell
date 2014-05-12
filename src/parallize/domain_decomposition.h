#ifndef __CJ_DOMAIN_DECOMPOSITION_H__
#define __CJ_DOMAIN_DECOMPOSITION_H__

#include <zjucad/matrix/matrix.h>
#include <boost/unordered_map.hpp>

/**
  * input: vector<size_t, vector<size_t, double>> regions;
  * output: vector<triangle_mesh> patches;
  *
 **/

using namespace std;

typedef struct tri_mesh {
    zjucad::matrix::matrix<size_t>         mesh;
    zjucad::matrix::matrix<double>         nodes;
    std::vector<size_t>                    interior;
    boost::unordered_map<size_t, size_t>   g2s;
    boost::unordered_map<size_t, size_t>   s2g;
    vector<vector<pair<size_t, double>>>   cons;
}tri_mesh;

typedef shared_ptr<tri_mesh> tri_mesh_ptr;


int get_patch_nodes(const vector<vector<pair<size_t, double>>>   &regions,
                    const size_t                                 nodes_number,
                    vector<vector<size_t>>                       &patch_pts,
                    vector<vector<vector<pair<size_t, double>>>> &patch_cons);


int gen_patch_tri_mesh(const zjucad::matrix::matrix<size_t>               &shell_mesh,
                       const zjucad::matrix::matrix<double>               &shell_nodes,
                       const vector<vector<size_t>>                       &patch_pts,
                       const vector<vector<vector<pair<size_t, double>>>> &patch_cons,
                       vector<tri_mesh_ptr>                               &patch_mesh);

int gen_patches(const zjucad::matrix::matrix<size_t>        &shell_mesh,
                const zjucad::matrix::matrix<double>        &shell_nodes,
                const size_t                                nodes_number,
                const vector<vector<pair<size_t, double>>>  &regions,
                vector<tri_mesh_ptr>                        &pathes);


void write_one_patch(const string                filename,
                     const vector<tri_mesh_ptr>  &patches,
                     const size_t                id);



























#endif
