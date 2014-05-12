#include "domain_decomposition.h"

#include <fstream>
#include <jtflib/mesh/mesh.h>
#include "src/shell_deformer/cluster.h"
#include "common/vtk.h"


using namespace std;
using namespace zjucad::matrix;
using jtf::mesh::one_ring_face_at_point;
using jtf::mesh::edge2cell_adjacent;

int gen_patches(const zjucad::matrix::matrix<size_t>       &shell_mesh,
                const zjucad::matrix::matrix<double>       &shell_nodes,
                const size_t                               nodes_number,
                const vector<vector<pair<size_t, double>>> &regions,
                vector<tri_mesh_ptr>                       &patches)
{
    cout << "[INFO] the program is generate patches\n";
    vector<vector<size_t>> patch_pts;
    vector<vector<vector<pair<size_t, double>>>> patch_cons;
    cout << "[INFO] step 1\n";
    if ( get_patch_nodes(regions, nodes_number, patch_pts, patch_cons) )
        return __LINE__;
    cout << "[INFO] step 2\n";
    if ( gen_patch_tri_mesh(shell_mesh, shell_nodes, patch_pts, patch_cons, patches) )
        return __LINE__;
    cout << "[INFO] generating patches done\n";
    return 0;
}

int get_patch_nodes(const vector<vector<pair<size_t, double>>>   &regions,
                    const size_t                                 nodes_num,
                    vector<vector<size_t>>                       &patch_pts,
                    vector<vector<vector<pair<size_t, double>>>> &patch_cons)
{
    patch_pts.resize(nodes_num);
    patch_cons.resize(nodes_num);
#define __JUMP_MERGE__
#ifdef __JUMP_MERGE__
    for (size_t s = 0; s < nodes_num; ++s) {
        for (size_t i = s; i < regions.size(); i += nodes_num) {
            patch_cons[s].push_back(regions[i]);
            for (size_t j = 0; j < regions[i].size(); ++j)
                patch_pts[s].push_back(regions[i][j].first); //interior point
        }
    }
#else
    int cnt = regions.size() / nodes_num;
    size_t i;
    for ( i = 0; i < cnt * nodes_num; ++i) {;
        patch_cons[i / cnt].push_back(regions[i]);
        for (size_t j = 0; j < regions[i].size(); ++j)
            patch_pts[i / cnt].push_back(regions[i][j].first);
    }
    int offset = 0;
    for (; i < regions.size(); ++i, ++offset) {
        patch_cons[nodes_num - 1 - offset].push_back(regions[i]);
        for (size_t j = 0; j < regions[i].size(); ++j)
            patch_pts[nodes_num - 1 - offset].push_back(regions[i][j].first);
    }
#endif
    return 0;
}

int gen_patch_tri_mesh(const zjucad::matrix::matrix<size_t>               &shell_mesh,
                       const zjucad::matrix::matrix<double>               &shell_nodes,
                       const vector<vector<size_t>>                       &patch_pts,
                       const vector<vector<vector<pair<size_t, double>>>> &patch_cons,
                       vector<tri_mesh_ptr>                               &patch_mesh)
{
    cout << "[INFO] the program is construct the mesh of the current patch\n";
    shared_ptr<edge2cell_adjacent> e2c(edge2cell_adjacent::create(shell_mesh));
    shared_ptr<one_ring_face_at_point> f2p(new one_ring_face_at_point);
    f2p->add_all_faces(shell_mesh, *e2c);

    assert(patch_cons.size() == patch_pts.size());

    for (size_t i = 0; i < patch_pts.size(); ++i) {
        tri_mesh_ptr tmp(new tri_mesh);
        boost::unordered_set<size_t> face_idx;
        boost::unordered_set<size_t> pts_idx;

        tmp->cons = patch_cons[i];
        tmp->interior.resize(patch_pts[i].size());
        std::copy(patch_pts[i].begin(), patch_pts[i].end(), tmp->interior.begin());

        for (size_t j = 0; j < patch_pts[i].size(); ++j) {
            size_t pid = patch_pts[i][j];
            for (size_t k = 0; k < f2p->p2f_[pid].size(); ++k) {
                size_t fid = f2p->p2f_[pid][k];
                face_idx.insert(fid);
                pts_idx.insert(shell_mesh(0, fid));
                pts_idx.insert(shell_mesh(1, fid));
                pts_idx.insert(shell_mesh(2, fid));
            }
        }
        tmp->mesh.resize(3, face_idx.size());
        tmp->nodes.resize(3, pts_idx.size());

        size_t col = 0;
        for (auto it = pts_idx.begin(); it != pts_idx.end(); ++it, ++col) {
            tmp->nodes(colon(), col) = shell_nodes(colon(), *it);
            tmp->g2s.insert(make_pair(*it, col));
            tmp->s2g.insert(make_pair(col, *it));
        }
        col = 0;
        for (auto it = face_idx.begin(); it != face_idx.end(); ++it, ++col)
            for (size_t k = 0; k < 3; ++k)
                tmp->mesh(k, col) = tmp->g2s[shell_mesh(k, *it)];
        for (size_t j = 0; j < tmp->cons.size(); ++j)
            for (size_t k = 0; k < tmp->cons[j].size(); ++k)
                tmp->cons[j][k].first = tmp->g2s[tmp->cons[j][k].first];
        patch_mesh.push_back(tmp);
    }
    return 0;
}

void write_one_patch(const string               filename,
                     const vector<tri_mesh_ptr> &patches,
                     const size_t               id)
{
    std::ofstream os(filename.c_str());
    tri2vtk(os, &patches[id]->nodes[0], patches[id]->nodes.size(2),
                &patches[id]->mesh[0], patches[id]->mesh.size(2));
}


