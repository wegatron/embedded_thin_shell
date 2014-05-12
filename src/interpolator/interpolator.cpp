#include "interpolator.h"
#include <jtflib/mesh/mesh.h>
#include <jtflib/mesh/util.h>
#include <sxxlib/tri_sub/tri_subdivision.h>
#include <ANN/ANN.h>
#include <hjlib/math/blas_lapack.h>
#include <zjucad/matrix/lapack.h>
#include <zjucad/matrix/io.h>
#include <zjucad/matrix/io.h>

using namespace zjucad::matrix;
using namespace std;
using namespace hj::sparse;

int remove_extra_node(matrix<size_t>     &mesh_,
                      matrix<double>     &nodes_)
{
    set<size_t> used_node_idx(mesh_.begin(), mesh_.end());
    if (used_node_idx.size() == nodes_.size(2))
        return 0;
    matrix<size_t> used_node_mat(used_node_idx.size(), 1);
    std::copy(used_node_idx.begin(), used_node_idx.end(), used_node_mat.begin());

    map<size_t,size_t> p2p;
    matrix<double> new_node(3, used_node_mat.size());

    for (size_t pi = 0; pi < used_node_mat.size(); ++pi) {
        new_node(colon(),pi) = nodes_(colon(), used_node_mat[pi]);
        p2p[used_node_mat[pi]] = pi;
    }
    for (size_t pi = 0; pi < mesh_.size(); ++pi)
        mesh_[pi] = p2p[mesh_[pi]];
    nodes_ = new_node;
    return 0;
}

int gen_outside_shell(const zjucad::matrix::matrix<size_t>  &tet_mesh,
                      const zjucad::matrix::matrix<double>  &tet_nodes,
                      zjucad::matrix::matrix<size_t>        &shell,
                      zjucad::matrix::matrix<double>        &shell_nodes,
                      zjucad::matrix::matrix<double>        &shell_normal,
                      const size_t                          sub_time,
                      const double                          embed_dist)
{
    using jtf::mesh::face2tet_adjacent;
    using jtf::mesh::edge2cell_adjacent;
    boost::shared_ptr<face2tet_adjacent> f2t(face2tet_adjacent::create(tet_mesh));
    matrix<double> surf_normal;

    jtf::mesh::get_outside_face(*f2t, shell, true);
    shell_nodes = tet_nodes;

    remove_extra_node(shell, shell_nodes);

    jtf::mesh::cal_point_normal(shell, shell_nodes, surf_normal);

    shell_nodes -= embed_dist * surf_normal;

    matrix<size_t> new_face;
    matrix<double> new_node;
    std::vector<size_t> add_node;
    int N = sub_time;
    while ( N-- ) {
        boost::shared_ptr<edge2cell_adjacent> e2c(edge2cell_adjacent::create(shell, false));
        std::vector<std::pair<size_t, size_t>> edges(e2c->edges_);
        for (auto it = edges.begin(); it != edges.end(); ++it)
            if ( it->first > it->second )
                std::swap(it->first, it->second);
        sxx::sub_edge(shell_nodes, shell, edges, new_node, new_face, add_node);
        shell = new_face;
        shell_nodes = new_node;
    }

    jtf::mesh::reorder_face(shell);
    jtf::mesh::cal_point_normal(shell, shell_nodes, shell_normal);
    return 0;
}

int tet_embed(const matrix<double>    &v,
              const matrix<size_t>    &tet,
              const matrix<double>    &pts,
              spm_csc<double>         &coef)
{
    const int tn = tet.size(2), pn = pts.size(2);

    vector<double*> pv(tn);
    matrix<double> tet_center(3, tn);
    {
        for(int i = 0; i < tn; ++i) {
            tet_center(colon(), i) = v(colon(), tet(colon(), i))*ones<double>(4, 1)/4;
            pv[i] = &tet_center(0, i);
        }
    }

    auto_ptr<ANNkd_tree> kdt(new ANNkd_tree(&pv[0], tn, v.size(1), 32));
    matrix<matrix<double> > bary_op(tn);
    {
        for(int i = 0; i < tn; ++i) {
            matrix<double> v44 = ones<double>(4, 4);
            for(int j = 0; j < 4; ++j)
                v44(colon(0, 2), j) = v(colon(), tet(j, i));
            inv(v44);
            bary_op[i] = v44;
        }
        cout << "create bary-coor operators success." << endl;
    }

    // alloc the sparse matrix
    coef.resize(v.size(2), pn, pn*4);
    coef.ptr() = colon(0, pn)*4;

    matrix<double> pt, w;
    const int ave_k = 40, iter_n = 4;
    const int max_k = static_cast<int>(40*floor(pow(2.0, iter_n)+0.5));
    matrix<double> dist(max_k);
    matrix<int> idx(max_k);
    double min_good = 1;
    int outside_cnt = 0;
    for(int pi = 0; pi < pn; ++pi) {
        if((pi%1000) == 0)
            cerr << "process " << pi << endl;

        pt = pts(colon(), pi);
        pair<int, double> best_t(-1, -10);
        for(int ki = 0, k = ave_k; ki < iter_n && k < max_k; ++ki, k*=2) {
            if(k > max_k)
                k = max_k;
            const double r2 = 1e1;
            kdt->annkSearch(&pt[0], max_k, &idx[0], &dist[0], 1e-10);
            for(int ti = (k > 40)?k/2:0; ti < k; ++ti) {
                int t_idx = idx[ti];
                w = bary_op[t_idx](colon(0, 3), colon(0, 2))*pt + bary_op[t_idx](colon(), 3);
                double good = min(w);
                if(best_t.second < good) {
                    best_t.second = good;
                    best_t.first = t_idx;
                }
                if(best_t.second >= 0)
                    break;
            }
            if(best_t.second >= 0)
                break;
        }
        if(best_t.second < 0)
            ++outside_cnt;
        if(best_t.second < min_good)
            min_good = best_t.second;
        if(best_t.first < 0) {
            cout << "Wow, very bad point!!" << endl;
            return __LINE__;
        }
        w = bary_op[best_t.first](colon(0, 3), colon(0, 2))*pt + bary_op[best_t.first](colon(), 3);
        if(fabs(sum(w)-1) > 1e-9) {
            cout << "strange weight." << trans(w);
            cout << "sum : " << sum(w) << endl;
        }

        coef.idx()(colon(coef.ptr()[pi], coef.ptr()[pi+1]-1)) = tet(colon(), best_t.first);
        coef.val()(colon(coef.ptr()[pi], coef.ptr()[pi+1]-1)) = w;
    }
    cout << "outside pt num is: " << outside_cnt << " min_good is: " << min_good << endl;

    return 0;
}
