#include "cluster.h"

#include <queue>
#include <boost/unordered_set.hpp>
#include <jtflib/mesh/mesh.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <cstdlib>
#include "../conf_para.h"
#include <omp.h>

using namespace std;
using namespace zjucad::matrix;
using jtf::mesh::edge2cell_adjacent;


//int cluster_machine::remove_extra_node()
//{
//    set<size_t> used_node_idx(mesh_.begin(), mesh_.end());
//    if (used_node_idx.size() == nodes_.size(2))
//        return 0;
//    matrix<size_t> used_node_mat(used_node_idx.size(), 1);
//    std::copy(used_node_idx.begin(), used_node_idx.end(), used_node_mat.begin());

//    map<size_t,size_t> p2p;
//    matrix<double> new_node(3, used_node_mat.size());

//    for (size_t pi = 0; pi < used_node_mat.size(); ++pi) {
//        new_node(colon(),pi) = nodes_(colon(), used_node_mat[pi]);
//        p2p[used_node_mat[pi]] = pi;
//    }
//    for (size_t pi = 0; pi < mesh_.size(); ++pi)
//        mesh_[pi] = p2p[mesh_[pi]];
//    nodes_ = new_node;
//    return 0;
//}

int cluster_machine::build_adj_list()
{
    boost::shared_ptr<edge2cell_adjacent> e2c(edge2cell_adjacent::create(mesh_));
    size_t edge_num = 2 * e2c->edges_.size();
    size_t max_node_idx = zjucad::matrix::max(mesh_);

    u_.resize(edge_num);
    v_.resize(edge_num);
    first_.resize(max_node_idx + 1);
    next_.resize(edge_num);

    for (size_t i = 0; i < first_.size(); ++i)
        first_[i] = -1;
    for (size_t i = 0; i < next_.size(); ++i)
        next_[i] = -1;

    //build undirected graph
    for (size_t i = 0; i < e2c->edges_.size(); ++i) {
        const size_t CURR = 2 * i;
        const size_t NEXT = CURR + 1;
        u_[CURR] = e2c->edges_[i].first;
        v_[CURR] = e2c->edges_[i].second;
        next_[CURR] = first_[u_[CURR]];
        first_[u_[CURR]] = CURR;

        u_[NEXT] = e2c->edges_[i].second;
        v_[NEXT] = e2c->edges_[i].first;
        next_[NEXT] = first_[u_[NEXT]];
        first_[u_[NEXT]] = NEXT;
    }
    return 0;
}

bool cluster_machine::stop()
{
    for (size_t i = 0; i < prime_.size(); ++i)
        if ( prime_[i].dis > 0.5 * R_ )
            return false;
    return true;
}

int cluster_machine::bfs(const size_t start)
{
    std::queue<state> q;
    boost::unordered_set<size_t> vis;

    char node[100];
    sprintf(node, "%lu.", start);
    state s{start, start, 0.0, string(node)};

    vis.insert(start);
    q.push(s);

    while ( !q.empty() ) {
        state curr = q.front();
        auxil_[curr.id] = curr;
        q.pop();
        for (size_t e = first_[curr.id]; e != -1; e = next_[e]) {
            assert(curr.id == u_[e]);
            if ( vis.find(v_[e]) == vis.end() ) {
                vis.insert(v_[e]);
                double edge_len = norm(nodes_(colon(), v_[e]) - nodes_(colon(), u_[e]));

                char node[100];
                sprintf(node, "%lu.", v_[e]);
                state nxt{v_[e], start, curr.dis + edge_len, curr.path + string(node)};

                q.push(nxt);
            }
        }
    }
    return 0;
}

bool cluster_machine::update(size_t &next)
{
    bool flag = false;
    double max_dis = -1.0;
    for (size_t i = 0; i < auxil_.size(); ++i) {
        if ( auxil_[i].dis < prime_[i].dis ) {
            flag = true;
            if ( auxil_[i].dis > max_dis ) {
                max_dis = auxil_[i].dis;
                next = auxil_[i].id;
            }
            prime_[i] = auxil_[i];
        }
    }
    return flag;
}

int cluster_machine::partition(const size_t num)
{
    build_adj_list();

    for (size_t i = 0; i < prime_.size(); ++i)
        prime_[i].dis = numeric_limits<double>::max();

    int N = num;
    size_t next = mesh_[0];
    while ( N-- ) {
        bfs(next);
        if ( !update(next) )
            break;
    }
    for (size_t i = 0; i < prime_.size(); ++i)
        regions_[prime_[i].center].push_back(std::make_pair(prime_[i].id, prime_[i].path));
    return 0;
}


void cluster_machine::dfs(const size_t curr, boost::unordered_set<size_t> &vis)
{
    vis.insert(curr);
    for (size_t e = first_[curr]; e != -1; e = next_[e]) {
        if ( vis.find(v_[e]) == vis.end() )
            dfs(v_[e], vis);
    }
}

size_t cluster_machine::connected_component()
{
    build_adj_list();
    boost::unordered_set<size_t> vis;
    size_t count = 0;
    for (size_t i = 0; i < mesh_.size(); ++i) {
        if ( vis.find(mesh_[i]) == vis.end() ) {
            count++;
            dfs(mesh_[i], vis);
        }
    }
    return count;
}


int calc_dist_to_center(const zjucad::matrix::matrix<double>                            &emb_node,
                        const std::map<size_t, std::vector<std::pair<size_t, string>>>  &regions,
                        std::map<size_t, std::vector<std::pair<size_t, double>>>        &region_dis)
{
    for (auto it = regions.begin(); it != regions.end(); ++it) {
        for (int k = 0; k < it->second.size(); ++k) {
            size_t idx = it->second[k].first;

            std::vector<string> str;
            boost::algorithm::split(str, it->second[k].second, boost::is_any_of("."), boost::token_compress_on);

            double dist = 0;
            if ( str.size() > 2 ) {
                for (size_t j = 0; j < str.size() - 2; ++j) {
                    size_t curr = atoi(str[j].c_str());
                    size_t next = atoi(str[j + 1].c_str());
                    dist += norm(emb_node(colon(), next) - emb_node(colon(), curr));
                }
            }
            region_dis[it->first].push_back(std::make_pair(idx, dist));
        }
    }
    return 0;
}


int calc_point_weight(const zjucad::matrix::matrix<double>                            &emb_node,
                      const std::map<size_t, std::vector<std::pair<size_t, string>>>  &regions,
                      std::vector<std::vector<std::pair<size_t, double>>>             &clt_w)
{
    clt_w.resize(regions.size());
    int count = 0;
    for (auto it = regions.begin(); it != regions.end(); ++it, ++count)
    {
        zjucad::matrix::matrix<double> coeff(it->second.size());

#pragma omp parallel for
        for (size_t k = 0; k < it->second.size(); ++k)
        {
            std::vector<string> str;
            boost::algorithm::split(str, it->second[k].second,
                                    boost::is_any_of("."), boost::token_compress_on);
            double dist = 0;
            if ( str.size() > 2 ) {
                for (size_t j = 0; j < str.size() - 2; ++j) {
                    size_t curr = atoi(str[j].c_str());
                    size_t next = atoi(str[j + 1].c_str());
                    dist += norm(emb_node(colon(), next) - emb_node(colon(), curr));
                }
            }
            coeff[k] = dist < 2 * __SIGMA ? exp(-0.5 * dist * dist / __SIGMA / __SIGMA) : 0;
        }

        coeff /= zjucad::matrix::sum(coeff);
        for (size_t k = 0; k < it->second.size(); ++k)
            if ( fabs(coeff[k]) > 1e-7 )
                clt_w[count].push_back(std::make_pair(it->second[k].first, coeff[k]));
    }
    return 0;
}

