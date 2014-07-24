#include <iostream>

// #include <Timer.h>

#include <FullStVKSimulator.h>

// #include <SubspaceSimulator.h>
#include <jtflib/mesh/io.h>
#include <zjucad/matrix/io.h>
#include <common/vtk.h>
#include <omp.h>
#include <hjlib/util/hrclock.h>
#include "interpolator/interpolator.h"
#include "shell_deformer/cluster.h"
#include "shell_deformer/deformer.h"
#include "conf_para.h"


using namespace std;
using namespace UTILITY;
using namespace SIMULATOR;
using namespace zjucad::matrix;

void vtk2abq(const char *inpath, const char *outpath)
{
    matrix<size_t> tet;
    matrix<double> node;
    jtf::mesh::tet_mesh_read_from_zjumat(inpath, &node, &tet);
    node /= 2.0;
    std::ofstream os(outpath);
    os << "*NODE" << endl;
    for (size_t i = 0; i < node.size(2); ++i) {
        os << i + 1 << ", " << node(0, i) << ", " << node(1, i) << ", " << node(2, i) << endl;
    }
    os << "*ELEMENT, TYPE=C3D4" << endl;
    for (size_t i = 0; i < tet.size(2); ++i)
        os << i + 1 << ", " << tet(0, i) + 1 << ", " << tet(1, i) + 1 << ", " << tet(2, i) + 1 << ", " << tet(3, i) + 1 << endl;
    os << "*ELSET,ELSET=EALL,GENERATE" << endl;
    os << "1," << tet.size(2) << endl;
}

//#define ABQ
#ifdef ABQ
int main()
{
    vtk2abq(__POJ_BASE_PATH "dat/cylinder_16k/init.tet",
            __POJ_BASE_PATH "dat/cylinder_16k/mesh.abq");

    matrix<size_t> mesh;
    matrix<double> node;

    jtf::mesh::tet_mesh_read_from_zjumat(__POJ_BASE_PATH "dat/cylinder_16k/init.tet",
                                         &node, &mesh);
    std::ofstream os(__POJ_BASE_PATH "dat/cylinder_16k/init.vtk");
    node /= 2.0;
    std::ofstream os("/home/chenjiong/usr/WorkSpace/embedded_shell/dat/cylinder_16k/init.vtk");

    tet2vtk(os, &node[0], node.size(2), &mesh[0], mesh.size(2));
    cout << "[INFO]DONE!\n";
    return 0;
}
#else

int main(int argc, char *argv[])
{
    //************load tet mesh and elastic parameter*************************************************

    const string ini_file = __POJ_BASE_PATH "dat/sofa/simu_full.ini";
    JsonFilePaser jsonf;
    bool succ = jsonf.open(ini_file);
    assert(succ);
    pTetMesh tet_mesh = pTetMesh(new TetMesh());
    {
        //load tet mesh
        string vol_file;
        succ = jsonf.readFilePath("vol_file",vol_file);
        assert(succ);
        succ = tet_mesh->load(vol_file);
        assert(succ);                                                                                                                                                                    //

        //load parameters
        string mtl_file;
        jsonf.readFilePath("elastic_mtl",mtl_file);
        succ = tet_mesh->loadElasticMtl(mtl_file);
        assert(succ);
    }
    //************************************************************************************************


    //************format transition*******************************************************************
        std::vector<size_t>    temp_cell;
        std::vector<double>    temp_node;
        matrix<size_t>         tet_cell;
        matrix<double>         tet_nodes;
        tet_mesh->tets(temp_cell);
        tet_mesh->nodes(temp_node);
        tet_cell.resize(4, temp_cell.size() / 4);
        tet_nodes.resize(3, temp_node.size() / 3);
        std::copy(temp_cell.begin(), temp_cell.end(), tet_cell.begin());
        std::copy(temp_node.begin(), temp_node.end(), tet_nodes.begin());
        {
            std::ofstream os(__POJ_BASE_PATH "result/test_volume/test_format_transition.vtk");
            tet2vtk(os, &tet_nodes[0], tet_nodes.size(2), &tet_cell[0], tet_cell.size(2));
        }
        cout << "[INFO]transition done!\n";
    //************************************************************************************************


    //*****************extract initial embed mesh and build interpolation matrix B*****************************************
        matrix<size_t> shell_cell;
        matrix<double> shell_nodes;
        matrix<double> shell_normal;
        gen_outside_shell(tet_cell, tet_nodes, shell_cell, shell_nodes, shell_normal, __SUBDIVISION_TIME, __EMBED_DEPTH);
        {
            std::ofstream os(__POJ_BASE_PATH "result/test_volume/tet_extract_init_embed.vtk");

            tri2vtk(os, &shell_nodes[0], shell_nodes.size(2), &shell_cell[0], shell_cell.size(2));
        }
        size_t row = tet_nodes.size(2);
        size_t col = shell_nodes.size(2);
        hj::sparse::spm_csc<double> B(row, col);
        tet_embed(tet_nodes, tet_cell, shell_nodes, B);
        matrix<double> B_(row, col);
        for (size_t j = 0; j < col; ++j)
            for (size_t i = 0; i < row; ++i)
                B_(i, j) = B(i, j);
        cout << "[INFO]generate initial embedded mesh done!\n";
    //************************************************************************************************

        cout << "[INFO]shell face size : " << shell_cell.size(2) << endl;
        cout << "[INFO]shell nodes size : " << shell_nodes.size(2) << endl;



    //***********init simulator and set fixed point*************************************************
    pSimulator simulator = pSimulator(new FullStVKSimulator());
    {
        succ = simulator->init(ini_file);
        assert(succ);
        simulator->setVolMesh(tet_mesh);
        simulator->precompute();
    }

//    {
//        vector<int> nodes{0,1,2,3,4,5,6,7,98,104,112,142,143,144,145,146,147,150,152,153,568,569,570,571,572,573,355,356,
//                          357,358,359,360,366,367,368,369,390,266,271,284,285,288,292,296,297,298,310,312,313,314,320,330,331,335,339,341,
//                         391,392,393,399,400,401,402,421,423,424,426,434,440,461,462,465,476,477,485,493,503,506,508,525,526,
//                          170,172,180,181,202,203,211,212,213,222,230,232,234,236,237,238,239,241,243,265};
////        for (int i = 42; i <= 49; ++i)
////            nodes.push_back(i);
////        for (int i = 23; i <= 30; ++i)
////            nodes.push_back(i);
////        for (int i = 79; i <= 88; ++i)
////            nodes.push_back(i);
//        cout << "num of fixed nodes: " << nodes.size() << endl;
//        simulator->setConNodes(nodes);
//        VectorXd uc(nodes.size()*3);
//        uc.setZero();
//        simulator->setUc(uc);
//    }         //for beam

    {
        vector<int> nodes{15,70,37,24,25,65,52,53,114,82,83,102,597,596,622
                         ,573,602,603,645,646,654,655,647};
        cout << "num of fixed nodes: " << nodes.size() << endl;
        simulator->setConNodes(nodes);
        VectorXd uc(nodes.size()*3);
        uc.setZero();
        simulator->setUc(uc);
    }        //for sofa

//    {
//        vector<int> nodes{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,
//                         42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65};
//        cout << "num of fixed nodes: " << nodes.size() << endl;
//        simulator->setConNodes(nodes);
//        VectorXd uc(nodes.size()*3);
//        uc.setZero();
//        simulator->setUc(uc);
//    }        //for cylinder

//    {
//        vector<int> nodes;
//        for (int i = 0; i <= 20; ++i)
//            nodes.push_back(i);
//        for (int i = 42; i <= 65; ++i)
//            nodes.push_back(i);
//        cout << "num of fixed nodes: " << nodes.size() << endl;
//        simulator->setConNodes(nodes);
//        VectorXd uc(nodes.size()*3);
//        uc.setZero();
//        simulator->setUc(uc);
//    }       //for new cylinder

//    {
//        vector<int> nodes;
//        for (int i = 0; i <= 31; ++i)
//            nodes.push_back(i);
//        for (int i = 64; i <= 139; ++i)
//            nodes.push_back(i);
//        cout << "num of fixed nodes: " << nodes.size() << endl;
//        simulator->setConNodes(nodes);
//        VectorXd uc(nodes.size()*3);
//        uc.setZero();
//        simulator->setUc(uc);
//    }         //for 16k face cylinder


    //**********************************************************************************************

    cout << "[INFO]the program is parting the embedded shell\n";
    cluster_machine handle(shell_cell, shell_nodes, __CLUSTER_RADIUS);
    handle.partition(__REGION_COUNT);



//    deformer shell_deformer(shell_cell, shell_nodes, shell_nodes, handle.regions_);
    deformer shell_deformer(shell_cell, shell_nodes, shell_nodes, handle.regions_);


    matrix<double> dx(tet_nodes.size(1), tet_nodes.size(2)),
                   q(tet_nodes.size(1), tet_nodes.size(2)),
                   xq(shell_nodes.size(1), shell_nodes.size(2));
    // Timer timer;
    {
//        for (int i = 35; i <= 39; ++i) {
//            const double f[3] = {0, 0, -1000};
//            simulator->setExtForceOfNode(i, f);
//        }  //force for new beam

        //single force on sofa
        const double force = -15000;
        const int nodeId0 = 474;
        const double f0[3] = {0, 0, force};
        simulator->setExtForceOfNode(nodeId0, f0);
        const int nodeId1 = 470;
        const double f1[3] = {0, 0, force};
        simulator->setExtForceOfNode(nodeId1, f1);
        const int nodeId2 = 465;
        const double f2[3] = {0, 0, force};
        simulator->setExtForceOfNode(nodeId2, f2);
        const int nodeId3 = 468;
        const double f3[3] = {0, 0, force};
        simulator->setExtForceOfNode(nodeId3, f3);
        const int nodeId4 = 473;
        const double f4[3] = {0, 0, force};
        simulator->setExtForceOfNode(nodeId4, f4);
        const int nodeId5 = 469;
        const double f5[3] = {0, 0, force};
        simulator->setExtForceOfNode(nodeId5, f5);
        const int nodeId6 = 472;
        const double f6[3] = {0, 0, force - 5000};
        simulator->setExtForceOfNode(nodeId6, f6);


//        //mutilple force on sofa
//          double force = -10000;
//       {
//            const int nodeId0 = 428;
//            const double f0[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId0, f0);
//            const int nodeId1 = 448;
//            const double f1[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId1, f1);
//            const int nodeId2 = 418;
//            const double f2[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId2, f2);
//            const int nodeId3 = 419;
//            const double f3[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId3, f3);
//            const int nodeId4 = 398;
//            const double f4[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId4, f4);
//            const int nodeId5 = 399;
//            const double f5[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId5, f5);
//            const int nodeId6 = 427;
//            const double f6[3] = {0, 0, force - 5000};
//            simulator->setExtForceOfNode(nodeId6, f6);
//          }
//        {
//            const int nodeId0 = 424;
//            const double f0[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId0, f0);
//            const int nodeId1 = 425;
//            const double f1[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId1, f1);
//            const int nodeId2 = 433;
//            const double f2[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId2, f2);
//            const int nodeId3 = 406;
//            const double f3[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId3, f3);
//            const int nodeId4 = 452;
//            const double f4[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId4, f4);
//            const int nodeId5 = 446;
//            const double f5[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId5, f5);
//            const int nodeId6 = 453;
//            const double f6[3] = {0, 0, force - 5000};
//            simulator->setExtForceOfNode(nodeId6, f6);
//        }

//        {
//            const int nodeId0 = 437;
//            const double f0[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId0, f0);
//            const int nodeId1 = 410;
//            const double f1[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId1, f1);
//            const int nodeId2 = 379;
//            const double f2[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId2, f2);
//            const int nodeId3 = 380;
//            const double f3[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId3, f3);
//            const int nodeId4 = 412;
//            const double f4[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId4, f4);
//            const int nodeId5 = 438;
//            const double f5[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId5, f5);
//            const int nodeId6 = 411;
//            const double f6[3] = {0, 0, force - 5000};
//            simulator->setExtForceOfNode(nodeId6, f6);
//        }
//        {

//            const int nodeId0 = 426;
//            const double f0[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId0, f0);
//            const int nodeId1 = 390;
//            const double f1[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId1, f1);
//            const int nodeId2 = 356;
//            const double f2[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId2, f2);
//            const int nodeId3 = 357;
//            const double f3[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId3, f3);
//            const int nodeId4 = 392;
//            const double f4[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId4, f4);
//            const int nodeId5 = 420;
//            const double f5[3] = {0, 0, force};
//            simulator->setExtForceOfNode(nodeId5, f5);
//            const int nodeId6 = 391;
//            const double f6[3] = {0, 0, force - 5000};
//            simulator->setExtForceOfNode(nodeId6, f6);
//        }




//cylinder push one face

//        for (int id = 68; id <= 89; ++id) {
//            const double f[3] = {0, 0, -6000};
//            simulator->setExtForceOfNode(id, f);
//        }


//new cylinder push one face
//        for (int i = 66; i <= 89; ++i) {
//            const double f[3] = {0, 0, -5000};
//            simulator->setExtForceOfNode(i, f);
//        }

//        //16k face cylinder push one face
//        for (int i = 140; i <= 215; ++i) {
//            const double f[3] = {0, 0, -1000};
//            simulator->setExtForceOfNode(i, f);
//        }


//        const double force = 1500;
//        const double f[3] = {0, 0, force};
//        for (int i = 105; i <= 111; ++i)
//            simulator->setExtForceOfNode(i, f);
//        //for beam


        VectorXd disp;
        for (int i = 0; i < 50; ++i)
        {
            cout << "\n\nstep " << i << endl;

            disp = simulator->getFullDisp();

            std::copy(disp.data(), disp.data() + disp.size(), dx.begin());
            q = tet_nodes + dx;
            xq = q * B_;

            if ( i == 30 )
                simulator->clearExtForce();

//            if ( i == 10 )
//                simulator->clearExtForce();

//            if ( i >= 20 && i < 80)
//            {
//                const double force = 2500;
//                matrix<double> dir = q(colon(), 21) - q(colon(), 41);
//                dir /= norm(dir);
//                const double f[3] = {force*dir(0, 0), force*dir(1, 0), force*dir(2, 0)};
//                simulator->setExtForceOfNode(21, f);
//                for (int id = 22; id <= 41; ++id) {
//                    matrix<double> dir = q(colon(), id) - q(colon(), id - 1);
//                    dir /= norm(dir);
//                    const double f[3] = {force*dir(0, 0), force*dir(1, 0), force*dir(2, 0)};
//                    simulator->setExtForceOfNode(id, f);
//                }
//            }

//            if ( i >= 80 )
//                simulator->clearExtForce();

//            if ( i >= 30 && i <= 80 ) {
//                const double force = 1000;
//                matrix<double> dir = q(colon(), 32) - q(colon(), 63);
//                dir /= norm(dir);
//                const double f[3] = {force*dir(0, 0), force*dir(1, 0), force*dir(2, 0)};
//                simulator->setExtForceOfNode(32, f);

//                for (int id = 33; id <= 63; ++id) {
//                    matrix<double> dir = q(colon(), id) - q(colon(), id - 1);
//                    dir /= norm(dir);
//                    const double f[3] = {force*dir(0, 0), force*dir(1, 0), force*dir(2, 0)};
//                    simulator->setExtForceOfNode(id, f);
//                }
//            }

//            if ( i == 80 )
//                simulator->clearExtForce();



            {
                //check the embedded mesh
                char outfile[100];
                sprintf(outfile, __POJ_BASE_PATH "result/test_volume/elastic_solid_%d.vtk", i);
                std::ofstream os(outfile);
                tet2vtk(os, &q[0], q.size(2), &tet_cell[0], tet_cell.size(2));
            }
            {   //check the embedded mesh
                char outfile[100];

                sprintf(outfile, __POJ_BASE_PATH "result/test_volume/emb_mesh_image_%d.vtk", i);

                std::ofstream os(outfile);
                tri2vtk(os, &xq[0], xq.size(2), &shell_cell[0], shell_cell.size(2));
            }
            {
                char outfile[100];
                sprintf(outfile, __POJ_BASE_PATH "result/after_deformation_%d.vtk", i);
                std::ofstream os(outfile);
                tri2vtk(os, &shell_nodes[0], shell_nodes.size(2), &shell_cell[0], shell_cell.size(2));
            }
//            {
//                char outfile[100];
//                sprintf(outfile, "/home/chenjiong/Desktop/beam_shell_mesh/shell_mesh_%d.obj", i);
//                jtf::mesh::save_obj(outfile, shell_cell, shell_nodes);
//            }
//            {
//                char outfile[100];
//                sprintf(outfile, "/home/chenjiong/Desktop/beam_embedded_mesh/embd_mesh_%d.obj", i);
//                jtf::mesh::save_obj(outfile, shell_cell, xq);
//            }

            hj::util::high_resolution_clock hrc;
            double begin = hrc.ms();
            simulator->forward();
            cout << "elastic deformation : " << hrc.ms() - begin << endl;
            shell_deformer.deform(shell_nodes, xq);

        }
    }

    // timer.stop("total simulation time for 200 steps is (seconds): ");
    cout << "[INFO]all done!\n";
    return 0;
}

#endif
