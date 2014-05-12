#include <iostream>
#include <omp.h>
#include <zjucad/matrix/matrix.h>
#include <hjlib/math/blas_lapack.h>
#include <zjucad/matrix/lapack.h>
#include <boost/property_tree/ptree.hpp>

using namespace std;

int main()
{
    boost::property_tree::ptree opts;
    const string solver_type1 = opts.get<string>("linear_solver/type.value", "PETsc");
    opts.put("linear_solver/type.value", "cholmod");
    const string solver_type2 = opts.get<string>("linear_solver/type.value", "umfpack");
    cout << solver_type1 << endl;
    cout << solver_type2 << endl;


    return 0;
    zjucad::matrix::matrix<double> a;
    a = zjucad::matrix::eye<double>(3);
    inv(a);
    int core_num = omp_get_num_procs();
    cout << core_num << endl;
#pragma omp parallel for
    for (size_t i = 0; i < 9; ++i)
        printf("%d\n", i);
    return 0;
    printf("hello world!\n");
    return 0;

}

//#define Fabs(x, y) \
//do {                         \
//    if ( x > y)              \
//        printf("%d\n", x);   \
//    else                     \
//        printf("%d\n", y);   \
//}while(0)




//int main()
//{
//    Fabs(3, 5);
//    return 0;
//}
