#include <stdio.h>
#include "mpi.h"

#define ASIZE	100
#define PI	3.141592653589793238462643

int main(int argc, char **argv)
{
    int me, nprocs, namelen;
        char processor_name[MPI_MAX_PROCESSOR_NAME];
        int val[ASIZE], sval[ASIZE], rval[ASIZE], i, j, flag;
        MPI_Status status;
        MPI_Request request;

        MPI_Init(&argc, &argv);
        MPI_Comm_size(MPI_COMM_WORLD, &nprocs);
        MPI_Comm_rank(MPI_COMM_WORLD, &me);
        MPI_Get_processor_name(processor_name, &namelen);

        printf("I'm process %d of %d\n", me, nprocs);

        /* Initialize: stuff some bogus values in an array */
        for (j = 0; j < ASIZE; j++)
            val[j] = sval[j] = j * me;

        for (i = 0; i < nprocs; i++) {
            /* Post up receive for values from neighboring process */
            MPI_Irecv(rval, ASIZE, MPI_INT, MPI_ANY_SOURCE, i,
                MPI_COMM_WORLD, &request);
            /* Send values to neighboring process */
            MPI_Send(sval, ASIZE, MPI_INT, (me < (nprocs-1) ? (me+1) : 0),
                i, MPI_COMM_WORLD);
            /* Wait until the the receive request has completed */
            MPI_Wait(&request, &status);
            for (j = 0; j < ASIZE; j++)
                sval[j] = rval[j];
        }

        for (j = flag = 0; j < ASIZE; j++)
            if (rval[j] != val[j]) flag++;

        if (flag)
            printf("%d: %d values were different!\n", me, flag);
        else
            printf("%d: No values were changed.\n", me);

        MPI_Finalize();
        return 0;
}


//int main(int argc, char *argv[])
//{
//    char *ProcessorName;
//    int ProcessorNameLen;
//    int MpiVersion, MpiSubversion;

//    MPI_Init( &argc, &argv );

//    int result = MPI_Get_processor_name( ProcessorName, &ProcessorNameLen );
//    printf( "result = %d, ProcessorName = %s, ProcessorNameLen = %d\n", result, ProcessorName, ProcessorNameLen );

//    result = MPI_Get_version( &MpiVersion, &MpiSubversion );
//    printf( "result = %d, MpiVersion = %d, MpiSubversion = %d\n", result, MpiVersion, MpiSubversion );

//    MPI_Finalize();
//    return 0;

//    MPI_Init( &argc, &argv );
//    printf( "Hello, world!\n" );
//    MPI_Finalize();
//    return 0;

//}


//#include <stdio.h>
//#include "mpi.h"
//int main(int argc, char **argv)
//{
//        int me, nprocs, namelen;
//        char processor_name[MPI_MAX_PROCESSOR_NAME];

//        MPI_Init(&argc, &argv);
//        MPI_Comm_size(MPI_COMM_WORLD, &nprocs);
//        MPI_Comm_rank(MPI_COMM_WORLD, &me);
//        MPI_Get_processor_name(processor_name, &namelen);

//    printf("Hello World!  I'm process %d of %d on %s\n", me, nprocs,
//        processor_name);

//        MPI_Finalize();
//        return 0;
//}


//#define ASIZE	100
//#define PI	3.141592653589793238462643

//int main(int argc, char **argv)
//{
//    int me, nprocs, namelen;
//    char processor_name[MPI_MAX_PROCESSOR_NAME];
//    int i;
//    double seed, init_val[ASIZE], val[ASIZE], sum, tsum;
//    MPI_Status status;

//    MPI_Init(&argc, &argv);
//    MPI_Comm_size(MPI_COMM_WORLD, &nprocs);
//    MPI_Comm_rank(MPI_COMM_WORLD, &me);
//    MPI_Get_processor_name(processor_name, &namelen);

//    if (!me) {	/* Only the first process in the group */
//        printf("Enter some kind of seed value:\n");
//        scanf("%lf", &seed);
//        for (i = 0; i < ASIZE; i++)
//            init_val[i] = (double)i * seed * PI;
//    }

//    /* Broadcast computed initial values to all other processes */
//    if (MPI_Bcast(init_val, ASIZE, MPI_DOUBLE, 0, MPI_COMM_WORLD)
//        != MPI_SUCCESS)
//        fprintf(stderr, "Oops! An error occurred in MPI_Bcast()\n");

//    for (i = 0, sum = 0.0; i < ASIZE; i++) {
//        val[i] = init_val[i] * me;
//        sum += val[i];
//    }
//    printf("%d: My sum is %lf\n", me, sum);

//    /* Add the value of sum from all processes and store the total in
//       tsum on process 0. */
//    MPI_Reduce(&sum, &tsum, 1, MPI_DOUBLE, MPI_SUM, 0, MPI_COMM_WORLD);

//    if (!me) printf("%d: Total sum is %lf\n", me, tsum);

//    MPI_Finalize();
//    return 0;
//}


//#include <stdio.h>
//#include <mpich2/mpi.h>


//#define ASIZE	100
//#define PI	3.141592653589793238462643

//int main(int argc, char **argv)
//{
//    int me, nprocs, namelen;
//    char processor_name[MPI_MAX_PROCESSOR_NAME];
//    int i;
//    double seed, init_val[ASIZE], val[ASIZE], sum, tsum;
//    MPI_Status status;

//    MPI_Init(&argc, &argv);
//    MPI_Comm_size(MPI_COMM_WORLD, &nprocs);
//        return 0;
//    MPI_Comm_rank(MPI_COMM_WORLD, &me);
//    return 0;
//    MPI_Get_processor_name(processor_name, &namelen);
//    return 0;
//    if (!me) {	/* Only the first process in the group */
//        printf("Enter some kind of seed value:\n");
//        scanf("%lf", &seed);
//        for (i = 0; i < ASIZE; i++)
//            init_val[i] = (double)i * seed * PI;
//    }

//    return 0;
//    /* Broadcast computed initial values to all other processes */
//    if (MPI_Bcast(init_val, ASIZE, MPI_DOUBLE, 0, MPI_COMM_WORLD)
//        != MPI_SUCCESS)
//        fprintf(stderr, "Oops! An error occurred in MPI_Bcast()\n");

//    for (i = 0, sum = 0.0; i < ASIZE; i++) {
//        val[i] = init_val[i] * me;
//        sum += val[i];
//    }
//    printf("%d: My sum is %lf\n", me, sum);

//    /* Send sum back to the first process */
//    if (me)	{	/* All processes except the one of rank 0 */
//        MPI_Send(&sum, 1, MPI_DOUBLE, 0, 1, MPI_COMM_WORLD);
//    }
//    else {
//        tsum = sum;
//        for (i = 1; i < nprocs; i++) {
//            MPI_Recv(&sum, 1, MPI_DOUBLE, MPI_ANY_SOURCE, 1,
//                MPI_COMM_WORLD, &status);
//            tsum += sum;
//        }
//        printf("%d: Total sum is %lf\n", me, tsum);
//    }

//    MPI_Finalize();
//}


//#include <stdio.h>
//#include <stdlib.h>
//#include <omp.h>
//#include <mpich2/mpi.h>

//int main(int argc, char* argv[])
//{
//int nprocs = 1; //the number of processes
//int myrank = 0;
//int provide;

//MPI_Init_thread(&argc,&argv,MPI_THREAD_FUNNELED,&provide);
//if (MPI_THREAD_FUNNELED != provide)
//{
//    printf ("%d != required %d", MPI_THREAD_FUNNELED, provide);
//    return 0;
//}

//MPI_Comm_size(MPI_COMM_WORLD,&nprocs);
//MPI_Comm_rank(MPI_COMM_WORLD,&myrank);

//int num_threads = 1;      //Openmp
//omp_set_dynamic(1);
//num_threads = 16;
//omp_set_num_threads(num_threads);

//#pragma omp parallel
//{
//    printf ("%d omp thread from %d mpi process\n", omp_get_thread_num(), myrank);

//}
//MPI_Finalize();

//}
