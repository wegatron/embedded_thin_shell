#ifndef __CJ_CONF_PARAMETER_H__
#define __CJ_CONF_PARAMETER_H__

//  #define  __Ex                40*1000
//  #define  __Eq                0.5*1000
//  #define  __H                 0.001
//  #define  __SIGMA             (3.0 * __H * pow((__Ex/3.0/__Eq),1.0/3))
//  #define  __GAMMA             (__H * __Ex)
//  #define  __CLUSTER_RADIUS    0.01
//  #define  __SUBDIVISION_TIME  2
//  #define  __EMBED_DEPTH       0.003
//  #define  __REGION_COUNT      50
//  #define  __NODES_NUMBER      8

#define  __Ex                40*1000
#define  __Eq                0.5*1000
#define  __H                 0.0007
#define  __SIGMA             (2.0 * __H * pow((__Ex/3.0/__Eq),1.0/3))
#define  __GAMMA             (__H * __Ex)
#define  __CLUSTER_RADIUS    0.01
#define  __SUBDIVISION_TIME  3
#define  __EMBED_DEPTH       0.005
#define  __REGION_COUNT      200
#define  __NODES_NUMBER      1

#endif
