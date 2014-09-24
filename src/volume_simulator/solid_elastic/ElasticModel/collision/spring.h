#ifndef __CJ_SPHERE_SPRING_H__
#define __CJ_SPHERE_SPRING_H__

extern "C" {

void spring_energy_(double *val, const double *x, const double *c, const double *r);

void spring_energy_jac_(double *jac, const double *x, const double *c, const double *r);

void spring_energy_hes_(double *hes, const double *x, const double *c, const double *r);

}

#endif
