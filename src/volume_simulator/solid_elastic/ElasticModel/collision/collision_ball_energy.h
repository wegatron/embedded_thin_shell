#ifndef COLLISION_BALL_ENERGY_H
#define COLLISION_BALL_ENERGY_H

extern "C" {
  void collision_ball_energy_(double *val, const double *v, const double *c, const double *r);

  void collision_ball_energy_jac_(double *jac, const double *v, const double *c, const double *r);

  void collision_ball_energy_hes_(double *hes, const double *v, const double *c, const double *r);
}

#endif /* COLLISION_BALL_ENERGY_H */
