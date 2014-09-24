#include <time.h>
#include <stdlib.h>

#include<iostream>
#include <Eigen/Dense>

#include "assertext.h"
#include "collision_ball_energy.h"
#include "spring.h"

using namespace std;

int main(int argc, char *argv[])
{
  double v[3] = {1, 0, 1};
  double c[3] = {0, 0, 0};

  double r = 1.0;

  double energy_val1;
  double energy_val2;
  Eigen::Vector3d jac1;
  Eigen::Vector3d jac2;
  Eigen::Matrix<double, 9, 1> hes1;
  Eigen::Matrix<double, 9, 1> hes2;

  srand(time(NULL));
  for (int i=0; i<1000; ++i) {
    v[0] = rand()%100/10.0;
    v[1] = rand()%100/10.0;
    v[2] = rand()%100/10.0;

    c[0] = rand()%100/10.0;
    c[1] = rand()%100/10.0;
    c[2] = rand()%100/10.0;

    collision_ball_energy_(&energy_val1, v, c, &r);
    collision_ball_energy_jac_(&jac1[0], v, c , &r);
    collision_ball_energy_hes_(&hes1[0], v, c, &r);

    spring_energy_(&energy_val2, v, c, &r);
    spring_energy_jac_(&jac2[0], v, c, &r);
    spring_energy_hes_(&hes2[0], v, c, &r);

    assert_eq(energy_val1, energy_val2);
    assert_eq(jac1, jac2);
    assert_eq(hes1, hes2);
  }
  return 0;
}
