#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3728982769992195016) {
   out_3728982769992195016[0] = delta_x[0] + nom_x[0];
   out_3728982769992195016[1] = delta_x[1] + nom_x[1];
   out_3728982769992195016[2] = delta_x[2] + nom_x[2];
   out_3728982769992195016[3] = delta_x[3] + nom_x[3];
   out_3728982769992195016[4] = delta_x[4] + nom_x[4];
   out_3728982769992195016[5] = delta_x[5] + nom_x[5];
   out_3728982769992195016[6] = delta_x[6] + nom_x[6];
   out_3728982769992195016[7] = delta_x[7] + nom_x[7];
   out_3728982769992195016[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7936685168515433141) {
   out_7936685168515433141[0] = -nom_x[0] + true_x[0];
   out_7936685168515433141[1] = -nom_x[1] + true_x[1];
   out_7936685168515433141[2] = -nom_x[2] + true_x[2];
   out_7936685168515433141[3] = -nom_x[3] + true_x[3];
   out_7936685168515433141[4] = -nom_x[4] + true_x[4];
   out_7936685168515433141[5] = -nom_x[5] + true_x[5];
   out_7936685168515433141[6] = -nom_x[6] + true_x[6];
   out_7936685168515433141[7] = -nom_x[7] + true_x[7];
   out_7936685168515433141[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7513730895619064577) {
   out_7513730895619064577[0] = 1.0;
   out_7513730895619064577[1] = 0.0;
   out_7513730895619064577[2] = 0.0;
   out_7513730895619064577[3] = 0.0;
   out_7513730895619064577[4] = 0.0;
   out_7513730895619064577[5] = 0.0;
   out_7513730895619064577[6] = 0.0;
   out_7513730895619064577[7] = 0.0;
   out_7513730895619064577[8] = 0.0;
   out_7513730895619064577[9] = 0.0;
   out_7513730895619064577[10] = 1.0;
   out_7513730895619064577[11] = 0.0;
   out_7513730895619064577[12] = 0.0;
   out_7513730895619064577[13] = 0.0;
   out_7513730895619064577[14] = 0.0;
   out_7513730895619064577[15] = 0.0;
   out_7513730895619064577[16] = 0.0;
   out_7513730895619064577[17] = 0.0;
   out_7513730895619064577[18] = 0.0;
   out_7513730895619064577[19] = 0.0;
   out_7513730895619064577[20] = 1.0;
   out_7513730895619064577[21] = 0.0;
   out_7513730895619064577[22] = 0.0;
   out_7513730895619064577[23] = 0.0;
   out_7513730895619064577[24] = 0.0;
   out_7513730895619064577[25] = 0.0;
   out_7513730895619064577[26] = 0.0;
   out_7513730895619064577[27] = 0.0;
   out_7513730895619064577[28] = 0.0;
   out_7513730895619064577[29] = 0.0;
   out_7513730895619064577[30] = 1.0;
   out_7513730895619064577[31] = 0.0;
   out_7513730895619064577[32] = 0.0;
   out_7513730895619064577[33] = 0.0;
   out_7513730895619064577[34] = 0.0;
   out_7513730895619064577[35] = 0.0;
   out_7513730895619064577[36] = 0.0;
   out_7513730895619064577[37] = 0.0;
   out_7513730895619064577[38] = 0.0;
   out_7513730895619064577[39] = 0.0;
   out_7513730895619064577[40] = 1.0;
   out_7513730895619064577[41] = 0.0;
   out_7513730895619064577[42] = 0.0;
   out_7513730895619064577[43] = 0.0;
   out_7513730895619064577[44] = 0.0;
   out_7513730895619064577[45] = 0.0;
   out_7513730895619064577[46] = 0.0;
   out_7513730895619064577[47] = 0.0;
   out_7513730895619064577[48] = 0.0;
   out_7513730895619064577[49] = 0.0;
   out_7513730895619064577[50] = 1.0;
   out_7513730895619064577[51] = 0.0;
   out_7513730895619064577[52] = 0.0;
   out_7513730895619064577[53] = 0.0;
   out_7513730895619064577[54] = 0.0;
   out_7513730895619064577[55] = 0.0;
   out_7513730895619064577[56] = 0.0;
   out_7513730895619064577[57] = 0.0;
   out_7513730895619064577[58] = 0.0;
   out_7513730895619064577[59] = 0.0;
   out_7513730895619064577[60] = 1.0;
   out_7513730895619064577[61] = 0.0;
   out_7513730895619064577[62] = 0.0;
   out_7513730895619064577[63] = 0.0;
   out_7513730895619064577[64] = 0.0;
   out_7513730895619064577[65] = 0.0;
   out_7513730895619064577[66] = 0.0;
   out_7513730895619064577[67] = 0.0;
   out_7513730895619064577[68] = 0.0;
   out_7513730895619064577[69] = 0.0;
   out_7513730895619064577[70] = 1.0;
   out_7513730895619064577[71] = 0.0;
   out_7513730895619064577[72] = 0.0;
   out_7513730895619064577[73] = 0.0;
   out_7513730895619064577[74] = 0.0;
   out_7513730895619064577[75] = 0.0;
   out_7513730895619064577[76] = 0.0;
   out_7513730895619064577[77] = 0.0;
   out_7513730895619064577[78] = 0.0;
   out_7513730895619064577[79] = 0.0;
   out_7513730895619064577[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2023060963930489365) {
   out_2023060963930489365[0] = state[0];
   out_2023060963930489365[1] = state[1];
   out_2023060963930489365[2] = state[2];
   out_2023060963930489365[3] = state[3];
   out_2023060963930489365[4] = state[4];
   out_2023060963930489365[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2023060963930489365[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2023060963930489365[7] = state[7];
   out_2023060963930489365[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5000204650175358919) {
   out_5000204650175358919[0] = 1;
   out_5000204650175358919[1] = 0;
   out_5000204650175358919[2] = 0;
   out_5000204650175358919[3] = 0;
   out_5000204650175358919[4] = 0;
   out_5000204650175358919[5] = 0;
   out_5000204650175358919[6] = 0;
   out_5000204650175358919[7] = 0;
   out_5000204650175358919[8] = 0;
   out_5000204650175358919[9] = 0;
   out_5000204650175358919[10] = 1;
   out_5000204650175358919[11] = 0;
   out_5000204650175358919[12] = 0;
   out_5000204650175358919[13] = 0;
   out_5000204650175358919[14] = 0;
   out_5000204650175358919[15] = 0;
   out_5000204650175358919[16] = 0;
   out_5000204650175358919[17] = 0;
   out_5000204650175358919[18] = 0;
   out_5000204650175358919[19] = 0;
   out_5000204650175358919[20] = 1;
   out_5000204650175358919[21] = 0;
   out_5000204650175358919[22] = 0;
   out_5000204650175358919[23] = 0;
   out_5000204650175358919[24] = 0;
   out_5000204650175358919[25] = 0;
   out_5000204650175358919[26] = 0;
   out_5000204650175358919[27] = 0;
   out_5000204650175358919[28] = 0;
   out_5000204650175358919[29] = 0;
   out_5000204650175358919[30] = 1;
   out_5000204650175358919[31] = 0;
   out_5000204650175358919[32] = 0;
   out_5000204650175358919[33] = 0;
   out_5000204650175358919[34] = 0;
   out_5000204650175358919[35] = 0;
   out_5000204650175358919[36] = 0;
   out_5000204650175358919[37] = 0;
   out_5000204650175358919[38] = 0;
   out_5000204650175358919[39] = 0;
   out_5000204650175358919[40] = 1;
   out_5000204650175358919[41] = 0;
   out_5000204650175358919[42] = 0;
   out_5000204650175358919[43] = 0;
   out_5000204650175358919[44] = 0;
   out_5000204650175358919[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5000204650175358919[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5000204650175358919[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5000204650175358919[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5000204650175358919[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5000204650175358919[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5000204650175358919[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5000204650175358919[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5000204650175358919[53] = -9.8100000000000005*dt;
   out_5000204650175358919[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5000204650175358919[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5000204650175358919[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5000204650175358919[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5000204650175358919[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5000204650175358919[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5000204650175358919[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5000204650175358919[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5000204650175358919[62] = 0;
   out_5000204650175358919[63] = 0;
   out_5000204650175358919[64] = 0;
   out_5000204650175358919[65] = 0;
   out_5000204650175358919[66] = 0;
   out_5000204650175358919[67] = 0;
   out_5000204650175358919[68] = 0;
   out_5000204650175358919[69] = 0;
   out_5000204650175358919[70] = 1;
   out_5000204650175358919[71] = 0;
   out_5000204650175358919[72] = 0;
   out_5000204650175358919[73] = 0;
   out_5000204650175358919[74] = 0;
   out_5000204650175358919[75] = 0;
   out_5000204650175358919[76] = 0;
   out_5000204650175358919[77] = 0;
   out_5000204650175358919[78] = 0;
   out_5000204650175358919[79] = 0;
   out_5000204650175358919[80] = 1;
}
void h_25(double *state, double *unused, double *out_6775712609208471806) {
   out_6775712609208471806[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2610039070434399737) {
   out_2610039070434399737[0] = 0;
   out_2610039070434399737[1] = 0;
   out_2610039070434399737[2] = 0;
   out_2610039070434399737[3] = 0;
   out_2610039070434399737[4] = 0;
   out_2610039070434399737[5] = 0;
   out_2610039070434399737[6] = 1;
   out_2610039070434399737[7] = 0;
   out_2610039070434399737[8] = 0;
}
void h_24(double *state, double *unused, double *out_6623912581949403498) {
   out_6623912581949403498[0] = state[4];
   out_6623912581949403498[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7478853935462106589) {
   out_7478853935462106589[0] = 0;
   out_7478853935462106589[1] = 0;
   out_7478853935462106589[2] = 0;
   out_7478853935462106589[3] = 0;
   out_7478853935462106589[4] = 1;
   out_7478853935462106589[5] = 0;
   out_7478853935462106589[6] = 0;
   out_7478853935462106589[7] = 0;
   out_7478853935462106589[8] = 0;
   out_7478853935462106589[9] = 0;
   out_7478853935462106589[10] = 0;
   out_7478853935462106589[11] = 0;
   out_7478853935462106589[12] = 0;
   out_7478853935462106589[13] = 0;
   out_7478853935462106589[14] = 1;
   out_7478853935462106589[15] = 0;
   out_7478853935462106589[16] = 0;
   out_7478853935462106589[17] = 0;
}
void h_30(double *state, double *unused, double *out_8723305474145594849) {
   out_8723305474145594849[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7137735400562007935) {
   out_7137735400562007935[0] = 0;
   out_7137735400562007935[1] = 0;
   out_7137735400562007935[2] = 0;
   out_7137735400562007935[3] = 0;
   out_7137735400562007935[4] = 1;
   out_7137735400562007935[5] = 0;
   out_7137735400562007935[6] = 0;
   out_7137735400562007935[7] = 0;
   out_7137735400562007935[8] = 0;
}
void h_26(double *state, double *unused, double *out_5686519422701262164) {
   out_5686519422701262164[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6351542389308455961) {
   out_6351542389308455961[0] = 0;
   out_6351542389308455961[1] = 0;
   out_6351542389308455961[2] = 0;
   out_6351542389308455961[3] = 0;
   out_6351542389308455961[4] = 0;
   out_6351542389308455961[5] = 0;
   out_6351542389308455961[6] = 0;
   out_6351542389308455961[7] = 1;
   out_6351542389308455961[8] = 0;
}
void h_27(double *state, double *unused, double *out_5832202756306809371) {
   out_5832202756306809371[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4914141329378064718) {
   out_4914141329378064718[0] = 0;
   out_4914141329378064718[1] = 0;
   out_4914141329378064718[2] = 0;
   out_4914141329378064718[3] = 1;
   out_4914141329378064718[4] = 0;
   out_4914141329378064718[5] = 0;
   out_4914141329378064718[6] = 0;
   out_4914141329378064718[7] = 0;
   out_4914141329378064718[8] = 0;
}
void h_29(double *state, double *unused, double *out_3721632206484749204) {
   out_3721632206484749204[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6627504056247615751) {
   out_6627504056247615751[0] = 0;
   out_6627504056247615751[1] = 1;
   out_6627504056247615751[2] = 0;
   out_6627504056247615751[3] = 0;
   out_6627504056247615751[4] = 0;
   out_6627504056247615751[5] = 0;
   out_6627504056247615751[6] = 0;
   out_6627504056247615751[7] = 0;
   out_6627504056247615751[8] = 0;
}
void h_28(double *state, double *unused, double *out_2105693166963939297) {
   out_2105693166963939297[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4663873784682289500) {
   out_4663873784682289500[0] = 1;
   out_4663873784682289500[1] = 0;
   out_4663873784682289500[2] = 0;
   out_4663873784682289500[3] = 0;
   out_4663873784682289500[4] = 0;
   out_4663873784682289500[5] = 0;
   out_4663873784682289500[6] = 0;
   out_4663873784682289500[7] = 0;
   out_4663873784682289500[8] = 0;
}
void h_31(double *state, double *unused, double *out_4900196238150728874) {
   out_4900196238150728874[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6977750491541807437) {
   out_6977750491541807437[0] = 0;
   out_6977750491541807437[1] = 0;
   out_6977750491541807437[2] = 0;
   out_6977750491541807437[3] = 0;
   out_6977750491541807437[4] = 0;
   out_6977750491541807437[5] = 0;
   out_6977750491541807437[6] = 0;
   out_6977750491541807437[7] = 0;
   out_6977750491541807437[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3728982769992195016) {
  err_fun(nom_x, delta_x, out_3728982769992195016);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7936685168515433141) {
  inv_err_fun(nom_x, true_x, out_7936685168515433141);
}
void car_H_mod_fun(double *state, double *out_7513730895619064577) {
  H_mod_fun(state, out_7513730895619064577);
}
void car_f_fun(double *state, double dt, double *out_2023060963930489365) {
  f_fun(state,  dt, out_2023060963930489365);
}
void car_F_fun(double *state, double dt, double *out_5000204650175358919) {
  F_fun(state,  dt, out_5000204650175358919);
}
void car_h_25(double *state, double *unused, double *out_6775712609208471806) {
  h_25(state, unused, out_6775712609208471806);
}
void car_H_25(double *state, double *unused, double *out_2610039070434399737) {
  H_25(state, unused, out_2610039070434399737);
}
void car_h_24(double *state, double *unused, double *out_6623912581949403498) {
  h_24(state, unused, out_6623912581949403498);
}
void car_H_24(double *state, double *unused, double *out_7478853935462106589) {
  H_24(state, unused, out_7478853935462106589);
}
void car_h_30(double *state, double *unused, double *out_8723305474145594849) {
  h_30(state, unused, out_8723305474145594849);
}
void car_H_30(double *state, double *unused, double *out_7137735400562007935) {
  H_30(state, unused, out_7137735400562007935);
}
void car_h_26(double *state, double *unused, double *out_5686519422701262164) {
  h_26(state, unused, out_5686519422701262164);
}
void car_H_26(double *state, double *unused, double *out_6351542389308455961) {
  H_26(state, unused, out_6351542389308455961);
}
void car_h_27(double *state, double *unused, double *out_5832202756306809371) {
  h_27(state, unused, out_5832202756306809371);
}
void car_H_27(double *state, double *unused, double *out_4914141329378064718) {
  H_27(state, unused, out_4914141329378064718);
}
void car_h_29(double *state, double *unused, double *out_3721632206484749204) {
  h_29(state, unused, out_3721632206484749204);
}
void car_H_29(double *state, double *unused, double *out_6627504056247615751) {
  H_29(state, unused, out_6627504056247615751);
}
void car_h_28(double *state, double *unused, double *out_2105693166963939297) {
  h_28(state, unused, out_2105693166963939297);
}
void car_H_28(double *state, double *unused, double *out_4663873784682289500) {
  H_28(state, unused, out_4663873784682289500);
}
void car_h_31(double *state, double *unused, double *out_4900196238150728874) {
  h_31(state, unused, out_4900196238150728874);
}
void car_H_31(double *state, double *unused, double *out_6977750491541807437) {
  H_31(state, unused, out_6977750491541807437);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
