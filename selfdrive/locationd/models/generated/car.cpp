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
void err_fun(double *nom_x, double *delta_x, double *out_6184164552224961757) {
   out_6184164552224961757[0] = delta_x[0] + nom_x[0];
   out_6184164552224961757[1] = delta_x[1] + nom_x[1];
   out_6184164552224961757[2] = delta_x[2] + nom_x[2];
   out_6184164552224961757[3] = delta_x[3] + nom_x[3];
   out_6184164552224961757[4] = delta_x[4] + nom_x[4];
   out_6184164552224961757[5] = delta_x[5] + nom_x[5];
   out_6184164552224961757[6] = delta_x[6] + nom_x[6];
   out_6184164552224961757[7] = delta_x[7] + nom_x[7];
   out_6184164552224961757[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1205517025722757514) {
   out_1205517025722757514[0] = -nom_x[0] + true_x[0];
   out_1205517025722757514[1] = -nom_x[1] + true_x[1];
   out_1205517025722757514[2] = -nom_x[2] + true_x[2];
   out_1205517025722757514[3] = -nom_x[3] + true_x[3];
   out_1205517025722757514[4] = -nom_x[4] + true_x[4];
   out_1205517025722757514[5] = -nom_x[5] + true_x[5];
   out_1205517025722757514[6] = -nom_x[6] + true_x[6];
   out_1205517025722757514[7] = -nom_x[7] + true_x[7];
   out_1205517025722757514[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8692073091461732284) {
   out_8692073091461732284[0] = 1.0;
   out_8692073091461732284[1] = 0.0;
   out_8692073091461732284[2] = 0.0;
   out_8692073091461732284[3] = 0.0;
   out_8692073091461732284[4] = 0.0;
   out_8692073091461732284[5] = 0.0;
   out_8692073091461732284[6] = 0.0;
   out_8692073091461732284[7] = 0.0;
   out_8692073091461732284[8] = 0.0;
   out_8692073091461732284[9] = 0.0;
   out_8692073091461732284[10] = 1.0;
   out_8692073091461732284[11] = 0.0;
   out_8692073091461732284[12] = 0.0;
   out_8692073091461732284[13] = 0.0;
   out_8692073091461732284[14] = 0.0;
   out_8692073091461732284[15] = 0.0;
   out_8692073091461732284[16] = 0.0;
   out_8692073091461732284[17] = 0.0;
   out_8692073091461732284[18] = 0.0;
   out_8692073091461732284[19] = 0.0;
   out_8692073091461732284[20] = 1.0;
   out_8692073091461732284[21] = 0.0;
   out_8692073091461732284[22] = 0.0;
   out_8692073091461732284[23] = 0.0;
   out_8692073091461732284[24] = 0.0;
   out_8692073091461732284[25] = 0.0;
   out_8692073091461732284[26] = 0.0;
   out_8692073091461732284[27] = 0.0;
   out_8692073091461732284[28] = 0.0;
   out_8692073091461732284[29] = 0.0;
   out_8692073091461732284[30] = 1.0;
   out_8692073091461732284[31] = 0.0;
   out_8692073091461732284[32] = 0.0;
   out_8692073091461732284[33] = 0.0;
   out_8692073091461732284[34] = 0.0;
   out_8692073091461732284[35] = 0.0;
   out_8692073091461732284[36] = 0.0;
   out_8692073091461732284[37] = 0.0;
   out_8692073091461732284[38] = 0.0;
   out_8692073091461732284[39] = 0.0;
   out_8692073091461732284[40] = 1.0;
   out_8692073091461732284[41] = 0.0;
   out_8692073091461732284[42] = 0.0;
   out_8692073091461732284[43] = 0.0;
   out_8692073091461732284[44] = 0.0;
   out_8692073091461732284[45] = 0.0;
   out_8692073091461732284[46] = 0.0;
   out_8692073091461732284[47] = 0.0;
   out_8692073091461732284[48] = 0.0;
   out_8692073091461732284[49] = 0.0;
   out_8692073091461732284[50] = 1.0;
   out_8692073091461732284[51] = 0.0;
   out_8692073091461732284[52] = 0.0;
   out_8692073091461732284[53] = 0.0;
   out_8692073091461732284[54] = 0.0;
   out_8692073091461732284[55] = 0.0;
   out_8692073091461732284[56] = 0.0;
   out_8692073091461732284[57] = 0.0;
   out_8692073091461732284[58] = 0.0;
   out_8692073091461732284[59] = 0.0;
   out_8692073091461732284[60] = 1.0;
   out_8692073091461732284[61] = 0.0;
   out_8692073091461732284[62] = 0.0;
   out_8692073091461732284[63] = 0.0;
   out_8692073091461732284[64] = 0.0;
   out_8692073091461732284[65] = 0.0;
   out_8692073091461732284[66] = 0.0;
   out_8692073091461732284[67] = 0.0;
   out_8692073091461732284[68] = 0.0;
   out_8692073091461732284[69] = 0.0;
   out_8692073091461732284[70] = 1.0;
   out_8692073091461732284[71] = 0.0;
   out_8692073091461732284[72] = 0.0;
   out_8692073091461732284[73] = 0.0;
   out_8692073091461732284[74] = 0.0;
   out_8692073091461732284[75] = 0.0;
   out_8692073091461732284[76] = 0.0;
   out_8692073091461732284[77] = 0.0;
   out_8692073091461732284[78] = 0.0;
   out_8692073091461732284[79] = 0.0;
   out_8692073091461732284[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6022347720067266341) {
   out_6022347720067266341[0] = state[0];
   out_6022347720067266341[1] = state[1];
   out_6022347720067266341[2] = state[2];
   out_6022347720067266341[3] = state[3];
   out_6022347720067266341[4] = state[4];
   out_6022347720067266341[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6022347720067266341[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6022347720067266341[7] = state[7];
   out_6022347720067266341[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2034100707598491156) {
   out_2034100707598491156[0] = 1;
   out_2034100707598491156[1] = 0;
   out_2034100707598491156[2] = 0;
   out_2034100707598491156[3] = 0;
   out_2034100707598491156[4] = 0;
   out_2034100707598491156[5] = 0;
   out_2034100707598491156[6] = 0;
   out_2034100707598491156[7] = 0;
   out_2034100707598491156[8] = 0;
   out_2034100707598491156[9] = 0;
   out_2034100707598491156[10] = 1;
   out_2034100707598491156[11] = 0;
   out_2034100707598491156[12] = 0;
   out_2034100707598491156[13] = 0;
   out_2034100707598491156[14] = 0;
   out_2034100707598491156[15] = 0;
   out_2034100707598491156[16] = 0;
   out_2034100707598491156[17] = 0;
   out_2034100707598491156[18] = 0;
   out_2034100707598491156[19] = 0;
   out_2034100707598491156[20] = 1;
   out_2034100707598491156[21] = 0;
   out_2034100707598491156[22] = 0;
   out_2034100707598491156[23] = 0;
   out_2034100707598491156[24] = 0;
   out_2034100707598491156[25] = 0;
   out_2034100707598491156[26] = 0;
   out_2034100707598491156[27] = 0;
   out_2034100707598491156[28] = 0;
   out_2034100707598491156[29] = 0;
   out_2034100707598491156[30] = 1;
   out_2034100707598491156[31] = 0;
   out_2034100707598491156[32] = 0;
   out_2034100707598491156[33] = 0;
   out_2034100707598491156[34] = 0;
   out_2034100707598491156[35] = 0;
   out_2034100707598491156[36] = 0;
   out_2034100707598491156[37] = 0;
   out_2034100707598491156[38] = 0;
   out_2034100707598491156[39] = 0;
   out_2034100707598491156[40] = 1;
   out_2034100707598491156[41] = 0;
   out_2034100707598491156[42] = 0;
   out_2034100707598491156[43] = 0;
   out_2034100707598491156[44] = 0;
   out_2034100707598491156[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2034100707598491156[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2034100707598491156[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2034100707598491156[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2034100707598491156[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2034100707598491156[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2034100707598491156[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2034100707598491156[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2034100707598491156[53] = -9.8100000000000005*dt;
   out_2034100707598491156[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2034100707598491156[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2034100707598491156[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2034100707598491156[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2034100707598491156[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2034100707598491156[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2034100707598491156[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2034100707598491156[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2034100707598491156[62] = 0;
   out_2034100707598491156[63] = 0;
   out_2034100707598491156[64] = 0;
   out_2034100707598491156[65] = 0;
   out_2034100707598491156[66] = 0;
   out_2034100707598491156[67] = 0;
   out_2034100707598491156[68] = 0;
   out_2034100707598491156[69] = 0;
   out_2034100707598491156[70] = 1;
   out_2034100707598491156[71] = 0;
   out_2034100707598491156[72] = 0;
   out_2034100707598491156[73] = 0;
   out_2034100707598491156[74] = 0;
   out_2034100707598491156[75] = 0;
   out_2034100707598491156[76] = 0;
   out_2034100707598491156[77] = 0;
   out_2034100707598491156[78] = 0;
   out_2034100707598491156[79] = 0;
   out_2034100707598491156[80] = 1;
}
void h_25(double *state, double *unused, double *out_1603510504705161482) {
   out_1603510504705161482[0] = state[6];
}
void H_25(double *state, double *unused, double *out_561482756335582095) {
   out_561482756335582095[0] = 0;
   out_561482756335582095[1] = 0;
   out_561482756335582095[2] = 0;
   out_561482756335582095[3] = 0;
   out_561482756335582095[4] = 0;
   out_561482756335582095[5] = 0;
   out_561482756335582095[6] = 1;
   out_561482756335582095[7] = 0;
   out_561482756335582095[8] = 0;
}
void h_24(double *state, double *unused, double *out_8697386051471671882) {
   out_8697386051471671882[0] = state[4];
   out_8697386051471671882[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1611166842669917471) {
   out_1611166842669917471[0] = 0;
   out_1611166842669917471[1] = 0;
   out_1611166842669917471[2] = 0;
   out_1611166842669917471[3] = 0;
   out_1611166842669917471[4] = 1;
   out_1611166842669917471[5] = 0;
   out_1611166842669917471[6] = 0;
   out_1611166842669917471[7] = 0;
   out_1611166842669917471[8] = 0;
   out_1611166842669917471[9] = 0;
   out_1611166842669917471[10] = 0;
   out_1611166842669917471[11] = 0;
   out_1611166842669917471[12] = 0;
   out_1611166842669917471[13] = 0;
   out_1611166842669917471[14] = 1;
   out_1611166842669917471[15] = 0;
   out_1611166842669917471[16] = 0;
   out_1611166842669917471[17] = 0;
}
void h_30(double *state, double *unused, double *out_6277762146048887703) {
   out_6277762146048887703[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7478173097827198850) {
   out_7478173097827198850[0] = 0;
   out_7478173097827198850[1] = 0;
   out_7478173097827198850[2] = 0;
   out_7478173097827198850[3] = 0;
   out_7478173097827198850[4] = 1;
   out_7478173097827198850[5] = 0;
   out_7478173097827198850[6] = 0;
   out_7478173097827198850[7] = 0;
   out_7478173097827198850[8] = 0;
}
void h_26(double *state, double *unused, double *out_7206601929979370681) {
   out_7206601929979370681[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3180020562538474129) {
   out_3180020562538474129[0] = 0;
   out_3180020562538474129[1] = 0;
   out_3180020562538474129[2] = 0;
   out_3180020562538474129[3] = 0;
   out_3180020562538474129[4] = 0;
   out_3180020562538474129[5] = 0;
   out_3180020562538474129[6] = 0;
   out_3180020562538474129[7] = 1;
   out_3180020562538474129[8] = 0;
}
void h_27(double *state, double *unused, double *out_3398162762955684163) {
   out_3398162762955684163[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5303409786026773939) {
   out_5303409786026773939[0] = 0;
   out_5303409786026773939[1] = 0;
   out_5303409786026773939[2] = 0;
   out_5303409786026773939[3] = 1;
   out_5303409786026773939[4] = 0;
   out_5303409786026773939[5] = 0;
   out_5303409786026773939[6] = 0;
   out_5303409786026773939[7] = 0;
   out_5303409786026773939[8] = 0;
}
void h_29(double *state, double *unused, double *out_7259846239384185815) {
   out_7259846239384185815[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3590047059157222906) {
   out_3590047059157222906[0] = 0;
   out_3590047059157222906[1] = 1;
   out_3590047059157222906[2] = 0;
   out_3590047059157222906[3] = 0;
   out_3590047059157222906[4] = 0;
   out_3590047059157222906[5] = 0;
   out_3590047059157222906[6] = 0;
   out_3590047059157222906[7] = 0;
   out_3590047059157222906[8] = 0;
}
void h_28(double *state, double *unused, double *out_585610659685830780) {
   out_585610659685830780[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1492351957912307668) {
   out_1492351957912307668[0] = 1;
   out_1492351957912307668[1] = 0;
   out_1492351957912307668[2] = 0;
   out_1492351957912307668[3] = 0;
   out_1492351957912307668[4] = 0;
   out_1492351957912307668[5] = 0;
   out_1492351957912307668[6] = 0;
   out_1492351957912307668[7] = 0;
   out_1492351957912307668[8] = 0;
}
void h_31(double *state, double *unused, double *out_5240960032959183832) {
   out_5240960032959183832[0] = state[8];
}
void H_31(double *state, double *unused, double *out_592128718212542523) {
   out_592128718212542523[0] = 0;
   out_592128718212542523[1] = 0;
   out_592128718212542523[2] = 0;
   out_592128718212542523[3] = 0;
   out_592128718212542523[4] = 0;
   out_592128718212542523[5] = 0;
   out_592128718212542523[6] = 0;
   out_592128718212542523[7] = 0;
   out_592128718212542523[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6184164552224961757) {
  err_fun(nom_x, delta_x, out_6184164552224961757);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1205517025722757514) {
  inv_err_fun(nom_x, true_x, out_1205517025722757514);
}
void car_H_mod_fun(double *state, double *out_8692073091461732284) {
  H_mod_fun(state, out_8692073091461732284);
}
void car_f_fun(double *state, double dt, double *out_6022347720067266341) {
  f_fun(state,  dt, out_6022347720067266341);
}
void car_F_fun(double *state, double dt, double *out_2034100707598491156) {
  F_fun(state,  dt, out_2034100707598491156);
}
void car_h_25(double *state, double *unused, double *out_1603510504705161482) {
  h_25(state, unused, out_1603510504705161482);
}
void car_H_25(double *state, double *unused, double *out_561482756335582095) {
  H_25(state, unused, out_561482756335582095);
}
void car_h_24(double *state, double *unused, double *out_8697386051471671882) {
  h_24(state, unused, out_8697386051471671882);
}
void car_H_24(double *state, double *unused, double *out_1611166842669917471) {
  H_24(state, unused, out_1611166842669917471);
}
void car_h_30(double *state, double *unused, double *out_6277762146048887703) {
  h_30(state, unused, out_6277762146048887703);
}
void car_H_30(double *state, double *unused, double *out_7478173097827198850) {
  H_30(state, unused, out_7478173097827198850);
}
void car_h_26(double *state, double *unused, double *out_7206601929979370681) {
  h_26(state, unused, out_7206601929979370681);
}
void car_H_26(double *state, double *unused, double *out_3180020562538474129) {
  H_26(state, unused, out_3180020562538474129);
}
void car_h_27(double *state, double *unused, double *out_3398162762955684163) {
  h_27(state, unused, out_3398162762955684163);
}
void car_H_27(double *state, double *unused, double *out_5303409786026773939) {
  H_27(state, unused, out_5303409786026773939);
}
void car_h_29(double *state, double *unused, double *out_7259846239384185815) {
  h_29(state, unused, out_7259846239384185815);
}
void car_H_29(double *state, double *unused, double *out_3590047059157222906) {
  H_29(state, unused, out_3590047059157222906);
}
void car_h_28(double *state, double *unused, double *out_585610659685830780) {
  h_28(state, unused, out_585610659685830780);
}
void car_H_28(double *state, double *unused, double *out_1492351957912307668) {
  H_28(state, unused, out_1492351957912307668);
}
void car_h_31(double *state, double *unused, double *out_5240960032959183832) {
  h_31(state, unused, out_5240960032959183832);
}
void car_H_31(double *state, double *unused, double *out_592128718212542523) {
  H_31(state, unused, out_592128718212542523);
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
