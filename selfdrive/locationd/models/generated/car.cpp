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
void err_fun(double *nom_x, double *delta_x, double *out_3639798656427526341) {
   out_3639798656427526341[0] = delta_x[0] + nom_x[0];
   out_3639798656427526341[1] = delta_x[1] + nom_x[1];
   out_3639798656427526341[2] = delta_x[2] + nom_x[2];
   out_3639798656427526341[3] = delta_x[3] + nom_x[3];
   out_3639798656427526341[4] = delta_x[4] + nom_x[4];
   out_3639798656427526341[5] = delta_x[5] + nom_x[5];
   out_3639798656427526341[6] = delta_x[6] + nom_x[6];
   out_3639798656427526341[7] = delta_x[7] + nom_x[7];
   out_3639798656427526341[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9141996593500649878) {
   out_9141996593500649878[0] = -nom_x[0] + true_x[0];
   out_9141996593500649878[1] = -nom_x[1] + true_x[1];
   out_9141996593500649878[2] = -nom_x[2] + true_x[2];
   out_9141996593500649878[3] = -nom_x[3] + true_x[3];
   out_9141996593500649878[4] = -nom_x[4] + true_x[4];
   out_9141996593500649878[5] = -nom_x[5] + true_x[5];
   out_9141996593500649878[6] = -nom_x[6] + true_x[6];
   out_9141996593500649878[7] = -nom_x[7] + true_x[7];
   out_9141996593500649878[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7268018853631160798) {
   out_7268018853631160798[0] = 1.0;
   out_7268018853631160798[1] = 0.0;
   out_7268018853631160798[2] = 0.0;
   out_7268018853631160798[3] = 0.0;
   out_7268018853631160798[4] = 0.0;
   out_7268018853631160798[5] = 0.0;
   out_7268018853631160798[6] = 0.0;
   out_7268018853631160798[7] = 0.0;
   out_7268018853631160798[8] = 0.0;
   out_7268018853631160798[9] = 0.0;
   out_7268018853631160798[10] = 1.0;
   out_7268018853631160798[11] = 0.0;
   out_7268018853631160798[12] = 0.0;
   out_7268018853631160798[13] = 0.0;
   out_7268018853631160798[14] = 0.0;
   out_7268018853631160798[15] = 0.0;
   out_7268018853631160798[16] = 0.0;
   out_7268018853631160798[17] = 0.0;
   out_7268018853631160798[18] = 0.0;
   out_7268018853631160798[19] = 0.0;
   out_7268018853631160798[20] = 1.0;
   out_7268018853631160798[21] = 0.0;
   out_7268018853631160798[22] = 0.0;
   out_7268018853631160798[23] = 0.0;
   out_7268018853631160798[24] = 0.0;
   out_7268018853631160798[25] = 0.0;
   out_7268018853631160798[26] = 0.0;
   out_7268018853631160798[27] = 0.0;
   out_7268018853631160798[28] = 0.0;
   out_7268018853631160798[29] = 0.0;
   out_7268018853631160798[30] = 1.0;
   out_7268018853631160798[31] = 0.0;
   out_7268018853631160798[32] = 0.0;
   out_7268018853631160798[33] = 0.0;
   out_7268018853631160798[34] = 0.0;
   out_7268018853631160798[35] = 0.0;
   out_7268018853631160798[36] = 0.0;
   out_7268018853631160798[37] = 0.0;
   out_7268018853631160798[38] = 0.0;
   out_7268018853631160798[39] = 0.0;
   out_7268018853631160798[40] = 1.0;
   out_7268018853631160798[41] = 0.0;
   out_7268018853631160798[42] = 0.0;
   out_7268018853631160798[43] = 0.0;
   out_7268018853631160798[44] = 0.0;
   out_7268018853631160798[45] = 0.0;
   out_7268018853631160798[46] = 0.0;
   out_7268018853631160798[47] = 0.0;
   out_7268018853631160798[48] = 0.0;
   out_7268018853631160798[49] = 0.0;
   out_7268018853631160798[50] = 1.0;
   out_7268018853631160798[51] = 0.0;
   out_7268018853631160798[52] = 0.0;
   out_7268018853631160798[53] = 0.0;
   out_7268018853631160798[54] = 0.0;
   out_7268018853631160798[55] = 0.0;
   out_7268018853631160798[56] = 0.0;
   out_7268018853631160798[57] = 0.0;
   out_7268018853631160798[58] = 0.0;
   out_7268018853631160798[59] = 0.0;
   out_7268018853631160798[60] = 1.0;
   out_7268018853631160798[61] = 0.0;
   out_7268018853631160798[62] = 0.0;
   out_7268018853631160798[63] = 0.0;
   out_7268018853631160798[64] = 0.0;
   out_7268018853631160798[65] = 0.0;
   out_7268018853631160798[66] = 0.0;
   out_7268018853631160798[67] = 0.0;
   out_7268018853631160798[68] = 0.0;
   out_7268018853631160798[69] = 0.0;
   out_7268018853631160798[70] = 1.0;
   out_7268018853631160798[71] = 0.0;
   out_7268018853631160798[72] = 0.0;
   out_7268018853631160798[73] = 0.0;
   out_7268018853631160798[74] = 0.0;
   out_7268018853631160798[75] = 0.0;
   out_7268018853631160798[76] = 0.0;
   out_7268018853631160798[77] = 0.0;
   out_7268018853631160798[78] = 0.0;
   out_7268018853631160798[79] = 0.0;
   out_7268018853631160798[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3932083014329006242) {
   out_3932083014329006242[0] = state[0];
   out_3932083014329006242[1] = state[1];
   out_3932083014329006242[2] = state[2];
   out_3932083014329006242[3] = state[3];
   out_3932083014329006242[4] = state[4];
   out_3932083014329006242[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3932083014329006242[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3932083014329006242[7] = state[7];
   out_3932083014329006242[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5001605271866367269) {
   out_5001605271866367269[0] = 1;
   out_5001605271866367269[1] = 0;
   out_5001605271866367269[2] = 0;
   out_5001605271866367269[3] = 0;
   out_5001605271866367269[4] = 0;
   out_5001605271866367269[5] = 0;
   out_5001605271866367269[6] = 0;
   out_5001605271866367269[7] = 0;
   out_5001605271866367269[8] = 0;
   out_5001605271866367269[9] = 0;
   out_5001605271866367269[10] = 1;
   out_5001605271866367269[11] = 0;
   out_5001605271866367269[12] = 0;
   out_5001605271866367269[13] = 0;
   out_5001605271866367269[14] = 0;
   out_5001605271866367269[15] = 0;
   out_5001605271866367269[16] = 0;
   out_5001605271866367269[17] = 0;
   out_5001605271866367269[18] = 0;
   out_5001605271866367269[19] = 0;
   out_5001605271866367269[20] = 1;
   out_5001605271866367269[21] = 0;
   out_5001605271866367269[22] = 0;
   out_5001605271866367269[23] = 0;
   out_5001605271866367269[24] = 0;
   out_5001605271866367269[25] = 0;
   out_5001605271866367269[26] = 0;
   out_5001605271866367269[27] = 0;
   out_5001605271866367269[28] = 0;
   out_5001605271866367269[29] = 0;
   out_5001605271866367269[30] = 1;
   out_5001605271866367269[31] = 0;
   out_5001605271866367269[32] = 0;
   out_5001605271866367269[33] = 0;
   out_5001605271866367269[34] = 0;
   out_5001605271866367269[35] = 0;
   out_5001605271866367269[36] = 0;
   out_5001605271866367269[37] = 0;
   out_5001605271866367269[38] = 0;
   out_5001605271866367269[39] = 0;
   out_5001605271866367269[40] = 1;
   out_5001605271866367269[41] = 0;
   out_5001605271866367269[42] = 0;
   out_5001605271866367269[43] = 0;
   out_5001605271866367269[44] = 0;
   out_5001605271866367269[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5001605271866367269[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5001605271866367269[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5001605271866367269[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5001605271866367269[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5001605271866367269[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5001605271866367269[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5001605271866367269[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5001605271866367269[53] = -9.8000000000000007*dt;
   out_5001605271866367269[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5001605271866367269[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5001605271866367269[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5001605271866367269[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5001605271866367269[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5001605271866367269[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5001605271866367269[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5001605271866367269[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5001605271866367269[62] = 0;
   out_5001605271866367269[63] = 0;
   out_5001605271866367269[64] = 0;
   out_5001605271866367269[65] = 0;
   out_5001605271866367269[66] = 0;
   out_5001605271866367269[67] = 0;
   out_5001605271866367269[68] = 0;
   out_5001605271866367269[69] = 0;
   out_5001605271866367269[70] = 1;
   out_5001605271866367269[71] = 0;
   out_5001605271866367269[72] = 0;
   out_5001605271866367269[73] = 0;
   out_5001605271866367269[74] = 0;
   out_5001605271866367269[75] = 0;
   out_5001605271866367269[76] = 0;
   out_5001605271866367269[77] = 0;
   out_5001605271866367269[78] = 0;
   out_5001605271866367269[79] = 0;
   out_5001605271866367269[80] = 1;
}
void h_25(double *state, double *unused, double *out_8062841473513369717) {
   out_8062841473513369717[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2364327028446495958) {
   out_2364327028446495958[0] = 0;
   out_2364327028446495958[1] = 0;
   out_2364327028446495958[2] = 0;
   out_2364327028446495958[3] = 0;
   out_2364327028446495958[4] = 0;
   out_2364327028446495958[5] = 0;
   out_2364327028446495958[6] = 1;
   out_2364327028446495958[7] = 0;
   out_2364327028446495958[8] = 0;
}
void h_24(double *state, double *unused, double *out_2051114860567036014) {
   out_2051114860567036014[0] = state[4];
   out_2051114860567036014[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7233141893474202810) {
   out_7233141893474202810[0] = 0;
   out_7233141893474202810[1] = 0;
   out_7233141893474202810[2] = 0;
   out_7233141893474202810[3] = 0;
   out_7233141893474202810[4] = 1;
   out_7233141893474202810[5] = 0;
   out_7233141893474202810[6] = 0;
   out_7233141893474202810[7] = 0;
   out_7233141893474202810[8] = 0;
   out_7233141893474202810[9] = 0;
   out_7233141893474202810[10] = 0;
   out_7233141893474202810[11] = 0;
   out_7233141893474202810[12] = 0;
   out_7233141893474202810[13] = 0;
   out_7233141893474202810[14] = 1;
   out_7233141893474202810[15] = 0;
   out_7233141893474202810[16] = 0;
   out_7233141893474202810[17] = 0;
}
void h_30(double *state, double *unused, double *out_4425391945259347367) {
   out_4425391945259347367[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6892023358574104156) {
   out_6892023358574104156[0] = 0;
   out_6892023358574104156[1] = 0;
   out_6892023358574104156[2] = 0;
   out_6892023358574104156[3] = 0;
   out_6892023358574104156[4] = 1;
   out_6892023358574104156[5] = 0;
   out_6892023358574104156[6] = 0;
   out_6892023358574104156[7] = 0;
   out_6892023358574104156[8] = 0;
}
void h_26(double *state, double *unused, double *out_1388605893815014682) {
   out_1388605893815014682[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6105830347320552182) {
   out_6105830347320552182[0] = 0;
   out_6105830347320552182[1] = 0;
   out_6105830347320552182[2] = 0;
   out_6105830347320552182[3] = 0;
   out_6105830347320552182[4] = 0;
   out_6105830347320552182[5] = 0;
   out_6105830347320552182[6] = 0;
   out_6105830347320552182[7] = 1;
   out_6105830347320552182[8] = 0;
}
void h_27(double *state, double *unused, double *out_9006351326415032152) {
   out_9006351326415032152[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4668429287390160939) {
   out_4668429287390160939[0] = 0;
   out_4668429287390160939[1] = 0;
   out_4668429287390160939[2] = 0;
   out_4668429287390160939[3] = 1;
   out_4668429287390160939[4] = 0;
   out_4668429287390160939[5] = 0;
   out_4668429287390160939[6] = 0;
   out_4668429287390160939[7] = 0;
   out_4668429287390160939[8] = 0;
}
void h_29(double *state, double *unused, double *out_5368901798161009802) {
   out_5368901798161009802[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6381792014259711972) {
   out_6381792014259711972[0] = 0;
   out_6381792014259711972[1] = 1;
   out_6381792014259711972[2] = 0;
   out_6381792014259711972[3] = 0;
   out_6381792014259711972[4] = 0;
   out_6381792014259711972[5] = 0;
   out_6381792014259711972[6] = 0;
   out_6381792014259711972[7] = 0;
   out_6381792014259711972[8] = 0;
}
void h_28(double *state, double *unused, double *out_6403606695850186779) {
   out_6403606695850186779[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4418161742694385721) {
   out_4418161742694385721[0] = 1;
   out_4418161742694385721[1] = 0;
   out_4418161742694385721[2] = 0;
   out_4418161742694385721[3] = 0;
   out_4418161742694385721[4] = 0;
   out_4418161742694385721[5] = 0;
   out_4418161742694385721[6] = 0;
   out_4418161742694385721[7] = 0;
   out_4418161742694385721[8] = 0;
}
void h_31(double *state, double *unused, double *out_6244759337255205356) {
   out_6244759337255205356[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6732038449553903658) {
   out_6732038449553903658[0] = 0;
   out_6732038449553903658[1] = 0;
   out_6732038449553903658[2] = 0;
   out_6732038449553903658[3] = 0;
   out_6732038449553903658[4] = 0;
   out_6732038449553903658[5] = 0;
   out_6732038449553903658[6] = 0;
   out_6732038449553903658[7] = 0;
   out_6732038449553903658[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3639798656427526341) {
  err_fun(nom_x, delta_x, out_3639798656427526341);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9141996593500649878) {
  inv_err_fun(nom_x, true_x, out_9141996593500649878);
}
void car_H_mod_fun(double *state, double *out_7268018853631160798) {
  H_mod_fun(state, out_7268018853631160798);
}
void car_f_fun(double *state, double dt, double *out_3932083014329006242) {
  f_fun(state,  dt, out_3932083014329006242);
}
void car_F_fun(double *state, double dt, double *out_5001605271866367269) {
  F_fun(state,  dt, out_5001605271866367269);
}
void car_h_25(double *state, double *unused, double *out_8062841473513369717) {
  h_25(state, unused, out_8062841473513369717);
}
void car_H_25(double *state, double *unused, double *out_2364327028446495958) {
  H_25(state, unused, out_2364327028446495958);
}
void car_h_24(double *state, double *unused, double *out_2051114860567036014) {
  h_24(state, unused, out_2051114860567036014);
}
void car_H_24(double *state, double *unused, double *out_7233141893474202810) {
  H_24(state, unused, out_7233141893474202810);
}
void car_h_30(double *state, double *unused, double *out_4425391945259347367) {
  h_30(state, unused, out_4425391945259347367);
}
void car_H_30(double *state, double *unused, double *out_6892023358574104156) {
  H_30(state, unused, out_6892023358574104156);
}
void car_h_26(double *state, double *unused, double *out_1388605893815014682) {
  h_26(state, unused, out_1388605893815014682);
}
void car_H_26(double *state, double *unused, double *out_6105830347320552182) {
  H_26(state, unused, out_6105830347320552182);
}
void car_h_27(double *state, double *unused, double *out_9006351326415032152) {
  h_27(state, unused, out_9006351326415032152);
}
void car_H_27(double *state, double *unused, double *out_4668429287390160939) {
  H_27(state, unused, out_4668429287390160939);
}
void car_h_29(double *state, double *unused, double *out_5368901798161009802) {
  h_29(state, unused, out_5368901798161009802);
}
void car_H_29(double *state, double *unused, double *out_6381792014259711972) {
  H_29(state, unused, out_6381792014259711972);
}
void car_h_28(double *state, double *unused, double *out_6403606695850186779) {
  h_28(state, unused, out_6403606695850186779);
}
void car_H_28(double *state, double *unused, double *out_4418161742694385721) {
  H_28(state, unused, out_4418161742694385721);
}
void car_h_31(double *state, double *unused, double *out_6244759337255205356) {
  h_31(state, unused, out_6244759337255205356);
}
void car_H_31(double *state, double *unused, double *out_6732038449553903658) {
  H_31(state, unused, out_6732038449553903658);
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
