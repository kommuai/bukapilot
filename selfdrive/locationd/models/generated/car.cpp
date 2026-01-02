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
void err_fun(double *nom_x, double *delta_x, double *out_185066053000096811) {
   out_185066053000096811[0] = delta_x[0] + nom_x[0];
   out_185066053000096811[1] = delta_x[1] + nom_x[1];
   out_185066053000096811[2] = delta_x[2] + nom_x[2];
   out_185066053000096811[3] = delta_x[3] + nom_x[3];
   out_185066053000096811[4] = delta_x[4] + nom_x[4];
   out_185066053000096811[5] = delta_x[5] + nom_x[5];
   out_185066053000096811[6] = delta_x[6] + nom_x[6];
   out_185066053000096811[7] = delta_x[7] + nom_x[7];
   out_185066053000096811[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1419290231265680633) {
   out_1419290231265680633[0] = -nom_x[0] + true_x[0];
   out_1419290231265680633[1] = -nom_x[1] + true_x[1];
   out_1419290231265680633[2] = -nom_x[2] + true_x[2];
   out_1419290231265680633[3] = -nom_x[3] + true_x[3];
   out_1419290231265680633[4] = -nom_x[4] + true_x[4];
   out_1419290231265680633[5] = -nom_x[5] + true_x[5];
   out_1419290231265680633[6] = -nom_x[6] + true_x[6];
   out_1419290231265680633[7] = -nom_x[7] + true_x[7];
   out_1419290231265680633[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4094801764284795946) {
   out_4094801764284795946[0] = 1.0;
   out_4094801764284795946[1] = 0.0;
   out_4094801764284795946[2] = 0.0;
   out_4094801764284795946[3] = 0.0;
   out_4094801764284795946[4] = 0.0;
   out_4094801764284795946[5] = 0.0;
   out_4094801764284795946[6] = 0.0;
   out_4094801764284795946[7] = 0.0;
   out_4094801764284795946[8] = 0.0;
   out_4094801764284795946[9] = 0.0;
   out_4094801764284795946[10] = 1.0;
   out_4094801764284795946[11] = 0.0;
   out_4094801764284795946[12] = 0.0;
   out_4094801764284795946[13] = 0.0;
   out_4094801764284795946[14] = 0.0;
   out_4094801764284795946[15] = 0.0;
   out_4094801764284795946[16] = 0.0;
   out_4094801764284795946[17] = 0.0;
   out_4094801764284795946[18] = 0.0;
   out_4094801764284795946[19] = 0.0;
   out_4094801764284795946[20] = 1.0;
   out_4094801764284795946[21] = 0.0;
   out_4094801764284795946[22] = 0.0;
   out_4094801764284795946[23] = 0.0;
   out_4094801764284795946[24] = 0.0;
   out_4094801764284795946[25] = 0.0;
   out_4094801764284795946[26] = 0.0;
   out_4094801764284795946[27] = 0.0;
   out_4094801764284795946[28] = 0.0;
   out_4094801764284795946[29] = 0.0;
   out_4094801764284795946[30] = 1.0;
   out_4094801764284795946[31] = 0.0;
   out_4094801764284795946[32] = 0.0;
   out_4094801764284795946[33] = 0.0;
   out_4094801764284795946[34] = 0.0;
   out_4094801764284795946[35] = 0.0;
   out_4094801764284795946[36] = 0.0;
   out_4094801764284795946[37] = 0.0;
   out_4094801764284795946[38] = 0.0;
   out_4094801764284795946[39] = 0.0;
   out_4094801764284795946[40] = 1.0;
   out_4094801764284795946[41] = 0.0;
   out_4094801764284795946[42] = 0.0;
   out_4094801764284795946[43] = 0.0;
   out_4094801764284795946[44] = 0.0;
   out_4094801764284795946[45] = 0.0;
   out_4094801764284795946[46] = 0.0;
   out_4094801764284795946[47] = 0.0;
   out_4094801764284795946[48] = 0.0;
   out_4094801764284795946[49] = 0.0;
   out_4094801764284795946[50] = 1.0;
   out_4094801764284795946[51] = 0.0;
   out_4094801764284795946[52] = 0.0;
   out_4094801764284795946[53] = 0.0;
   out_4094801764284795946[54] = 0.0;
   out_4094801764284795946[55] = 0.0;
   out_4094801764284795946[56] = 0.0;
   out_4094801764284795946[57] = 0.0;
   out_4094801764284795946[58] = 0.0;
   out_4094801764284795946[59] = 0.0;
   out_4094801764284795946[60] = 1.0;
   out_4094801764284795946[61] = 0.0;
   out_4094801764284795946[62] = 0.0;
   out_4094801764284795946[63] = 0.0;
   out_4094801764284795946[64] = 0.0;
   out_4094801764284795946[65] = 0.0;
   out_4094801764284795946[66] = 0.0;
   out_4094801764284795946[67] = 0.0;
   out_4094801764284795946[68] = 0.0;
   out_4094801764284795946[69] = 0.0;
   out_4094801764284795946[70] = 1.0;
   out_4094801764284795946[71] = 0.0;
   out_4094801764284795946[72] = 0.0;
   out_4094801764284795946[73] = 0.0;
   out_4094801764284795946[74] = 0.0;
   out_4094801764284795946[75] = 0.0;
   out_4094801764284795946[76] = 0.0;
   out_4094801764284795946[77] = 0.0;
   out_4094801764284795946[78] = 0.0;
   out_4094801764284795946[79] = 0.0;
   out_4094801764284795946[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5978480815451850811) {
   out_5978480815451850811[0] = state[0];
   out_5978480815451850811[1] = state[1];
   out_5978480815451850811[2] = state[2];
   out_5978480815451850811[3] = state[3];
   out_5978480815451850811[4] = state[4];
   out_5978480815451850811[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5978480815451850811[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5978480815451850811[7] = state[7];
   out_5978480815451850811[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7673283050767777171) {
   out_7673283050767777171[0] = 1;
   out_7673283050767777171[1] = 0;
   out_7673283050767777171[2] = 0;
   out_7673283050767777171[3] = 0;
   out_7673283050767777171[4] = 0;
   out_7673283050767777171[5] = 0;
   out_7673283050767777171[6] = 0;
   out_7673283050767777171[7] = 0;
   out_7673283050767777171[8] = 0;
   out_7673283050767777171[9] = 0;
   out_7673283050767777171[10] = 1;
   out_7673283050767777171[11] = 0;
   out_7673283050767777171[12] = 0;
   out_7673283050767777171[13] = 0;
   out_7673283050767777171[14] = 0;
   out_7673283050767777171[15] = 0;
   out_7673283050767777171[16] = 0;
   out_7673283050767777171[17] = 0;
   out_7673283050767777171[18] = 0;
   out_7673283050767777171[19] = 0;
   out_7673283050767777171[20] = 1;
   out_7673283050767777171[21] = 0;
   out_7673283050767777171[22] = 0;
   out_7673283050767777171[23] = 0;
   out_7673283050767777171[24] = 0;
   out_7673283050767777171[25] = 0;
   out_7673283050767777171[26] = 0;
   out_7673283050767777171[27] = 0;
   out_7673283050767777171[28] = 0;
   out_7673283050767777171[29] = 0;
   out_7673283050767777171[30] = 1;
   out_7673283050767777171[31] = 0;
   out_7673283050767777171[32] = 0;
   out_7673283050767777171[33] = 0;
   out_7673283050767777171[34] = 0;
   out_7673283050767777171[35] = 0;
   out_7673283050767777171[36] = 0;
   out_7673283050767777171[37] = 0;
   out_7673283050767777171[38] = 0;
   out_7673283050767777171[39] = 0;
   out_7673283050767777171[40] = 1;
   out_7673283050767777171[41] = 0;
   out_7673283050767777171[42] = 0;
   out_7673283050767777171[43] = 0;
   out_7673283050767777171[44] = 0;
   out_7673283050767777171[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7673283050767777171[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7673283050767777171[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7673283050767777171[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7673283050767777171[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7673283050767777171[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7673283050767777171[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7673283050767777171[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7673283050767777171[53] = -9.8000000000000007*dt;
   out_7673283050767777171[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7673283050767777171[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7673283050767777171[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7673283050767777171[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7673283050767777171[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7673283050767777171[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7673283050767777171[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7673283050767777171[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7673283050767777171[62] = 0;
   out_7673283050767777171[63] = 0;
   out_7673283050767777171[64] = 0;
   out_7673283050767777171[65] = 0;
   out_7673283050767777171[66] = 0;
   out_7673283050767777171[67] = 0;
   out_7673283050767777171[68] = 0;
   out_7673283050767777171[69] = 0;
   out_7673283050767777171[70] = 1;
   out_7673283050767777171[71] = 0;
   out_7673283050767777171[72] = 0;
   out_7673283050767777171[73] = 0;
   out_7673283050767777171[74] = 0;
   out_7673283050767777171[75] = 0;
   out_7673283050767777171[76] = 0;
   out_7673283050767777171[77] = 0;
   out_7673283050767777171[78] = 0;
   out_7673283050767777171[79] = 0;
   out_7673283050767777171[80] = 1;
}
void h_25(double *state, double *unused, double *out_2480916803033305557) {
   out_2480916803033305557[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6430604276450895865) {
   out_6430604276450895865[0] = 0;
   out_6430604276450895865[1] = 0;
   out_6430604276450895865[2] = 0;
   out_6430604276450895865[3] = 0;
   out_6430604276450895865[4] = 0;
   out_6430604276450895865[5] = 0;
   out_6430604276450895865[6] = 1;
   out_6430604276450895865[7] = 0;
   out_6430604276450895865[8] = 0;
}
void h_24(double *state, double *unused, double *out_8982522961258785433) {
   out_8982522961258785433[0] = state[4];
   out_8982522961258785433[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5017576028412487028) {
   out_5017576028412487028[0] = 0;
   out_5017576028412487028[1] = 0;
   out_5017576028412487028[2] = 0;
   out_5017576028412487028[3] = 0;
   out_5017576028412487028[4] = 1;
   out_5017576028412487028[5] = 0;
   out_5017576028412487028[6] = 0;
   out_5017576028412487028[7] = 0;
   out_5017576028412487028[8] = 0;
   out_5017576028412487028[9] = 0;
   out_5017576028412487028[10] = 0;
   out_5017576028412487028[11] = 0;
   out_5017576028412487028[12] = 0;
   out_5017576028412487028[13] = 0;
   out_5017576028412487028[14] = 1;
   out_5017576028412487028[15] = 0;
   out_5017576028412487028[16] = 0;
   out_5017576028412487028[17] = 0;
}
void h_30(double *state, double *unused, double *out_4706036101424752772) {
   out_4706036101424752772[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7488443467131047553) {
   out_7488443467131047553[0] = 0;
   out_7488443467131047553[1] = 0;
   out_7488443467131047553[2] = 0;
   out_7488443467131047553[3] = 0;
   out_7488443467131047553[4] = 1;
   out_7488443467131047553[5] = 0;
   out_7488443467131047553[6] = 0;
   out_7488443467131047553[7] = 0;
   out_7488443467131047553[8] = 0;
}
void h_26(double *state, double *unused, double *out_2101630780616055121) {
   out_2101630780616055121[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8274636478384599527) {
   out_8274636478384599527[0] = 0;
   out_8274636478384599527[1] = 0;
   out_8274636478384599527[2] = 0;
   out_8274636478384599527[3] = 0;
   out_8274636478384599527[4] = 0;
   out_8274636478384599527[5] = 0;
   out_8274636478384599527[6] = 0;
   out_8274636478384599527[7] = 1;
   out_8274636478384599527[8] = 0;
}
void h_27(double *state, double *unused, double *out_7482590070694151202) {
   out_7482590070694151202[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8734706535394560846) {
   out_8734706535394560846[0] = 0;
   out_8734706535394560846[1] = 0;
   out_8734706535394560846[2] = 0;
   out_8734706535394560846[3] = 1;
   out_8734706535394560846[4] = 0;
   out_8734706535394560846[5] = 0;
   out_8734706535394560846[6] = 0;
   out_8734706535394560846[7] = 0;
   out_8734706535394560846[8] = 0;
}
void h_29(double *state, double *unused, double *out_8016428012659915453) {
   out_8016428012659915453[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7998674811445439737) {
   out_7998674811445439737[0] = 0;
   out_7998674811445439737[1] = 1;
   out_7998674811445439737[2] = 0;
   out_7998674811445439737[3] = 0;
   out_7998674811445439737[4] = 0;
   out_7998674811445439737[5] = 0;
   out_7998674811445439737[6] = 0;
   out_7998674811445439737[7] = 0;
   out_7998674811445439737[8] = 0;
}
void h_28(double *state, double *unused, double *out_6122962539684781374) {
   out_6122962539684781374[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2916275794375909163) {
   out_2916275794375909163[0] = 1;
   out_2916275794375909163[1] = 0;
   out_2916275794375909163[2] = 0;
   out_2916275794375909163[3] = 0;
   out_2916275794375909163[4] = 0;
   out_2916275794375909163[5] = 0;
   out_2916275794375909163[6] = 0;
   out_2916275794375909163[7] = 0;
   out_2916275794375909163[8] = 0;
}
void h_31(double *state, double *unused, double *out_1156532725220716793) {
   out_1156532725220716793[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7648428376151248051) {
   out_7648428376151248051[0] = 0;
   out_7648428376151248051[1] = 0;
   out_7648428376151248051[2] = 0;
   out_7648428376151248051[3] = 0;
   out_7648428376151248051[4] = 0;
   out_7648428376151248051[5] = 0;
   out_7648428376151248051[6] = 0;
   out_7648428376151248051[7] = 0;
   out_7648428376151248051[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_185066053000096811) {
  err_fun(nom_x, delta_x, out_185066053000096811);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1419290231265680633) {
  inv_err_fun(nom_x, true_x, out_1419290231265680633);
}
void car_H_mod_fun(double *state, double *out_4094801764284795946) {
  H_mod_fun(state, out_4094801764284795946);
}
void car_f_fun(double *state, double dt, double *out_5978480815451850811) {
  f_fun(state,  dt, out_5978480815451850811);
}
void car_F_fun(double *state, double dt, double *out_7673283050767777171) {
  F_fun(state,  dt, out_7673283050767777171);
}
void car_h_25(double *state, double *unused, double *out_2480916803033305557) {
  h_25(state, unused, out_2480916803033305557);
}
void car_H_25(double *state, double *unused, double *out_6430604276450895865) {
  H_25(state, unused, out_6430604276450895865);
}
void car_h_24(double *state, double *unused, double *out_8982522961258785433) {
  h_24(state, unused, out_8982522961258785433);
}
void car_H_24(double *state, double *unused, double *out_5017576028412487028) {
  H_24(state, unused, out_5017576028412487028);
}
void car_h_30(double *state, double *unused, double *out_4706036101424752772) {
  h_30(state, unused, out_4706036101424752772);
}
void car_H_30(double *state, double *unused, double *out_7488443467131047553) {
  H_30(state, unused, out_7488443467131047553);
}
void car_h_26(double *state, double *unused, double *out_2101630780616055121) {
  h_26(state, unused, out_2101630780616055121);
}
void car_H_26(double *state, double *unused, double *out_8274636478384599527) {
  H_26(state, unused, out_8274636478384599527);
}
void car_h_27(double *state, double *unused, double *out_7482590070694151202) {
  h_27(state, unused, out_7482590070694151202);
}
void car_H_27(double *state, double *unused, double *out_8734706535394560846) {
  H_27(state, unused, out_8734706535394560846);
}
void car_h_29(double *state, double *unused, double *out_8016428012659915453) {
  h_29(state, unused, out_8016428012659915453);
}
void car_H_29(double *state, double *unused, double *out_7998674811445439737) {
  H_29(state, unused, out_7998674811445439737);
}
void car_h_28(double *state, double *unused, double *out_6122962539684781374) {
  h_28(state, unused, out_6122962539684781374);
}
void car_H_28(double *state, double *unused, double *out_2916275794375909163) {
  H_28(state, unused, out_2916275794375909163);
}
void car_h_31(double *state, double *unused, double *out_1156532725220716793) {
  h_31(state, unused, out_1156532725220716793);
}
void car_H_31(double *state, double *unused, double *out_7648428376151248051) {
  H_31(state, unused, out_7648428376151248051);
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
