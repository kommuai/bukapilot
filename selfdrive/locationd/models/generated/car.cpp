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
void err_fun(double *nom_x, double *delta_x, double *out_2161272052723994744) {
   out_2161272052723994744[0] = delta_x[0] + nom_x[0];
   out_2161272052723994744[1] = delta_x[1] + nom_x[1];
   out_2161272052723994744[2] = delta_x[2] + nom_x[2];
   out_2161272052723994744[3] = delta_x[3] + nom_x[3];
   out_2161272052723994744[4] = delta_x[4] + nom_x[4];
   out_2161272052723994744[5] = delta_x[5] + nom_x[5];
   out_2161272052723994744[6] = delta_x[6] + nom_x[6];
   out_2161272052723994744[7] = delta_x[7] + nom_x[7];
   out_2161272052723994744[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6541742512428809330) {
   out_6541742512428809330[0] = -nom_x[0] + true_x[0];
   out_6541742512428809330[1] = -nom_x[1] + true_x[1];
   out_6541742512428809330[2] = -nom_x[2] + true_x[2];
   out_6541742512428809330[3] = -nom_x[3] + true_x[3];
   out_6541742512428809330[4] = -nom_x[4] + true_x[4];
   out_6541742512428809330[5] = -nom_x[5] + true_x[5];
   out_6541742512428809330[6] = -nom_x[6] + true_x[6];
   out_6541742512428809330[7] = -nom_x[7] + true_x[7];
   out_6541742512428809330[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4810974423062247760) {
   out_4810974423062247760[0] = 1.0;
   out_4810974423062247760[1] = 0.0;
   out_4810974423062247760[2] = 0.0;
   out_4810974423062247760[3] = 0.0;
   out_4810974423062247760[4] = 0.0;
   out_4810974423062247760[5] = 0.0;
   out_4810974423062247760[6] = 0.0;
   out_4810974423062247760[7] = 0.0;
   out_4810974423062247760[8] = 0.0;
   out_4810974423062247760[9] = 0.0;
   out_4810974423062247760[10] = 1.0;
   out_4810974423062247760[11] = 0.0;
   out_4810974423062247760[12] = 0.0;
   out_4810974423062247760[13] = 0.0;
   out_4810974423062247760[14] = 0.0;
   out_4810974423062247760[15] = 0.0;
   out_4810974423062247760[16] = 0.0;
   out_4810974423062247760[17] = 0.0;
   out_4810974423062247760[18] = 0.0;
   out_4810974423062247760[19] = 0.0;
   out_4810974423062247760[20] = 1.0;
   out_4810974423062247760[21] = 0.0;
   out_4810974423062247760[22] = 0.0;
   out_4810974423062247760[23] = 0.0;
   out_4810974423062247760[24] = 0.0;
   out_4810974423062247760[25] = 0.0;
   out_4810974423062247760[26] = 0.0;
   out_4810974423062247760[27] = 0.0;
   out_4810974423062247760[28] = 0.0;
   out_4810974423062247760[29] = 0.0;
   out_4810974423062247760[30] = 1.0;
   out_4810974423062247760[31] = 0.0;
   out_4810974423062247760[32] = 0.0;
   out_4810974423062247760[33] = 0.0;
   out_4810974423062247760[34] = 0.0;
   out_4810974423062247760[35] = 0.0;
   out_4810974423062247760[36] = 0.0;
   out_4810974423062247760[37] = 0.0;
   out_4810974423062247760[38] = 0.0;
   out_4810974423062247760[39] = 0.0;
   out_4810974423062247760[40] = 1.0;
   out_4810974423062247760[41] = 0.0;
   out_4810974423062247760[42] = 0.0;
   out_4810974423062247760[43] = 0.0;
   out_4810974423062247760[44] = 0.0;
   out_4810974423062247760[45] = 0.0;
   out_4810974423062247760[46] = 0.0;
   out_4810974423062247760[47] = 0.0;
   out_4810974423062247760[48] = 0.0;
   out_4810974423062247760[49] = 0.0;
   out_4810974423062247760[50] = 1.0;
   out_4810974423062247760[51] = 0.0;
   out_4810974423062247760[52] = 0.0;
   out_4810974423062247760[53] = 0.0;
   out_4810974423062247760[54] = 0.0;
   out_4810974423062247760[55] = 0.0;
   out_4810974423062247760[56] = 0.0;
   out_4810974423062247760[57] = 0.0;
   out_4810974423062247760[58] = 0.0;
   out_4810974423062247760[59] = 0.0;
   out_4810974423062247760[60] = 1.0;
   out_4810974423062247760[61] = 0.0;
   out_4810974423062247760[62] = 0.0;
   out_4810974423062247760[63] = 0.0;
   out_4810974423062247760[64] = 0.0;
   out_4810974423062247760[65] = 0.0;
   out_4810974423062247760[66] = 0.0;
   out_4810974423062247760[67] = 0.0;
   out_4810974423062247760[68] = 0.0;
   out_4810974423062247760[69] = 0.0;
   out_4810974423062247760[70] = 1.0;
   out_4810974423062247760[71] = 0.0;
   out_4810974423062247760[72] = 0.0;
   out_4810974423062247760[73] = 0.0;
   out_4810974423062247760[74] = 0.0;
   out_4810974423062247760[75] = 0.0;
   out_4810974423062247760[76] = 0.0;
   out_4810974423062247760[77] = 0.0;
   out_4810974423062247760[78] = 0.0;
   out_4810974423062247760[79] = 0.0;
   out_4810974423062247760[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3010562746147677696) {
   out_3010562746147677696[0] = state[0];
   out_3010562746147677696[1] = state[1];
   out_3010562746147677696[2] = state[2];
   out_3010562746147677696[3] = state[3];
   out_3010562746147677696[4] = state[4];
   out_3010562746147677696[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3010562746147677696[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3010562746147677696[7] = state[7];
   out_3010562746147677696[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3081481798655148787) {
   out_3081481798655148787[0] = 1;
   out_3081481798655148787[1] = 0;
   out_3081481798655148787[2] = 0;
   out_3081481798655148787[3] = 0;
   out_3081481798655148787[4] = 0;
   out_3081481798655148787[5] = 0;
   out_3081481798655148787[6] = 0;
   out_3081481798655148787[7] = 0;
   out_3081481798655148787[8] = 0;
   out_3081481798655148787[9] = 0;
   out_3081481798655148787[10] = 1;
   out_3081481798655148787[11] = 0;
   out_3081481798655148787[12] = 0;
   out_3081481798655148787[13] = 0;
   out_3081481798655148787[14] = 0;
   out_3081481798655148787[15] = 0;
   out_3081481798655148787[16] = 0;
   out_3081481798655148787[17] = 0;
   out_3081481798655148787[18] = 0;
   out_3081481798655148787[19] = 0;
   out_3081481798655148787[20] = 1;
   out_3081481798655148787[21] = 0;
   out_3081481798655148787[22] = 0;
   out_3081481798655148787[23] = 0;
   out_3081481798655148787[24] = 0;
   out_3081481798655148787[25] = 0;
   out_3081481798655148787[26] = 0;
   out_3081481798655148787[27] = 0;
   out_3081481798655148787[28] = 0;
   out_3081481798655148787[29] = 0;
   out_3081481798655148787[30] = 1;
   out_3081481798655148787[31] = 0;
   out_3081481798655148787[32] = 0;
   out_3081481798655148787[33] = 0;
   out_3081481798655148787[34] = 0;
   out_3081481798655148787[35] = 0;
   out_3081481798655148787[36] = 0;
   out_3081481798655148787[37] = 0;
   out_3081481798655148787[38] = 0;
   out_3081481798655148787[39] = 0;
   out_3081481798655148787[40] = 1;
   out_3081481798655148787[41] = 0;
   out_3081481798655148787[42] = 0;
   out_3081481798655148787[43] = 0;
   out_3081481798655148787[44] = 0;
   out_3081481798655148787[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3081481798655148787[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3081481798655148787[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3081481798655148787[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3081481798655148787[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3081481798655148787[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3081481798655148787[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3081481798655148787[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3081481798655148787[53] = -9.8100000000000005*dt;
   out_3081481798655148787[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3081481798655148787[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3081481798655148787[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3081481798655148787[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3081481798655148787[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3081481798655148787[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3081481798655148787[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3081481798655148787[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3081481798655148787[62] = 0;
   out_3081481798655148787[63] = 0;
   out_3081481798655148787[64] = 0;
   out_3081481798655148787[65] = 0;
   out_3081481798655148787[66] = 0;
   out_3081481798655148787[67] = 0;
   out_3081481798655148787[68] = 0;
   out_3081481798655148787[69] = 0;
   out_3081481798655148787[70] = 1;
   out_3081481798655148787[71] = 0;
   out_3081481798655148787[72] = 0;
   out_3081481798655148787[73] = 0;
   out_3081481798655148787[74] = 0;
   out_3081481798655148787[75] = 0;
   out_3081481798655148787[76] = 0;
   out_3081481798655148787[77] = 0;
   out_3081481798655148787[78] = 0;
   out_3081481798655148787[79] = 0;
   out_3081481798655148787[80] = 1;
}
void h_25(double *state, double *unused, double *out_1026424918856168907) {
   out_1026424918856168907[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3110363609911612045) {
   out_3110363609911612045[0] = 0;
   out_3110363609911612045[1] = 0;
   out_3110363609911612045[2] = 0;
   out_3110363609911612045[3] = 0;
   out_3110363609911612045[4] = 0;
   out_3110363609911612045[5] = 0;
   out_3110363609911612045[6] = 1;
   out_3110363609911612045[7] = 0;
   out_3110363609911612045[8] = 0;
}
void h_24(double *state, double *unused, double *out_3822164015639221808) {
   out_3822164015639221808[0] = state[4];
   out_3822164015639221808[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2713718305557752550) {
   out_2713718305557752550[0] = 0;
   out_2713718305557752550[1] = 0;
   out_2713718305557752550[2] = 0;
   out_2713718305557752550[3] = 0;
   out_2713718305557752550[4] = 1;
   out_2713718305557752550[5] = 0;
   out_2713718305557752550[6] = 0;
   out_2713718305557752550[7] = 0;
   out_2713718305557752550[8] = 0;
   out_2713718305557752550[9] = 0;
   out_2713718305557752550[10] = 0;
   out_2713718305557752550[11] = 0;
   out_2713718305557752550[12] = 0;
   out_2713718305557752550[13] = 0;
   out_2713718305557752550[14] = 1;
   out_2713718305557752550[15] = 0;
   out_2713718305557752550[16] = 0;
   out_2713718305557752550[17] = 0;
}
void h_30(double *state, double *unused, double *out_5140689121114965396) {
   out_5140689121114965396[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5628696568418860672) {
   out_5628696568418860672[0] = 0;
   out_5628696568418860672[1] = 0;
   out_5628696568418860672[2] = 0;
   out_5628696568418860672[3] = 0;
   out_5628696568418860672[4] = 1;
   out_5628696568418860672[5] = 0;
   out_5628696568418860672[6] = 0;
   out_5628696568418860672[7] = 0;
   out_5628696568418860672[8] = 0;
}
void h_26(double *state, double *unused, double *out_1258352396298883750) {
   out_1258352396298883750[0] = state[7];
}
void H_26(double *state, double *unused, double *out_631139708962444179) {
   out_631139708962444179[0] = 0;
   out_631139708962444179[1] = 0;
   out_631139708962444179[2] = 0;
   out_631139708962444179[3] = 0;
   out_631139708962444179[4] = 0;
   out_631139708962444179[5] = 0;
   out_631139708962444179[6] = 0;
   out_631139708962444179[7] = 1;
   out_631139708962444179[8] = 0;
}
void h_27(double *state, double *unused, double *out_6028098186517014552) {
   out_6028098186517014552[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3453933256618435761) {
   out_3453933256618435761[0] = 0;
   out_3453933256618435761[1] = 0;
   out_3453933256618435761[2] = 0;
   out_3453933256618435761[3] = 1;
   out_3453933256618435761[4] = 0;
   out_3453933256618435761[5] = 0;
   out_3453933256618435761[6] = 0;
   out_3453933256618435761[7] = 0;
   out_3453933256618435761[8] = 0;
}
void h_29(double *state, double *unused, double *out_502040749965991439) {
   out_502040749965991439[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6138927912733252856) {
   out_6138927912733252856[0] = 0;
   out_6138927912733252856[1] = 1;
   out_6138927912733252856[2] = 0;
   out_6138927912733252856[3] = 0;
   out_6138927912733252856[4] = 0;
   out_6138927912733252856[5] = 0;
   out_6138927912733252856[6] = 0;
   out_6138927912733252856[7] = 0;
   out_6138927912733252856[8] = 0;
}
void h_28(double *state, double *unused, double *out_5084588333615352117) {
   out_5084588333615352117[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1056528895663722282) {
   out_1056528895663722282[0] = 1;
   out_1056528895663722282[1] = 0;
   out_1056528895663722282[2] = 0;
   out_1056528895663722282[3] = 0;
   out_1056528895663722282[4] = 0;
   out_1056528895663722282[5] = 0;
   out_1056528895663722282[6] = 0;
   out_1056528895663722282[7] = 0;
   out_1056528895663722282[8] = 0;
}
void h_31(double *state, double *unused, double *out_8789690175676841348) {
   out_8789690175676841348[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1257347811195795655) {
   out_1257347811195795655[0] = 0;
   out_1257347811195795655[1] = 0;
   out_1257347811195795655[2] = 0;
   out_1257347811195795655[3] = 0;
   out_1257347811195795655[4] = 0;
   out_1257347811195795655[5] = 0;
   out_1257347811195795655[6] = 0;
   out_1257347811195795655[7] = 0;
   out_1257347811195795655[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2161272052723994744) {
  err_fun(nom_x, delta_x, out_2161272052723994744);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6541742512428809330) {
  inv_err_fun(nom_x, true_x, out_6541742512428809330);
}
void car_H_mod_fun(double *state, double *out_4810974423062247760) {
  H_mod_fun(state, out_4810974423062247760);
}
void car_f_fun(double *state, double dt, double *out_3010562746147677696) {
  f_fun(state,  dt, out_3010562746147677696);
}
void car_F_fun(double *state, double dt, double *out_3081481798655148787) {
  F_fun(state,  dt, out_3081481798655148787);
}
void car_h_25(double *state, double *unused, double *out_1026424918856168907) {
  h_25(state, unused, out_1026424918856168907);
}
void car_H_25(double *state, double *unused, double *out_3110363609911612045) {
  H_25(state, unused, out_3110363609911612045);
}
void car_h_24(double *state, double *unused, double *out_3822164015639221808) {
  h_24(state, unused, out_3822164015639221808);
}
void car_H_24(double *state, double *unused, double *out_2713718305557752550) {
  H_24(state, unused, out_2713718305557752550);
}
void car_h_30(double *state, double *unused, double *out_5140689121114965396) {
  h_30(state, unused, out_5140689121114965396);
}
void car_H_30(double *state, double *unused, double *out_5628696568418860672) {
  H_30(state, unused, out_5628696568418860672);
}
void car_h_26(double *state, double *unused, double *out_1258352396298883750) {
  h_26(state, unused, out_1258352396298883750);
}
void car_H_26(double *state, double *unused, double *out_631139708962444179) {
  H_26(state, unused, out_631139708962444179);
}
void car_h_27(double *state, double *unused, double *out_6028098186517014552) {
  h_27(state, unused, out_6028098186517014552);
}
void car_H_27(double *state, double *unused, double *out_3453933256618435761) {
  H_27(state, unused, out_3453933256618435761);
}
void car_h_29(double *state, double *unused, double *out_502040749965991439) {
  h_29(state, unused, out_502040749965991439);
}
void car_H_29(double *state, double *unused, double *out_6138927912733252856) {
  H_29(state, unused, out_6138927912733252856);
}
void car_h_28(double *state, double *unused, double *out_5084588333615352117) {
  h_28(state, unused, out_5084588333615352117);
}
void car_H_28(double *state, double *unused, double *out_1056528895663722282) {
  H_28(state, unused, out_1056528895663722282);
}
void car_h_31(double *state, double *unused, double *out_8789690175676841348) {
  h_31(state, unused, out_8789690175676841348);
}
void car_H_31(double *state, double *unused, double *out_1257347811195795655) {
  H_31(state, unused, out_1257347811195795655);
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
