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
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8363476756673253557) {
   out_8363476756673253557[0] = delta_x[0] + nom_x[0];
   out_8363476756673253557[1] = delta_x[1] + nom_x[1];
   out_8363476756673253557[2] = delta_x[2] + nom_x[2];
   out_8363476756673253557[3] = delta_x[3] + nom_x[3];
   out_8363476756673253557[4] = delta_x[4] + nom_x[4];
   out_8363476756673253557[5] = delta_x[5] + nom_x[5];
   out_8363476756673253557[6] = delta_x[6] + nom_x[6];
   out_8363476756673253557[7] = delta_x[7] + nom_x[7];
   out_8363476756673253557[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1977797309478282067) {
   out_1977797309478282067[0] = -nom_x[0] + true_x[0];
   out_1977797309478282067[1] = -nom_x[1] + true_x[1];
   out_1977797309478282067[2] = -nom_x[2] + true_x[2];
   out_1977797309478282067[3] = -nom_x[3] + true_x[3];
   out_1977797309478282067[4] = -nom_x[4] + true_x[4];
   out_1977797309478282067[5] = -nom_x[5] + true_x[5];
   out_1977797309478282067[6] = -nom_x[6] + true_x[6];
   out_1977797309478282067[7] = -nom_x[7] + true_x[7];
   out_1977797309478282067[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_801006497277677196) {
   out_801006497277677196[0] = 1.0;
   out_801006497277677196[1] = 0;
   out_801006497277677196[2] = 0;
   out_801006497277677196[3] = 0;
   out_801006497277677196[4] = 0;
   out_801006497277677196[5] = 0;
   out_801006497277677196[6] = 0;
   out_801006497277677196[7] = 0;
   out_801006497277677196[8] = 0;
   out_801006497277677196[9] = 0;
   out_801006497277677196[10] = 1.0;
   out_801006497277677196[11] = 0;
   out_801006497277677196[12] = 0;
   out_801006497277677196[13] = 0;
   out_801006497277677196[14] = 0;
   out_801006497277677196[15] = 0;
   out_801006497277677196[16] = 0;
   out_801006497277677196[17] = 0;
   out_801006497277677196[18] = 0;
   out_801006497277677196[19] = 0;
   out_801006497277677196[20] = 1.0;
   out_801006497277677196[21] = 0;
   out_801006497277677196[22] = 0;
   out_801006497277677196[23] = 0;
   out_801006497277677196[24] = 0;
   out_801006497277677196[25] = 0;
   out_801006497277677196[26] = 0;
   out_801006497277677196[27] = 0;
   out_801006497277677196[28] = 0;
   out_801006497277677196[29] = 0;
   out_801006497277677196[30] = 1.0;
   out_801006497277677196[31] = 0;
   out_801006497277677196[32] = 0;
   out_801006497277677196[33] = 0;
   out_801006497277677196[34] = 0;
   out_801006497277677196[35] = 0;
   out_801006497277677196[36] = 0;
   out_801006497277677196[37] = 0;
   out_801006497277677196[38] = 0;
   out_801006497277677196[39] = 0;
   out_801006497277677196[40] = 1.0;
   out_801006497277677196[41] = 0;
   out_801006497277677196[42] = 0;
   out_801006497277677196[43] = 0;
   out_801006497277677196[44] = 0;
   out_801006497277677196[45] = 0;
   out_801006497277677196[46] = 0;
   out_801006497277677196[47] = 0;
   out_801006497277677196[48] = 0;
   out_801006497277677196[49] = 0;
   out_801006497277677196[50] = 1.0;
   out_801006497277677196[51] = 0;
   out_801006497277677196[52] = 0;
   out_801006497277677196[53] = 0;
   out_801006497277677196[54] = 0;
   out_801006497277677196[55] = 0;
   out_801006497277677196[56] = 0;
   out_801006497277677196[57] = 0;
   out_801006497277677196[58] = 0;
   out_801006497277677196[59] = 0;
   out_801006497277677196[60] = 1.0;
   out_801006497277677196[61] = 0;
   out_801006497277677196[62] = 0;
   out_801006497277677196[63] = 0;
   out_801006497277677196[64] = 0;
   out_801006497277677196[65] = 0;
   out_801006497277677196[66] = 0;
   out_801006497277677196[67] = 0;
   out_801006497277677196[68] = 0;
   out_801006497277677196[69] = 0;
   out_801006497277677196[70] = 1.0;
   out_801006497277677196[71] = 0;
   out_801006497277677196[72] = 0;
   out_801006497277677196[73] = 0;
   out_801006497277677196[74] = 0;
   out_801006497277677196[75] = 0;
   out_801006497277677196[76] = 0;
   out_801006497277677196[77] = 0;
   out_801006497277677196[78] = 0;
   out_801006497277677196[79] = 0;
   out_801006497277677196[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4787359001335959165) {
   out_4787359001335959165[0] = state[0];
   out_4787359001335959165[1] = state[1];
   out_4787359001335959165[2] = state[2];
   out_4787359001335959165[3] = state[3];
   out_4787359001335959165[4] = state[4];
   out_4787359001335959165[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4787359001335959165[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4787359001335959165[7] = state[7];
   out_4787359001335959165[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4451177536231554248) {
   out_4451177536231554248[0] = 1;
   out_4451177536231554248[1] = 0;
   out_4451177536231554248[2] = 0;
   out_4451177536231554248[3] = 0;
   out_4451177536231554248[4] = 0;
   out_4451177536231554248[5] = 0;
   out_4451177536231554248[6] = 0;
   out_4451177536231554248[7] = 0;
   out_4451177536231554248[8] = 0;
   out_4451177536231554248[9] = 0;
   out_4451177536231554248[10] = 1;
   out_4451177536231554248[11] = 0;
   out_4451177536231554248[12] = 0;
   out_4451177536231554248[13] = 0;
   out_4451177536231554248[14] = 0;
   out_4451177536231554248[15] = 0;
   out_4451177536231554248[16] = 0;
   out_4451177536231554248[17] = 0;
   out_4451177536231554248[18] = 0;
   out_4451177536231554248[19] = 0;
   out_4451177536231554248[20] = 1;
   out_4451177536231554248[21] = 0;
   out_4451177536231554248[22] = 0;
   out_4451177536231554248[23] = 0;
   out_4451177536231554248[24] = 0;
   out_4451177536231554248[25] = 0;
   out_4451177536231554248[26] = 0;
   out_4451177536231554248[27] = 0;
   out_4451177536231554248[28] = 0;
   out_4451177536231554248[29] = 0;
   out_4451177536231554248[30] = 1;
   out_4451177536231554248[31] = 0;
   out_4451177536231554248[32] = 0;
   out_4451177536231554248[33] = 0;
   out_4451177536231554248[34] = 0;
   out_4451177536231554248[35] = 0;
   out_4451177536231554248[36] = 0;
   out_4451177536231554248[37] = 0;
   out_4451177536231554248[38] = 0;
   out_4451177536231554248[39] = 0;
   out_4451177536231554248[40] = 1;
   out_4451177536231554248[41] = 0;
   out_4451177536231554248[42] = 0;
   out_4451177536231554248[43] = 0;
   out_4451177536231554248[44] = 0;
   out_4451177536231554248[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4451177536231554248[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4451177536231554248[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4451177536231554248[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4451177536231554248[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4451177536231554248[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4451177536231554248[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4451177536231554248[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4451177536231554248[53] = -9.8000000000000007*dt;
   out_4451177536231554248[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4451177536231554248[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4451177536231554248[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4451177536231554248[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4451177536231554248[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4451177536231554248[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4451177536231554248[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4451177536231554248[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4451177536231554248[62] = 0;
   out_4451177536231554248[63] = 0;
   out_4451177536231554248[64] = 0;
   out_4451177536231554248[65] = 0;
   out_4451177536231554248[66] = 0;
   out_4451177536231554248[67] = 0;
   out_4451177536231554248[68] = 0;
   out_4451177536231554248[69] = 0;
   out_4451177536231554248[70] = 1;
   out_4451177536231554248[71] = 0;
   out_4451177536231554248[72] = 0;
   out_4451177536231554248[73] = 0;
   out_4451177536231554248[74] = 0;
   out_4451177536231554248[75] = 0;
   out_4451177536231554248[76] = 0;
   out_4451177536231554248[77] = 0;
   out_4451177536231554248[78] = 0;
   out_4451177536231554248[79] = 0;
   out_4451177536231554248[80] = 1;
}
void h_25(double *state, double *unused, double *out_7740290648445303998) {
   out_7740290648445303998[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4114592028682951273) {
   out_4114592028682951273[0] = 0;
   out_4114592028682951273[1] = 0;
   out_4114592028682951273[2] = 0;
   out_4114592028682951273[3] = 0;
   out_4114592028682951273[4] = 0;
   out_4114592028682951273[5] = 0;
   out_4114592028682951273[6] = 1;
   out_4114592028682951273[7] = 0;
   out_4114592028682951273[8] = 0;
}
void h_24(double *state, double *unused, double *out_5892028796349262122) {
   out_5892028796349262122[0] = state[4];
   out_5892028796349262122[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6287241627688450839) {
   out_6287241627688450839[0] = 0;
   out_6287241627688450839[1] = 0;
   out_6287241627688450839[2] = 0;
   out_6287241627688450839[3] = 0;
   out_6287241627688450839[4] = 1;
   out_6287241627688450839[5] = 0;
   out_6287241627688450839[6] = 0;
   out_6287241627688450839[7] = 0;
   out_6287241627688450839[8] = 0;
   out_6287241627688450839[9] = 0;
   out_6287241627688450839[10] = 0;
   out_6287241627688450839[11] = 0;
   out_6287241627688450839[12] = 0;
   out_6287241627688450839[13] = 0;
   out_6287241627688450839[14] = 1;
   out_6287241627688450839[15] = 0;
   out_6287241627688450839[16] = 0;
   out_6287241627688450839[17] = 0;
}
void h_30(double *state, double *unused, double *out_5319145870600894991) {
   out_5319145870600894991[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2802098312808665482) {
   out_2802098312808665482[0] = 0;
   out_2802098312808665482[1] = 0;
   out_2802098312808665482[2] = 0;
   out_2802098312808665482[3] = 0;
   out_2802098312808665482[4] = 1;
   out_2802098312808665482[5] = 0;
   out_2802098312808665482[6] = 0;
   out_2802098312808665482[7] = 0;
   out_2802098312808665482[8] = 0;
}
void h_26(double *state, double *unused, double *out_4224101575499120674) {
   out_4224101575499120674[0] = state[7];
}
void H_26(double *state, double *unused, double *out_810066058922150672) {
   out_810066058922150672[0] = 0;
   out_810066058922150672[1] = 0;
   out_810066058922150672[2] = 0;
   out_810066058922150672[3] = 0;
   out_810066058922150672[4] = 0;
   out_810066058922150672[5] = 0;
   out_810066058922150672[6] = 0;
   out_810066058922150672[7] = 1;
   out_810066058922150672[8] = 0;
}
void h_27(double *state, double *unused, double *out_441132223024443353) {
   out_441132223024443353[0] = state[3];
}
void H_27(double *state, double *unused, double *out_627335001008240571) {
   out_627335001008240571[0] = 0;
   out_627335001008240571[1] = 0;
   out_627335001008240571[2] = 0;
   out_627335001008240571[3] = 1;
   out_627335001008240571[4] = 0;
   out_627335001008240571[5] = 0;
   out_627335001008240571[6] = 0;
   out_627335001008240571[7] = 0;
   out_627335001008240571[8] = 0;
}
void h_29(double *state, double *unused, double *out_8997568804065107999) {
   out_8997568804065107999[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1086027725861310462) {
   out_1086027725861310462[0] = 0;
   out_1086027725861310462[1] = 1;
   out_1086027725861310462[2] = 0;
   out_1086027725861310462[3] = 0;
   out_1086027725861310462[4] = 0;
   out_1086027725861310462[5] = 0;
   out_1086027725861310462[6] = 0;
   out_1086027725861310462[7] = 0;
   out_1086027725861310462[8] = 0;
}
void h_28(double *state, double *unused, double *out_3045537543833141004) {
   out_3045537543833141004[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6168426742930841036) {
   out_6168426742930841036[0] = 1;
   out_6168426742930841036[1] = 0;
   out_6168426742930841036[2] = 0;
   out_6168426742930841036[3] = 0;
   out_6168426742930841036[4] = 0;
   out_6168426742930841036[5] = 0;
   out_6168426742930841036[6] = 0;
   out_6168426742930841036[7] = 0;
   out_6168426742930841036[8] = 0;
}
void h_31(double *state, double *unused, double *out_9142255106595760966) {
   out_9142255106595760966[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4083946066805990845) {
   out_4083946066805990845[0] = 0;
   out_4083946066805990845[1] = 0;
   out_4083946066805990845[2] = 0;
   out_4083946066805990845[3] = 0;
   out_4083946066805990845[4] = 0;
   out_4083946066805990845[5] = 0;
   out_4083946066805990845[6] = 0;
   out_4083946066805990845[7] = 0;
   out_4083946066805990845[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8363476756673253557) {
  err_fun(nom_x, delta_x, out_8363476756673253557);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1977797309478282067) {
  inv_err_fun(nom_x, true_x, out_1977797309478282067);
}
void car_H_mod_fun(double *state, double *out_801006497277677196) {
  H_mod_fun(state, out_801006497277677196);
}
void car_f_fun(double *state, double dt, double *out_4787359001335959165) {
  f_fun(state,  dt, out_4787359001335959165);
}
void car_F_fun(double *state, double dt, double *out_4451177536231554248) {
  F_fun(state,  dt, out_4451177536231554248);
}
void car_h_25(double *state, double *unused, double *out_7740290648445303998) {
  h_25(state, unused, out_7740290648445303998);
}
void car_H_25(double *state, double *unused, double *out_4114592028682951273) {
  H_25(state, unused, out_4114592028682951273);
}
void car_h_24(double *state, double *unused, double *out_5892028796349262122) {
  h_24(state, unused, out_5892028796349262122);
}
void car_H_24(double *state, double *unused, double *out_6287241627688450839) {
  H_24(state, unused, out_6287241627688450839);
}
void car_h_30(double *state, double *unused, double *out_5319145870600894991) {
  h_30(state, unused, out_5319145870600894991);
}
void car_H_30(double *state, double *unused, double *out_2802098312808665482) {
  H_30(state, unused, out_2802098312808665482);
}
void car_h_26(double *state, double *unused, double *out_4224101575499120674) {
  h_26(state, unused, out_4224101575499120674);
}
void car_H_26(double *state, double *unused, double *out_810066058922150672) {
  H_26(state, unused, out_810066058922150672);
}
void car_h_27(double *state, double *unused, double *out_441132223024443353) {
  h_27(state, unused, out_441132223024443353);
}
void car_H_27(double *state, double *unused, double *out_627335001008240571) {
  H_27(state, unused, out_627335001008240571);
}
void car_h_29(double *state, double *unused, double *out_8997568804065107999) {
  h_29(state, unused, out_8997568804065107999);
}
void car_H_29(double *state, double *unused, double *out_1086027725861310462) {
  H_29(state, unused, out_1086027725861310462);
}
void car_h_28(double *state, double *unused, double *out_3045537543833141004) {
  h_28(state, unused, out_3045537543833141004);
}
void car_H_28(double *state, double *unused, double *out_6168426742930841036) {
  H_28(state, unused, out_6168426742930841036);
}
void car_h_31(double *state, double *unused, double *out_9142255106595760966) {
  h_31(state, unused, out_9142255106595760966);
}
void car_H_31(double *state, double *unused, double *out_4083946066805990845) {
  H_31(state, unused, out_4083946066805990845);
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

ekf_init(car);
