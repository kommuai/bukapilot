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
void err_fun(double *nom_x, double *delta_x, double *out_9130459952118648993) {
   out_9130459952118648993[0] = delta_x[0] + nom_x[0];
   out_9130459952118648993[1] = delta_x[1] + nom_x[1];
   out_9130459952118648993[2] = delta_x[2] + nom_x[2];
   out_9130459952118648993[3] = delta_x[3] + nom_x[3];
   out_9130459952118648993[4] = delta_x[4] + nom_x[4];
   out_9130459952118648993[5] = delta_x[5] + nom_x[5];
   out_9130459952118648993[6] = delta_x[6] + nom_x[6];
   out_9130459952118648993[7] = delta_x[7] + nom_x[7];
   out_9130459952118648993[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7134613961939753672) {
   out_7134613961939753672[0] = -nom_x[0] + true_x[0];
   out_7134613961939753672[1] = -nom_x[1] + true_x[1];
   out_7134613961939753672[2] = -nom_x[2] + true_x[2];
   out_7134613961939753672[3] = -nom_x[3] + true_x[3];
   out_7134613961939753672[4] = -nom_x[4] + true_x[4];
   out_7134613961939753672[5] = -nom_x[5] + true_x[5];
   out_7134613961939753672[6] = -nom_x[6] + true_x[6];
   out_7134613961939753672[7] = -nom_x[7] + true_x[7];
   out_7134613961939753672[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5012168072766857418) {
   out_5012168072766857418[0] = 1.0;
   out_5012168072766857418[1] = 0;
   out_5012168072766857418[2] = 0;
   out_5012168072766857418[3] = 0;
   out_5012168072766857418[4] = 0;
   out_5012168072766857418[5] = 0;
   out_5012168072766857418[6] = 0;
   out_5012168072766857418[7] = 0;
   out_5012168072766857418[8] = 0;
   out_5012168072766857418[9] = 0;
   out_5012168072766857418[10] = 1.0;
   out_5012168072766857418[11] = 0;
   out_5012168072766857418[12] = 0;
   out_5012168072766857418[13] = 0;
   out_5012168072766857418[14] = 0;
   out_5012168072766857418[15] = 0;
   out_5012168072766857418[16] = 0;
   out_5012168072766857418[17] = 0;
   out_5012168072766857418[18] = 0;
   out_5012168072766857418[19] = 0;
   out_5012168072766857418[20] = 1.0;
   out_5012168072766857418[21] = 0;
   out_5012168072766857418[22] = 0;
   out_5012168072766857418[23] = 0;
   out_5012168072766857418[24] = 0;
   out_5012168072766857418[25] = 0;
   out_5012168072766857418[26] = 0;
   out_5012168072766857418[27] = 0;
   out_5012168072766857418[28] = 0;
   out_5012168072766857418[29] = 0;
   out_5012168072766857418[30] = 1.0;
   out_5012168072766857418[31] = 0;
   out_5012168072766857418[32] = 0;
   out_5012168072766857418[33] = 0;
   out_5012168072766857418[34] = 0;
   out_5012168072766857418[35] = 0;
   out_5012168072766857418[36] = 0;
   out_5012168072766857418[37] = 0;
   out_5012168072766857418[38] = 0;
   out_5012168072766857418[39] = 0;
   out_5012168072766857418[40] = 1.0;
   out_5012168072766857418[41] = 0;
   out_5012168072766857418[42] = 0;
   out_5012168072766857418[43] = 0;
   out_5012168072766857418[44] = 0;
   out_5012168072766857418[45] = 0;
   out_5012168072766857418[46] = 0;
   out_5012168072766857418[47] = 0;
   out_5012168072766857418[48] = 0;
   out_5012168072766857418[49] = 0;
   out_5012168072766857418[50] = 1.0;
   out_5012168072766857418[51] = 0;
   out_5012168072766857418[52] = 0;
   out_5012168072766857418[53] = 0;
   out_5012168072766857418[54] = 0;
   out_5012168072766857418[55] = 0;
   out_5012168072766857418[56] = 0;
   out_5012168072766857418[57] = 0;
   out_5012168072766857418[58] = 0;
   out_5012168072766857418[59] = 0;
   out_5012168072766857418[60] = 1.0;
   out_5012168072766857418[61] = 0;
   out_5012168072766857418[62] = 0;
   out_5012168072766857418[63] = 0;
   out_5012168072766857418[64] = 0;
   out_5012168072766857418[65] = 0;
   out_5012168072766857418[66] = 0;
   out_5012168072766857418[67] = 0;
   out_5012168072766857418[68] = 0;
   out_5012168072766857418[69] = 0;
   out_5012168072766857418[70] = 1.0;
   out_5012168072766857418[71] = 0;
   out_5012168072766857418[72] = 0;
   out_5012168072766857418[73] = 0;
   out_5012168072766857418[74] = 0;
   out_5012168072766857418[75] = 0;
   out_5012168072766857418[76] = 0;
   out_5012168072766857418[77] = 0;
   out_5012168072766857418[78] = 0;
   out_5012168072766857418[79] = 0;
   out_5012168072766857418[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_9200361991106576411) {
   out_9200361991106576411[0] = state[0];
   out_9200361991106576411[1] = state[1];
   out_9200361991106576411[2] = state[2];
   out_9200361991106576411[3] = state[3];
   out_9200361991106576411[4] = state[4];
   out_9200361991106576411[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_9200361991106576411[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_9200361991106576411[7] = state[7];
   out_9200361991106576411[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6355094581427914180) {
   out_6355094581427914180[0] = 1;
   out_6355094581427914180[1] = 0;
   out_6355094581427914180[2] = 0;
   out_6355094581427914180[3] = 0;
   out_6355094581427914180[4] = 0;
   out_6355094581427914180[5] = 0;
   out_6355094581427914180[6] = 0;
   out_6355094581427914180[7] = 0;
   out_6355094581427914180[8] = 0;
   out_6355094581427914180[9] = 0;
   out_6355094581427914180[10] = 1;
   out_6355094581427914180[11] = 0;
   out_6355094581427914180[12] = 0;
   out_6355094581427914180[13] = 0;
   out_6355094581427914180[14] = 0;
   out_6355094581427914180[15] = 0;
   out_6355094581427914180[16] = 0;
   out_6355094581427914180[17] = 0;
   out_6355094581427914180[18] = 0;
   out_6355094581427914180[19] = 0;
   out_6355094581427914180[20] = 1;
   out_6355094581427914180[21] = 0;
   out_6355094581427914180[22] = 0;
   out_6355094581427914180[23] = 0;
   out_6355094581427914180[24] = 0;
   out_6355094581427914180[25] = 0;
   out_6355094581427914180[26] = 0;
   out_6355094581427914180[27] = 0;
   out_6355094581427914180[28] = 0;
   out_6355094581427914180[29] = 0;
   out_6355094581427914180[30] = 1;
   out_6355094581427914180[31] = 0;
   out_6355094581427914180[32] = 0;
   out_6355094581427914180[33] = 0;
   out_6355094581427914180[34] = 0;
   out_6355094581427914180[35] = 0;
   out_6355094581427914180[36] = 0;
   out_6355094581427914180[37] = 0;
   out_6355094581427914180[38] = 0;
   out_6355094581427914180[39] = 0;
   out_6355094581427914180[40] = 1;
   out_6355094581427914180[41] = 0;
   out_6355094581427914180[42] = 0;
   out_6355094581427914180[43] = 0;
   out_6355094581427914180[44] = 0;
   out_6355094581427914180[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6355094581427914180[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6355094581427914180[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6355094581427914180[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6355094581427914180[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6355094581427914180[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6355094581427914180[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6355094581427914180[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6355094581427914180[53] = -9.8000000000000007*dt;
   out_6355094581427914180[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6355094581427914180[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6355094581427914180[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6355094581427914180[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6355094581427914180[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6355094581427914180[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6355094581427914180[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6355094581427914180[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6355094581427914180[62] = 0;
   out_6355094581427914180[63] = 0;
   out_6355094581427914180[64] = 0;
   out_6355094581427914180[65] = 0;
   out_6355094581427914180[66] = 0;
   out_6355094581427914180[67] = 0;
   out_6355094581427914180[68] = 0;
   out_6355094581427914180[69] = 0;
   out_6355094581427914180[70] = 1;
   out_6355094581427914180[71] = 0;
   out_6355094581427914180[72] = 0;
   out_6355094581427914180[73] = 0;
   out_6355094581427914180[74] = 0;
   out_6355094581427914180[75] = 0;
   out_6355094581427914180[76] = 0;
   out_6355094581427914180[77] = 0;
   out_6355094581427914180[78] = 0;
   out_6355094581427914180[79] = 0;
   out_6355094581427914180[80] = 1;
}
void h_25(double *state, double *unused, double *out_3279746131878314915) {
   out_3279746131878314915[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4695936252697041279) {
   out_4695936252697041279[0] = 0;
   out_4695936252697041279[1] = 0;
   out_4695936252697041279[2] = 0;
   out_4695936252697041279[3] = 0;
   out_4695936252697041279[4] = 0;
   out_4695936252697041279[5] = 0;
   out_4695936252697041279[6] = 1;
   out_4695936252697041279[7] = 0;
   out_4695936252697041279[8] = 0;
}
void h_24(double *state, double *unused, double *out_6457534050077103686) {
   out_6457534050077103686[0] = state[4];
   out_6457534050077103686[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2523286653691541713) {
   out_2523286653691541713[0] = 0;
   out_2523286653691541713[1] = 0;
   out_2523286653691541713[2] = 0;
   out_2523286653691541713[3] = 0;
   out_2523286653691541713[4] = 1;
   out_2523286653691541713[5] = 0;
   out_2523286653691541713[6] = 0;
   out_2523286653691541713[7] = 0;
   out_2523286653691541713[8] = 0;
   out_2523286653691541713[9] = 0;
   out_2523286653691541713[10] = 0;
   out_2523286653691541713[11] = 0;
   out_2523286653691541713[12] = 0;
   out_2523286653691541713[13] = 0;
   out_2523286653691541713[14] = 1;
   out_2523286653691541713[15] = 0;
   out_2523286653691541713[16] = 0;
   out_2523286653691541713[17] = 0;
}
void h_30(double *state, double *unused, double *out_5242583122295845460) {
   out_5242583122295845460[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7214269211204289906) {
   out_7214269211204289906[0] = 0;
   out_7214269211204289906[1] = 0;
   out_7214269211204289906[2] = 0;
   out_7214269211204289906[3] = 0;
   out_7214269211204289906[4] = 1;
   out_7214269211204289906[5] = 0;
   out_7214269211204289906[6] = 0;
   out_7214269211204289906[7] = 0;
   out_7214269211204289906[8] = 0;
}
void h_26(double *state, double *unused, double *out_6948336229454658902) {
   out_6948336229454658902[0] = state[7];
}
void H_26(double *state, double *unused, double *out_954432933822985055) {
   out_954432933822985055[0] = 0;
   out_954432933822985055[1] = 0;
   out_954432933822985055[2] = 0;
   out_954432933822985055[3] = 0;
   out_954432933822985055[4] = 0;
   out_954432933822985055[5] = 0;
   out_954432933822985055[6] = 0;
   out_954432933822985055[7] = 1;
   out_954432933822985055[8] = 0;
}
void h_27(double *state, double *unused, double *out_8123773559799036180) {
   out_8123773559799036180[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5039505899403864995) {
   out_5039505899403864995[0] = 0;
   out_5039505899403864995[1] = 0;
   out_5039505899403864995[2] = 0;
   out_5039505899403864995[3] = 1;
   out_5039505899403864995[4] = 0;
   out_5039505899403864995[5] = 0;
   out_5039505899403864995[6] = 0;
   out_5039505899403864995[7] = 0;
   out_5039505899403864995[8] = 0;
}
void h_29(double *state, double *unused, double *out_5127380634965125020) {
   out_5127380634965125020[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7724500555518682090) {
   out_7724500555518682090[0] = 0;
   out_7724500555518682090[1] = 1;
   out_7724500555518682090[2] = 0;
   out_7724500555518682090[3] = 0;
   out_7724500555518682090[4] = 0;
   out_7724500555518682090[5] = 0;
   out_7724500555518682090[6] = 0;
   out_7724500555518682090[7] = 0;
   out_7724500555518682090[8] = 0;
}
void h_28(double *state, double *unused, double *out_3122100292138190535) {
   out_3122100292138190535[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2642101538449151516) {
   out_2642101538449151516[0] = 1;
   out_2642101538449151516[1] = 0;
   out_2642101538449151516[2] = 0;
   out_2642101538449151516[3] = 0;
   out_2642101538449151516[4] = 0;
   out_2642101538449151516[5] = 0;
   out_2642101538449151516[6] = 0;
   out_2642101538449151516[7] = 0;
   out_2642101538449151516[8] = 0;
}
void h_31(double *state, double *unused, double *out_1188058135829320558) {
   out_1188058135829320558[0] = state[8];
}
void H_31(double *state, double *unused, double *out_328224831589633579) {
   out_328224831589633579[0] = 0;
   out_328224831589633579[1] = 0;
   out_328224831589633579[2] = 0;
   out_328224831589633579[3] = 0;
   out_328224831589633579[4] = 0;
   out_328224831589633579[5] = 0;
   out_328224831589633579[6] = 0;
   out_328224831589633579[7] = 0;
   out_328224831589633579[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_9130459952118648993) {
  err_fun(nom_x, delta_x, out_9130459952118648993);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7134613961939753672) {
  inv_err_fun(nom_x, true_x, out_7134613961939753672);
}
void car_H_mod_fun(double *state, double *out_5012168072766857418) {
  H_mod_fun(state, out_5012168072766857418);
}
void car_f_fun(double *state, double dt, double *out_9200361991106576411) {
  f_fun(state,  dt, out_9200361991106576411);
}
void car_F_fun(double *state, double dt, double *out_6355094581427914180) {
  F_fun(state,  dt, out_6355094581427914180);
}
void car_h_25(double *state, double *unused, double *out_3279746131878314915) {
  h_25(state, unused, out_3279746131878314915);
}
void car_H_25(double *state, double *unused, double *out_4695936252697041279) {
  H_25(state, unused, out_4695936252697041279);
}
void car_h_24(double *state, double *unused, double *out_6457534050077103686) {
  h_24(state, unused, out_6457534050077103686);
}
void car_H_24(double *state, double *unused, double *out_2523286653691541713) {
  H_24(state, unused, out_2523286653691541713);
}
void car_h_30(double *state, double *unused, double *out_5242583122295845460) {
  h_30(state, unused, out_5242583122295845460);
}
void car_H_30(double *state, double *unused, double *out_7214269211204289906) {
  H_30(state, unused, out_7214269211204289906);
}
void car_h_26(double *state, double *unused, double *out_6948336229454658902) {
  h_26(state, unused, out_6948336229454658902);
}
void car_H_26(double *state, double *unused, double *out_954432933822985055) {
  H_26(state, unused, out_954432933822985055);
}
void car_h_27(double *state, double *unused, double *out_8123773559799036180) {
  h_27(state, unused, out_8123773559799036180);
}
void car_H_27(double *state, double *unused, double *out_5039505899403864995) {
  H_27(state, unused, out_5039505899403864995);
}
void car_h_29(double *state, double *unused, double *out_5127380634965125020) {
  h_29(state, unused, out_5127380634965125020);
}
void car_H_29(double *state, double *unused, double *out_7724500555518682090) {
  H_29(state, unused, out_7724500555518682090);
}
void car_h_28(double *state, double *unused, double *out_3122100292138190535) {
  h_28(state, unused, out_3122100292138190535);
}
void car_H_28(double *state, double *unused, double *out_2642101538449151516) {
  H_28(state, unused, out_2642101538449151516);
}
void car_h_31(double *state, double *unused, double *out_1188058135829320558) {
  h_31(state, unused, out_1188058135829320558);
}
void car_H_31(double *state, double *unused, double *out_328224831589633579) {
  H_31(state, unused, out_328224831589633579);
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
