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
void err_fun(double *nom_x, double *delta_x, double *out_4427157744248383102) {
   out_4427157744248383102[0] = delta_x[0] + nom_x[0];
   out_4427157744248383102[1] = delta_x[1] + nom_x[1];
   out_4427157744248383102[2] = delta_x[2] + nom_x[2];
   out_4427157744248383102[3] = delta_x[3] + nom_x[3];
   out_4427157744248383102[4] = delta_x[4] + nom_x[4];
   out_4427157744248383102[5] = delta_x[5] + nom_x[5];
   out_4427157744248383102[6] = delta_x[6] + nom_x[6];
   out_4427157744248383102[7] = delta_x[7] + nom_x[7];
   out_4427157744248383102[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7045651039375787549) {
   out_7045651039375787549[0] = -nom_x[0] + true_x[0];
   out_7045651039375787549[1] = -nom_x[1] + true_x[1];
   out_7045651039375787549[2] = -nom_x[2] + true_x[2];
   out_7045651039375787549[3] = -nom_x[3] + true_x[3];
   out_7045651039375787549[4] = -nom_x[4] + true_x[4];
   out_7045651039375787549[5] = -nom_x[5] + true_x[5];
   out_7045651039375787549[6] = -nom_x[6] + true_x[6];
   out_7045651039375787549[7] = -nom_x[7] + true_x[7];
   out_7045651039375787549[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6801501052206484974) {
   out_6801501052206484974[0] = 1.0;
   out_6801501052206484974[1] = 0;
   out_6801501052206484974[2] = 0;
   out_6801501052206484974[3] = 0;
   out_6801501052206484974[4] = 0;
   out_6801501052206484974[5] = 0;
   out_6801501052206484974[6] = 0;
   out_6801501052206484974[7] = 0;
   out_6801501052206484974[8] = 0;
   out_6801501052206484974[9] = 0;
   out_6801501052206484974[10] = 1.0;
   out_6801501052206484974[11] = 0;
   out_6801501052206484974[12] = 0;
   out_6801501052206484974[13] = 0;
   out_6801501052206484974[14] = 0;
   out_6801501052206484974[15] = 0;
   out_6801501052206484974[16] = 0;
   out_6801501052206484974[17] = 0;
   out_6801501052206484974[18] = 0;
   out_6801501052206484974[19] = 0;
   out_6801501052206484974[20] = 1.0;
   out_6801501052206484974[21] = 0;
   out_6801501052206484974[22] = 0;
   out_6801501052206484974[23] = 0;
   out_6801501052206484974[24] = 0;
   out_6801501052206484974[25] = 0;
   out_6801501052206484974[26] = 0;
   out_6801501052206484974[27] = 0;
   out_6801501052206484974[28] = 0;
   out_6801501052206484974[29] = 0;
   out_6801501052206484974[30] = 1.0;
   out_6801501052206484974[31] = 0;
   out_6801501052206484974[32] = 0;
   out_6801501052206484974[33] = 0;
   out_6801501052206484974[34] = 0;
   out_6801501052206484974[35] = 0;
   out_6801501052206484974[36] = 0;
   out_6801501052206484974[37] = 0;
   out_6801501052206484974[38] = 0;
   out_6801501052206484974[39] = 0;
   out_6801501052206484974[40] = 1.0;
   out_6801501052206484974[41] = 0;
   out_6801501052206484974[42] = 0;
   out_6801501052206484974[43] = 0;
   out_6801501052206484974[44] = 0;
   out_6801501052206484974[45] = 0;
   out_6801501052206484974[46] = 0;
   out_6801501052206484974[47] = 0;
   out_6801501052206484974[48] = 0;
   out_6801501052206484974[49] = 0;
   out_6801501052206484974[50] = 1.0;
   out_6801501052206484974[51] = 0;
   out_6801501052206484974[52] = 0;
   out_6801501052206484974[53] = 0;
   out_6801501052206484974[54] = 0;
   out_6801501052206484974[55] = 0;
   out_6801501052206484974[56] = 0;
   out_6801501052206484974[57] = 0;
   out_6801501052206484974[58] = 0;
   out_6801501052206484974[59] = 0;
   out_6801501052206484974[60] = 1.0;
   out_6801501052206484974[61] = 0;
   out_6801501052206484974[62] = 0;
   out_6801501052206484974[63] = 0;
   out_6801501052206484974[64] = 0;
   out_6801501052206484974[65] = 0;
   out_6801501052206484974[66] = 0;
   out_6801501052206484974[67] = 0;
   out_6801501052206484974[68] = 0;
   out_6801501052206484974[69] = 0;
   out_6801501052206484974[70] = 1.0;
   out_6801501052206484974[71] = 0;
   out_6801501052206484974[72] = 0;
   out_6801501052206484974[73] = 0;
   out_6801501052206484974[74] = 0;
   out_6801501052206484974[75] = 0;
   out_6801501052206484974[76] = 0;
   out_6801501052206484974[77] = 0;
   out_6801501052206484974[78] = 0;
   out_6801501052206484974[79] = 0;
   out_6801501052206484974[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3034258192735895458) {
   out_3034258192735895458[0] = state[0];
   out_3034258192735895458[1] = state[1];
   out_3034258192735895458[2] = state[2];
   out_3034258192735895458[3] = state[3];
   out_3034258192735895458[4] = state[4];
   out_3034258192735895458[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3034258192735895458[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3034258192735895458[7] = state[7];
   out_3034258192735895458[8] = state[8];
}
void F_fun(double *state, double dt, double *out_742065270288018262) {
   out_742065270288018262[0] = 1;
   out_742065270288018262[1] = 0;
   out_742065270288018262[2] = 0;
   out_742065270288018262[3] = 0;
   out_742065270288018262[4] = 0;
   out_742065270288018262[5] = 0;
   out_742065270288018262[6] = 0;
   out_742065270288018262[7] = 0;
   out_742065270288018262[8] = 0;
   out_742065270288018262[9] = 0;
   out_742065270288018262[10] = 1;
   out_742065270288018262[11] = 0;
   out_742065270288018262[12] = 0;
   out_742065270288018262[13] = 0;
   out_742065270288018262[14] = 0;
   out_742065270288018262[15] = 0;
   out_742065270288018262[16] = 0;
   out_742065270288018262[17] = 0;
   out_742065270288018262[18] = 0;
   out_742065270288018262[19] = 0;
   out_742065270288018262[20] = 1;
   out_742065270288018262[21] = 0;
   out_742065270288018262[22] = 0;
   out_742065270288018262[23] = 0;
   out_742065270288018262[24] = 0;
   out_742065270288018262[25] = 0;
   out_742065270288018262[26] = 0;
   out_742065270288018262[27] = 0;
   out_742065270288018262[28] = 0;
   out_742065270288018262[29] = 0;
   out_742065270288018262[30] = 1;
   out_742065270288018262[31] = 0;
   out_742065270288018262[32] = 0;
   out_742065270288018262[33] = 0;
   out_742065270288018262[34] = 0;
   out_742065270288018262[35] = 0;
   out_742065270288018262[36] = 0;
   out_742065270288018262[37] = 0;
   out_742065270288018262[38] = 0;
   out_742065270288018262[39] = 0;
   out_742065270288018262[40] = 1;
   out_742065270288018262[41] = 0;
   out_742065270288018262[42] = 0;
   out_742065270288018262[43] = 0;
   out_742065270288018262[44] = 0;
   out_742065270288018262[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_742065270288018262[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_742065270288018262[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_742065270288018262[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_742065270288018262[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_742065270288018262[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_742065270288018262[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_742065270288018262[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_742065270288018262[53] = -9.8000000000000007*dt;
   out_742065270288018262[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_742065270288018262[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_742065270288018262[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_742065270288018262[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_742065270288018262[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_742065270288018262[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_742065270288018262[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_742065270288018262[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_742065270288018262[62] = 0;
   out_742065270288018262[63] = 0;
   out_742065270288018262[64] = 0;
   out_742065270288018262[65] = 0;
   out_742065270288018262[66] = 0;
   out_742065270288018262[67] = 0;
   out_742065270288018262[68] = 0;
   out_742065270288018262[69] = 0;
   out_742065270288018262[70] = 1;
   out_742065270288018262[71] = 0;
   out_742065270288018262[72] = 0;
   out_742065270288018262[73] = 0;
   out_742065270288018262[74] = 0;
   out_742065270288018262[75] = 0;
   out_742065270288018262[76] = 0;
   out_742065270288018262[77] = 0;
   out_742065270288018262[78] = 0;
   out_742065270288018262[79] = 0;
   out_742065270288018262[80] = 1;
}
void h_25(double *state, double *unused, double *out_3481936820668391358) {
   out_3481936820668391358[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8705174448933309154) {
   out_8705174448933309154[0] = 0;
   out_8705174448933309154[1] = 0;
   out_8705174448933309154[2] = 0;
   out_8705174448933309154[3] = 0;
   out_8705174448933309154[4] = 0;
   out_8705174448933309154[5] = 0;
   out_8705174448933309154[6] = 1;
   out_8705174448933309154[7] = 0;
   out_8705174448933309154[8] = 0;
}
void h_24(double *state, double *unused, double *out_3854856096366595906) {
   out_3854856096366595906[0] = state[4];
   out_3854856096366595906[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7568920025770742896) {
   out_7568920025770742896[0] = 0;
   out_7568920025770742896[1] = 0;
   out_7568920025770742896[2] = 0;
   out_7568920025770742896[3] = 0;
   out_7568920025770742896[4] = 1;
   out_7568920025770742896[5] = 0;
   out_7568920025770742896[6] = 0;
   out_7568920025770742896[7] = 0;
   out_7568920025770742896[8] = 0;
   out_7568920025770742896[9] = 0;
   out_7568920025770742896[10] = 0;
   out_7568920025770742896[11] = 0;
   out_7568920025770742896[12] = 0;
   out_7568920025770742896[13] = 0;
   out_7568920025770742896[14] = 1;
   out_7568920025770742896[15] = 0;
   out_7568920025770742896[16] = 0;
   out_7568920025770742896[17] = 0;
}
void h_30(double *state, double *unused, double *out_7201541996220487817) {
   out_7201541996220487817[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6186841490426060527) {
   out_6186841490426060527[0] = 0;
   out_6186841490426060527[1] = 0;
   out_6186841490426060527[2] = 0;
   out_6186841490426060527[3] = 0;
   out_6186841490426060527[4] = 1;
   out_6186841490426060527[5] = 0;
   out_6186841490426060527[6] = 0;
   out_6186841490426060527[7] = 0;
   out_6186841490426060527[8] = 0;
}
void h_26(double *state, double *unused, double *out_8064484404317752036) {
   out_8064484404317752036[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5400648479172508553) {
   out_5400648479172508553[0] = 0;
   out_5400648479172508553[1] = 0;
   out_5400648479172508553[2] = 0;
   out_5400648479172508553[3] = 0;
   out_5400648479172508553[4] = 0;
   out_5400648479172508553[5] = 0;
   out_5400648479172508553[6] = 0;
   out_5400648479172508553[7] = 1;
   out_5400648479172508553[8] = 0;
}
void h_27(double *state, double *unused, double *out_1519736446992454287) {
   out_1519736446992454287[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3963247419242117310) {
   out_3963247419242117310[0] = 0;
   out_3963247419242117310[1] = 0;
   out_3963247419242117310[2] = 0;
   out_3963247419242117310[3] = 1;
   out_3963247419242117310[4] = 0;
   out_3963247419242117310[5] = 0;
   out_3963247419242117310[6] = 0;
   out_3963247419242117310[7] = 0;
   out_3963247419242117310[8] = 0;
}
void h_29(double *state, double *unused, double *out_7804857880896965334) {
   out_7804857880896965334[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5676610146111668343) {
   out_5676610146111668343[0] = 0;
   out_5676610146111668343[1] = 1;
   out_5676610146111668343[2] = 0;
   out_5676610146111668343[3] = 0;
   out_5676610146111668343[4] = 0;
   out_5676610146111668343[5] = 0;
   out_5676610146111668343[6] = 0;
   out_5676610146111668343[7] = 0;
   out_5676610146111668343[8] = 0;
}
void h_28(double *state, double *unused, double *out_8209170706848405003) {
   out_8209170706848405003[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7687734910528352699) {
   out_7687734910528352699[0] = 1;
   out_7687734910528352699[1] = 0;
   out_7687734910528352699[2] = 0;
   out_7687734910528352699[3] = 0;
   out_7687734910528352699[4] = 0;
   out_7687734910528352699[5] = 0;
   out_7687734910528352699[6] = 0;
   out_7687734910528352699[7] = 0;
   out_7687734910528352699[8] = 0;
}
void h_31(double *state, double *unused, double *out_5655780464015459824) {
   out_5655780464015459824[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5373858203668834762) {
   out_5373858203668834762[0] = 0;
   out_5373858203668834762[1] = 0;
   out_5373858203668834762[2] = 0;
   out_5373858203668834762[3] = 0;
   out_5373858203668834762[4] = 0;
   out_5373858203668834762[5] = 0;
   out_5373858203668834762[6] = 0;
   out_5373858203668834762[7] = 0;
   out_5373858203668834762[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4427157744248383102) {
  err_fun(nom_x, delta_x, out_4427157744248383102);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7045651039375787549) {
  inv_err_fun(nom_x, true_x, out_7045651039375787549);
}
void car_H_mod_fun(double *state, double *out_6801501052206484974) {
  H_mod_fun(state, out_6801501052206484974);
}
void car_f_fun(double *state, double dt, double *out_3034258192735895458) {
  f_fun(state,  dt, out_3034258192735895458);
}
void car_F_fun(double *state, double dt, double *out_742065270288018262) {
  F_fun(state,  dt, out_742065270288018262);
}
void car_h_25(double *state, double *unused, double *out_3481936820668391358) {
  h_25(state, unused, out_3481936820668391358);
}
void car_H_25(double *state, double *unused, double *out_8705174448933309154) {
  H_25(state, unused, out_8705174448933309154);
}
void car_h_24(double *state, double *unused, double *out_3854856096366595906) {
  h_24(state, unused, out_3854856096366595906);
}
void car_H_24(double *state, double *unused, double *out_7568920025770742896) {
  H_24(state, unused, out_7568920025770742896);
}
void car_h_30(double *state, double *unused, double *out_7201541996220487817) {
  h_30(state, unused, out_7201541996220487817);
}
void car_H_30(double *state, double *unused, double *out_6186841490426060527) {
  H_30(state, unused, out_6186841490426060527);
}
void car_h_26(double *state, double *unused, double *out_8064484404317752036) {
  h_26(state, unused, out_8064484404317752036);
}
void car_H_26(double *state, double *unused, double *out_5400648479172508553) {
  H_26(state, unused, out_5400648479172508553);
}
void car_h_27(double *state, double *unused, double *out_1519736446992454287) {
  h_27(state, unused, out_1519736446992454287);
}
void car_H_27(double *state, double *unused, double *out_3963247419242117310) {
  H_27(state, unused, out_3963247419242117310);
}
void car_h_29(double *state, double *unused, double *out_7804857880896965334) {
  h_29(state, unused, out_7804857880896965334);
}
void car_H_29(double *state, double *unused, double *out_5676610146111668343) {
  H_29(state, unused, out_5676610146111668343);
}
void car_h_28(double *state, double *unused, double *out_8209170706848405003) {
  h_28(state, unused, out_8209170706848405003);
}
void car_H_28(double *state, double *unused, double *out_7687734910528352699) {
  H_28(state, unused, out_7687734910528352699);
}
void car_h_31(double *state, double *unused, double *out_5655780464015459824) {
  h_31(state, unused, out_5655780464015459824);
}
void car_H_31(double *state, double *unused, double *out_5373858203668834762) {
  H_31(state, unused, out_5373858203668834762);
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
