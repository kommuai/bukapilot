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
void err_fun(double *nom_x, double *delta_x, double *out_5652973485134955936) {
   out_5652973485134955936[0] = delta_x[0] + nom_x[0];
   out_5652973485134955936[1] = delta_x[1] + nom_x[1];
   out_5652973485134955936[2] = delta_x[2] + nom_x[2];
   out_5652973485134955936[3] = delta_x[3] + nom_x[3];
   out_5652973485134955936[4] = delta_x[4] + nom_x[4];
   out_5652973485134955936[5] = delta_x[5] + nom_x[5];
   out_5652973485134955936[6] = delta_x[6] + nom_x[6];
   out_5652973485134955936[7] = delta_x[7] + nom_x[7];
   out_5652973485134955936[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5828134910451363179) {
   out_5828134910451363179[0] = -nom_x[0] + true_x[0];
   out_5828134910451363179[1] = -nom_x[1] + true_x[1];
   out_5828134910451363179[2] = -nom_x[2] + true_x[2];
   out_5828134910451363179[3] = -nom_x[3] + true_x[3];
   out_5828134910451363179[4] = -nom_x[4] + true_x[4];
   out_5828134910451363179[5] = -nom_x[5] + true_x[5];
   out_5828134910451363179[6] = -nom_x[6] + true_x[6];
   out_5828134910451363179[7] = -nom_x[7] + true_x[7];
   out_5828134910451363179[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5063456026301800763) {
   out_5063456026301800763[0] = 1.0;
   out_5063456026301800763[1] = 0;
   out_5063456026301800763[2] = 0;
   out_5063456026301800763[3] = 0;
   out_5063456026301800763[4] = 0;
   out_5063456026301800763[5] = 0;
   out_5063456026301800763[6] = 0;
   out_5063456026301800763[7] = 0;
   out_5063456026301800763[8] = 0;
   out_5063456026301800763[9] = 0;
   out_5063456026301800763[10] = 1.0;
   out_5063456026301800763[11] = 0;
   out_5063456026301800763[12] = 0;
   out_5063456026301800763[13] = 0;
   out_5063456026301800763[14] = 0;
   out_5063456026301800763[15] = 0;
   out_5063456026301800763[16] = 0;
   out_5063456026301800763[17] = 0;
   out_5063456026301800763[18] = 0;
   out_5063456026301800763[19] = 0;
   out_5063456026301800763[20] = 1.0;
   out_5063456026301800763[21] = 0;
   out_5063456026301800763[22] = 0;
   out_5063456026301800763[23] = 0;
   out_5063456026301800763[24] = 0;
   out_5063456026301800763[25] = 0;
   out_5063456026301800763[26] = 0;
   out_5063456026301800763[27] = 0;
   out_5063456026301800763[28] = 0;
   out_5063456026301800763[29] = 0;
   out_5063456026301800763[30] = 1.0;
   out_5063456026301800763[31] = 0;
   out_5063456026301800763[32] = 0;
   out_5063456026301800763[33] = 0;
   out_5063456026301800763[34] = 0;
   out_5063456026301800763[35] = 0;
   out_5063456026301800763[36] = 0;
   out_5063456026301800763[37] = 0;
   out_5063456026301800763[38] = 0;
   out_5063456026301800763[39] = 0;
   out_5063456026301800763[40] = 1.0;
   out_5063456026301800763[41] = 0;
   out_5063456026301800763[42] = 0;
   out_5063456026301800763[43] = 0;
   out_5063456026301800763[44] = 0;
   out_5063456026301800763[45] = 0;
   out_5063456026301800763[46] = 0;
   out_5063456026301800763[47] = 0;
   out_5063456026301800763[48] = 0;
   out_5063456026301800763[49] = 0;
   out_5063456026301800763[50] = 1.0;
   out_5063456026301800763[51] = 0;
   out_5063456026301800763[52] = 0;
   out_5063456026301800763[53] = 0;
   out_5063456026301800763[54] = 0;
   out_5063456026301800763[55] = 0;
   out_5063456026301800763[56] = 0;
   out_5063456026301800763[57] = 0;
   out_5063456026301800763[58] = 0;
   out_5063456026301800763[59] = 0;
   out_5063456026301800763[60] = 1.0;
   out_5063456026301800763[61] = 0;
   out_5063456026301800763[62] = 0;
   out_5063456026301800763[63] = 0;
   out_5063456026301800763[64] = 0;
   out_5063456026301800763[65] = 0;
   out_5063456026301800763[66] = 0;
   out_5063456026301800763[67] = 0;
   out_5063456026301800763[68] = 0;
   out_5063456026301800763[69] = 0;
   out_5063456026301800763[70] = 1.0;
   out_5063456026301800763[71] = 0;
   out_5063456026301800763[72] = 0;
   out_5063456026301800763[73] = 0;
   out_5063456026301800763[74] = 0;
   out_5063456026301800763[75] = 0;
   out_5063456026301800763[76] = 0;
   out_5063456026301800763[77] = 0;
   out_5063456026301800763[78] = 0;
   out_5063456026301800763[79] = 0;
   out_5063456026301800763[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_9008031268712533140) {
   out_9008031268712533140[0] = state[0];
   out_9008031268712533140[1] = state[1];
   out_9008031268712533140[2] = state[2];
   out_9008031268712533140[3] = state[3];
   out_9008031268712533140[4] = state[4];
   out_9008031268712533140[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_9008031268712533140[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_9008031268712533140[7] = state[7];
   out_9008031268712533140[8] = state[8];
}
void F_fun(double *state, double dt, double *out_399800080831945283) {
   out_399800080831945283[0] = 1;
   out_399800080831945283[1] = 0;
   out_399800080831945283[2] = 0;
   out_399800080831945283[3] = 0;
   out_399800080831945283[4] = 0;
   out_399800080831945283[5] = 0;
   out_399800080831945283[6] = 0;
   out_399800080831945283[7] = 0;
   out_399800080831945283[8] = 0;
   out_399800080831945283[9] = 0;
   out_399800080831945283[10] = 1;
   out_399800080831945283[11] = 0;
   out_399800080831945283[12] = 0;
   out_399800080831945283[13] = 0;
   out_399800080831945283[14] = 0;
   out_399800080831945283[15] = 0;
   out_399800080831945283[16] = 0;
   out_399800080831945283[17] = 0;
   out_399800080831945283[18] = 0;
   out_399800080831945283[19] = 0;
   out_399800080831945283[20] = 1;
   out_399800080831945283[21] = 0;
   out_399800080831945283[22] = 0;
   out_399800080831945283[23] = 0;
   out_399800080831945283[24] = 0;
   out_399800080831945283[25] = 0;
   out_399800080831945283[26] = 0;
   out_399800080831945283[27] = 0;
   out_399800080831945283[28] = 0;
   out_399800080831945283[29] = 0;
   out_399800080831945283[30] = 1;
   out_399800080831945283[31] = 0;
   out_399800080831945283[32] = 0;
   out_399800080831945283[33] = 0;
   out_399800080831945283[34] = 0;
   out_399800080831945283[35] = 0;
   out_399800080831945283[36] = 0;
   out_399800080831945283[37] = 0;
   out_399800080831945283[38] = 0;
   out_399800080831945283[39] = 0;
   out_399800080831945283[40] = 1;
   out_399800080831945283[41] = 0;
   out_399800080831945283[42] = 0;
   out_399800080831945283[43] = 0;
   out_399800080831945283[44] = 0;
   out_399800080831945283[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_399800080831945283[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_399800080831945283[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_399800080831945283[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_399800080831945283[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_399800080831945283[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_399800080831945283[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_399800080831945283[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_399800080831945283[53] = -9.8000000000000007*dt;
   out_399800080831945283[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_399800080831945283[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_399800080831945283[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_399800080831945283[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_399800080831945283[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_399800080831945283[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_399800080831945283[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_399800080831945283[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_399800080831945283[62] = 0;
   out_399800080831945283[63] = 0;
   out_399800080831945283[64] = 0;
   out_399800080831945283[65] = 0;
   out_399800080831945283[66] = 0;
   out_399800080831945283[67] = 0;
   out_399800080831945283[68] = 0;
   out_399800080831945283[69] = 0;
   out_399800080831945283[70] = 1;
   out_399800080831945283[71] = 0;
   out_399800080831945283[72] = 0;
   out_399800080831945283[73] = 0;
   out_399800080831945283[74] = 0;
   out_399800080831945283[75] = 0;
   out_399800080831945283[76] = 0;
   out_399800080831945283[77] = 0;
   out_399800080831945283[78] = 0;
   out_399800080831945283[79] = 0;
   out_399800080831945283[80] = 1;
}
void h_25(double *state, double *unused, double *out_8429616611094014799) {
   out_8429616611094014799[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3751244995790767177) {
   out_3751244995790767177[0] = 0;
   out_3751244995790767177[1] = 0;
   out_3751244995790767177[2] = 0;
   out_3751244995790767177[3] = 0;
   out_3751244995790767177[4] = 0;
   out_3751244995790767177[5] = 0;
   out_3751244995790767177[6] = 1;
   out_3751244995790767177[7] = 0;
   out_3751244995790767177[8] = 0;
}
void h_24(double *state, double *unused, double *out_6860572812747741310) {
   out_6860572812747741310[0] = state[4];
   out_6860572812747741310[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7752957074421275810) {
   out_7752957074421275810[0] = 0;
   out_7752957074421275810[1] = 0;
   out_7752957074421275810[2] = 0;
   out_7752957074421275810[3] = 0;
   out_7752957074421275810[4] = 1;
   out_7752957074421275810[5] = 0;
   out_7752957074421275810[6] = 0;
   out_7752957074421275810[7] = 0;
   out_7752957074421275810[8] = 0;
   out_7752957074421275810[9] = 0;
   out_7752957074421275810[10] = 0;
   out_7752957074421275810[11] = 0;
   out_7752957074421275810[12] = 0;
   out_7752957074421275810[13] = 0;
   out_7752957074421275810[14] = 1;
   out_7752957074421275810[15] = 0;
   out_7752957074421275810[16] = 0;
   out_7752957074421275810[17] = 0;
}
void h_30(double *state, double *unused, double *out_1841788684617719636) {
   out_1841788684617719636[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7778808736427167684) {
   out_7778808736427167684[0] = 0;
   out_7778808736427167684[1] = 0;
   out_7778808736427167684[2] = 0;
   out_7778808736427167684[3] = 0;
   out_7778808736427167684[4] = 1;
   out_7778808736427167684[5] = 0;
   out_7778808736427167684[6] = 0;
   out_7778808736427167684[7] = 0;
   out_7778808736427167684[8] = 0;
}
void h_26(double *state, double *unused, double *out_898278831716057201) {
   out_898278831716057201[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9741676916710953) {
   out_9741676916710953[0] = 0;
   out_9741676916710953[1] = 0;
   out_9741676916710953[2] = 0;
   out_9741676916710953[3] = 0;
   out_9741676916710953[4] = 0;
   out_9741676916710953[5] = 0;
   out_9741676916710953[6] = 0;
   out_9741676916710953[7] = 1;
   out_9741676916710953[8] = 0;
}
void h_27(double *state, double *unused, double *out_780271437782680457) {
   out_780271437782680457[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8493172025481959021) {
   out_8493172025481959021[0] = 0;
   out_8493172025481959021[1] = 0;
   out_8493172025481959021[2] = 0;
   out_8493172025481959021[3] = 1;
   out_8493172025481959021[4] = 0;
   out_8493172025481959021[5] = 0;
   out_8493172025481959021[6] = 0;
   out_8493172025481959021[7] = 0;
   out_8493172025481959021[8] = 0;
}
void h_29(double *state, double *unused, double *out_4878574736062052321) {
   out_4878574736062052321[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7268577392112775500) {
   out_7268577392112775500[0] = 0;
   out_7268577392112775500[1] = 1;
   out_7268577392112775500[2] = 0;
   out_7268577392112775500[3] = 0;
   out_7268577392112775500[4] = 0;
   out_7268577392112775500[5] = 0;
   out_7268577392112775500[6] = 0;
   out_7268577392112775500[7] = 0;
   out_7268577392112775500[8] = 0;
}
void h_28(double *state, double *unused, double *out_201812655552617655) {
   out_201812655552617655[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6095767664527245542) {
   out_6095767664527245542[0] = 1;
   out_6095767664527245542[1] = 0;
   out_6095767664527245542[2] = 0;
   out_6095767664527245542[3] = 0;
   out_6095767664527245542[4] = 0;
   out_6095767664527245542[5] = 0;
   out_6095767664527245542[6] = 0;
   out_6095767664527245542[7] = 0;
   out_6095767664527245542[8] = 0;
}
void h_31(double *state, double *unused, double *out_5775956747982297834) {
   out_5775956747982297834[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3781890957667727605) {
   out_3781890957667727605[0] = 0;
   out_3781890957667727605[1] = 0;
   out_3781890957667727605[2] = 0;
   out_3781890957667727605[3] = 0;
   out_3781890957667727605[4] = 0;
   out_3781890957667727605[5] = 0;
   out_3781890957667727605[6] = 0;
   out_3781890957667727605[7] = 0;
   out_3781890957667727605[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5652973485134955936) {
  err_fun(nom_x, delta_x, out_5652973485134955936);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5828134910451363179) {
  inv_err_fun(nom_x, true_x, out_5828134910451363179);
}
void car_H_mod_fun(double *state, double *out_5063456026301800763) {
  H_mod_fun(state, out_5063456026301800763);
}
void car_f_fun(double *state, double dt, double *out_9008031268712533140) {
  f_fun(state,  dt, out_9008031268712533140);
}
void car_F_fun(double *state, double dt, double *out_399800080831945283) {
  F_fun(state,  dt, out_399800080831945283);
}
void car_h_25(double *state, double *unused, double *out_8429616611094014799) {
  h_25(state, unused, out_8429616611094014799);
}
void car_H_25(double *state, double *unused, double *out_3751244995790767177) {
  H_25(state, unused, out_3751244995790767177);
}
void car_h_24(double *state, double *unused, double *out_6860572812747741310) {
  h_24(state, unused, out_6860572812747741310);
}
void car_H_24(double *state, double *unused, double *out_7752957074421275810) {
  H_24(state, unused, out_7752957074421275810);
}
void car_h_30(double *state, double *unused, double *out_1841788684617719636) {
  h_30(state, unused, out_1841788684617719636);
}
void car_H_30(double *state, double *unused, double *out_7778808736427167684) {
  H_30(state, unused, out_7778808736427167684);
}
void car_h_26(double *state, double *unused, double *out_898278831716057201) {
  h_26(state, unused, out_898278831716057201);
}
void car_H_26(double *state, double *unused, double *out_9741676916710953) {
  H_26(state, unused, out_9741676916710953);
}
void car_h_27(double *state, double *unused, double *out_780271437782680457) {
  h_27(state, unused, out_780271437782680457);
}
void car_H_27(double *state, double *unused, double *out_8493172025481959021) {
  H_27(state, unused, out_8493172025481959021);
}
void car_h_29(double *state, double *unused, double *out_4878574736062052321) {
  h_29(state, unused, out_4878574736062052321);
}
void car_H_29(double *state, double *unused, double *out_7268577392112775500) {
  H_29(state, unused, out_7268577392112775500);
}
void car_h_28(double *state, double *unused, double *out_201812655552617655) {
  h_28(state, unused, out_201812655552617655);
}
void car_H_28(double *state, double *unused, double *out_6095767664527245542) {
  H_28(state, unused, out_6095767664527245542);
}
void car_h_31(double *state, double *unused, double *out_5775956747982297834) {
  h_31(state, unused, out_5775956747982297834);
}
void car_H_31(double *state, double *unused, double *out_3781890957667727605) {
  H_31(state, unused, out_3781890957667727605);
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
