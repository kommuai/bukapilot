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
void err_fun(double *nom_x, double *delta_x, double *out_8049540775897896583) {
   out_8049540775897896583[0] = delta_x[0] + nom_x[0];
   out_8049540775897896583[1] = delta_x[1] + nom_x[1];
   out_8049540775897896583[2] = delta_x[2] + nom_x[2];
   out_8049540775897896583[3] = delta_x[3] + nom_x[3];
   out_8049540775897896583[4] = delta_x[4] + nom_x[4];
   out_8049540775897896583[5] = delta_x[5] + nom_x[5];
   out_8049540775897896583[6] = delta_x[6] + nom_x[6];
   out_8049540775897896583[7] = delta_x[7] + nom_x[7];
   out_8049540775897896583[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7074179482604986343) {
   out_7074179482604986343[0] = -nom_x[0] + true_x[0];
   out_7074179482604986343[1] = -nom_x[1] + true_x[1];
   out_7074179482604986343[2] = -nom_x[2] + true_x[2];
   out_7074179482604986343[3] = -nom_x[3] + true_x[3];
   out_7074179482604986343[4] = -nom_x[4] + true_x[4];
   out_7074179482604986343[5] = -nom_x[5] + true_x[5];
   out_7074179482604986343[6] = -nom_x[6] + true_x[6];
   out_7074179482604986343[7] = -nom_x[7] + true_x[7];
   out_7074179482604986343[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2468025013813546553) {
   out_2468025013813546553[0] = 1.0;
   out_2468025013813546553[1] = 0;
   out_2468025013813546553[2] = 0;
   out_2468025013813546553[3] = 0;
   out_2468025013813546553[4] = 0;
   out_2468025013813546553[5] = 0;
   out_2468025013813546553[6] = 0;
   out_2468025013813546553[7] = 0;
   out_2468025013813546553[8] = 0;
   out_2468025013813546553[9] = 0;
   out_2468025013813546553[10] = 1.0;
   out_2468025013813546553[11] = 0;
   out_2468025013813546553[12] = 0;
   out_2468025013813546553[13] = 0;
   out_2468025013813546553[14] = 0;
   out_2468025013813546553[15] = 0;
   out_2468025013813546553[16] = 0;
   out_2468025013813546553[17] = 0;
   out_2468025013813546553[18] = 0;
   out_2468025013813546553[19] = 0;
   out_2468025013813546553[20] = 1.0;
   out_2468025013813546553[21] = 0;
   out_2468025013813546553[22] = 0;
   out_2468025013813546553[23] = 0;
   out_2468025013813546553[24] = 0;
   out_2468025013813546553[25] = 0;
   out_2468025013813546553[26] = 0;
   out_2468025013813546553[27] = 0;
   out_2468025013813546553[28] = 0;
   out_2468025013813546553[29] = 0;
   out_2468025013813546553[30] = 1.0;
   out_2468025013813546553[31] = 0;
   out_2468025013813546553[32] = 0;
   out_2468025013813546553[33] = 0;
   out_2468025013813546553[34] = 0;
   out_2468025013813546553[35] = 0;
   out_2468025013813546553[36] = 0;
   out_2468025013813546553[37] = 0;
   out_2468025013813546553[38] = 0;
   out_2468025013813546553[39] = 0;
   out_2468025013813546553[40] = 1.0;
   out_2468025013813546553[41] = 0;
   out_2468025013813546553[42] = 0;
   out_2468025013813546553[43] = 0;
   out_2468025013813546553[44] = 0;
   out_2468025013813546553[45] = 0;
   out_2468025013813546553[46] = 0;
   out_2468025013813546553[47] = 0;
   out_2468025013813546553[48] = 0;
   out_2468025013813546553[49] = 0;
   out_2468025013813546553[50] = 1.0;
   out_2468025013813546553[51] = 0;
   out_2468025013813546553[52] = 0;
   out_2468025013813546553[53] = 0;
   out_2468025013813546553[54] = 0;
   out_2468025013813546553[55] = 0;
   out_2468025013813546553[56] = 0;
   out_2468025013813546553[57] = 0;
   out_2468025013813546553[58] = 0;
   out_2468025013813546553[59] = 0;
   out_2468025013813546553[60] = 1.0;
   out_2468025013813546553[61] = 0;
   out_2468025013813546553[62] = 0;
   out_2468025013813546553[63] = 0;
   out_2468025013813546553[64] = 0;
   out_2468025013813546553[65] = 0;
   out_2468025013813546553[66] = 0;
   out_2468025013813546553[67] = 0;
   out_2468025013813546553[68] = 0;
   out_2468025013813546553[69] = 0;
   out_2468025013813546553[70] = 1.0;
   out_2468025013813546553[71] = 0;
   out_2468025013813546553[72] = 0;
   out_2468025013813546553[73] = 0;
   out_2468025013813546553[74] = 0;
   out_2468025013813546553[75] = 0;
   out_2468025013813546553[76] = 0;
   out_2468025013813546553[77] = 0;
   out_2468025013813546553[78] = 0;
   out_2468025013813546553[79] = 0;
   out_2468025013813546553[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_9198223713481844381) {
   out_9198223713481844381[0] = state[0];
   out_9198223713481844381[1] = state[1];
   out_9198223713481844381[2] = state[2];
   out_9198223713481844381[3] = state[3];
   out_9198223713481844381[4] = state[4];
   out_9198223713481844381[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_9198223713481844381[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_9198223713481844381[7] = state[7];
   out_9198223713481844381[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1931036568146832837) {
   out_1931036568146832837[0] = 1;
   out_1931036568146832837[1] = 0;
   out_1931036568146832837[2] = 0;
   out_1931036568146832837[3] = 0;
   out_1931036568146832837[4] = 0;
   out_1931036568146832837[5] = 0;
   out_1931036568146832837[6] = 0;
   out_1931036568146832837[7] = 0;
   out_1931036568146832837[8] = 0;
   out_1931036568146832837[9] = 0;
   out_1931036568146832837[10] = 1;
   out_1931036568146832837[11] = 0;
   out_1931036568146832837[12] = 0;
   out_1931036568146832837[13] = 0;
   out_1931036568146832837[14] = 0;
   out_1931036568146832837[15] = 0;
   out_1931036568146832837[16] = 0;
   out_1931036568146832837[17] = 0;
   out_1931036568146832837[18] = 0;
   out_1931036568146832837[19] = 0;
   out_1931036568146832837[20] = 1;
   out_1931036568146832837[21] = 0;
   out_1931036568146832837[22] = 0;
   out_1931036568146832837[23] = 0;
   out_1931036568146832837[24] = 0;
   out_1931036568146832837[25] = 0;
   out_1931036568146832837[26] = 0;
   out_1931036568146832837[27] = 0;
   out_1931036568146832837[28] = 0;
   out_1931036568146832837[29] = 0;
   out_1931036568146832837[30] = 1;
   out_1931036568146832837[31] = 0;
   out_1931036568146832837[32] = 0;
   out_1931036568146832837[33] = 0;
   out_1931036568146832837[34] = 0;
   out_1931036568146832837[35] = 0;
   out_1931036568146832837[36] = 0;
   out_1931036568146832837[37] = 0;
   out_1931036568146832837[38] = 0;
   out_1931036568146832837[39] = 0;
   out_1931036568146832837[40] = 1;
   out_1931036568146832837[41] = 0;
   out_1931036568146832837[42] = 0;
   out_1931036568146832837[43] = 0;
   out_1931036568146832837[44] = 0;
   out_1931036568146832837[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1931036568146832837[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1931036568146832837[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1931036568146832837[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1931036568146832837[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1931036568146832837[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1931036568146832837[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1931036568146832837[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1931036568146832837[53] = -9.8000000000000007*dt;
   out_1931036568146832837[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1931036568146832837[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1931036568146832837[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1931036568146832837[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1931036568146832837[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1931036568146832837[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1931036568146832837[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1931036568146832837[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1931036568146832837[62] = 0;
   out_1931036568146832837[63] = 0;
   out_1931036568146832837[64] = 0;
   out_1931036568146832837[65] = 0;
   out_1931036568146832837[66] = 0;
   out_1931036568146832837[67] = 0;
   out_1931036568146832837[68] = 0;
   out_1931036568146832837[69] = 0;
   out_1931036568146832837[70] = 1;
   out_1931036568146832837[71] = 0;
   out_1931036568146832837[72] = 0;
   out_1931036568146832837[73] = 0;
   out_1931036568146832837[74] = 0;
   out_1931036568146832837[75] = 0;
   out_1931036568146832837[76] = 0;
   out_1931036568146832837[77] = 0;
   out_1931036568146832837[78] = 0;
   out_1931036568146832837[79] = 0;
   out_1931036568146832837[80] = 1;
}
void h_25(double *state, double *unused, double *out_2954337136221408228) {
   out_2954337136221408228[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7651867888905377355) {
   out_7651867888905377355[0] = 0;
   out_7651867888905377355[1] = 0;
   out_7651867888905377355[2] = 0;
   out_7651867888905377355[3] = 0;
   out_7651867888905377355[4] = 0;
   out_7651867888905377355[5] = 0;
   out_7651867888905377355[6] = 1;
   out_7651867888905377355[7] = 0;
   out_7651867888905377355[8] = 0;
}
void h_24(double *state, double *unused, double *out_4961849974984120864) {
   out_4961849974984120864[0] = state[4];
   out_4961849974984120864[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8622226585798674695) {
   out_8622226585798674695[0] = 0;
   out_8622226585798674695[1] = 0;
   out_8622226585798674695[2] = 0;
   out_8622226585798674695[3] = 0;
   out_8622226585798674695[4] = 1;
   out_8622226585798674695[5] = 0;
   out_8622226585798674695[6] = 0;
   out_8622226585798674695[7] = 0;
   out_8622226585798674695[8] = 0;
   out_8622226585798674695[9] = 0;
   out_8622226585798674695[10] = 0;
   out_8622226585798674695[11] = 0;
   out_8622226585798674695[12] = 0;
   out_8622226585798674695[13] = 0;
   out_8622226585798674695[14] = 1;
   out_8622226585798674695[15] = 0;
   out_8622226585798674695[16] = 0;
   out_8622226585798674695[17] = 0;
}
void h_30(double *state, double *unused, double *out_4404968528850291395) {
   out_4404968528850291395[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5133534930398128728) {
   out_5133534930398128728[0] = 0;
   out_5133534930398128728[1] = 0;
   out_5133534930398128728[2] = 0;
   out_5133534930398128728[3] = 0;
   out_5133534930398128728[4] = 1;
   out_5133534930398128728[5] = 0;
   out_5133534930398128728[6] = 0;
   out_5133534930398128728[7] = 0;
   out_5133534930398128728[8] = 0;
}
void h_26(double *state, double *unused, double *out_5008284413526768912) {
   out_5008284413526768912[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7053372865930118037) {
   out_7053372865930118037[0] = 0;
   out_7053372865930118037[1] = 0;
   out_7053372865930118037[2] = 0;
   out_7053372865930118037[3] = 0;
   out_7053372865930118037[4] = 0;
   out_7053372865930118037[5] = 0;
   out_7053372865930118037[6] = 0;
   out_7053372865930118037[7] = 1;
   out_7053372865930118037[8] = 0;
}
void h_27(double *state, double *unused, double *out_321510676526048361) {
   out_321510676526048361[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4092416542876141152) {
   out_4092416542876141152[0] = 0;
   out_4092416542876141152[1] = 0;
   out_4092416542876141152[2] = 0;
   out_4092416542876141152[3] = 1;
   out_4092416542876141152[4] = 0;
   out_4092416542876141152[5] = 0;
   out_4092416542876141152[6] = 0;
   out_4092416542876141152[7] = 0;
   out_4092416542876141152[8] = 0;
}
void h_29(double *state, double *unused, double *out_596704738810554250) {
   out_596704738810554250[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6777411198990958247) {
   out_6777411198990958247[0] = 0;
   out_6777411198990958247[1] = 1;
   out_6777411198990958247[2] = 0;
   out_6777411198990958247[3] = 0;
   out_6777411198990958247[4] = 0;
   out_6777411198990958247[5] = 0;
   out_6777411198990958247[6] = 0;
   out_6777411198990958247[7] = 0;
   out_6777411198990958247[8] = 0;
}
void h_28(double *state, double *unused, double *out_6934633040567403348) {
   out_6934633040567403348[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8741041470556284498) {
   out_8741041470556284498[0] = 1;
   out_8741041470556284498[1] = 0;
   out_8741041470556284498[2] = 0;
   out_8741041470556284498[3] = 0;
   out_8741041470556284498[4] = 0;
   out_8741041470556284498[5] = 0;
   out_8741041470556284498[6] = 0;
   out_8741041470556284498[7] = 0;
   out_8741041470556284498[8] = 0;
}
void h_31(double *state, double *unused, double *out_3816498090128942708) {
   out_3816498090128942708[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6427164763696766561) {
   out_6427164763696766561[0] = 0;
   out_6427164763696766561[1] = 0;
   out_6427164763696766561[2] = 0;
   out_6427164763696766561[3] = 0;
   out_6427164763696766561[4] = 0;
   out_6427164763696766561[5] = 0;
   out_6427164763696766561[6] = 0;
   out_6427164763696766561[7] = 0;
   out_6427164763696766561[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8049540775897896583) {
  err_fun(nom_x, delta_x, out_8049540775897896583);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7074179482604986343) {
  inv_err_fun(nom_x, true_x, out_7074179482604986343);
}
void car_H_mod_fun(double *state, double *out_2468025013813546553) {
  H_mod_fun(state, out_2468025013813546553);
}
void car_f_fun(double *state, double dt, double *out_9198223713481844381) {
  f_fun(state,  dt, out_9198223713481844381);
}
void car_F_fun(double *state, double dt, double *out_1931036568146832837) {
  F_fun(state,  dt, out_1931036568146832837);
}
void car_h_25(double *state, double *unused, double *out_2954337136221408228) {
  h_25(state, unused, out_2954337136221408228);
}
void car_H_25(double *state, double *unused, double *out_7651867888905377355) {
  H_25(state, unused, out_7651867888905377355);
}
void car_h_24(double *state, double *unused, double *out_4961849974984120864) {
  h_24(state, unused, out_4961849974984120864);
}
void car_H_24(double *state, double *unused, double *out_8622226585798674695) {
  H_24(state, unused, out_8622226585798674695);
}
void car_h_30(double *state, double *unused, double *out_4404968528850291395) {
  h_30(state, unused, out_4404968528850291395);
}
void car_H_30(double *state, double *unused, double *out_5133534930398128728) {
  H_30(state, unused, out_5133534930398128728);
}
void car_h_26(double *state, double *unused, double *out_5008284413526768912) {
  h_26(state, unused, out_5008284413526768912);
}
void car_H_26(double *state, double *unused, double *out_7053372865930118037) {
  H_26(state, unused, out_7053372865930118037);
}
void car_h_27(double *state, double *unused, double *out_321510676526048361) {
  h_27(state, unused, out_321510676526048361);
}
void car_H_27(double *state, double *unused, double *out_4092416542876141152) {
  H_27(state, unused, out_4092416542876141152);
}
void car_h_29(double *state, double *unused, double *out_596704738810554250) {
  h_29(state, unused, out_596704738810554250);
}
void car_H_29(double *state, double *unused, double *out_6777411198990958247) {
  H_29(state, unused, out_6777411198990958247);
}
void car_h_28(double *state, double *unused, double *out_6934633040567403348) {
  h_28(state, unused, out_6934633040567403348);
}
void car_H_28(double *state, double *unused, double *out_8741041470556284498) {
  H_28(state, unused, out_8741041470556284498);
}
void car_h_31(double *state, double *unused, double *out_3816498090128942708) {
  h_31(state, unused, out_3816498090128942708);
}
void car_H_31(double *state, double *unused, double *out_6427164763696766561) {
  H_31(state, unused, out_6427164763696766561);
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
