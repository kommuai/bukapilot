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
void err_fun(double *nom_x, double *delta_x, double *out_8975246762864864452) {
   out_8975246762864864452[0] = delta_x[0] + nom_x[0];
   out_8975246762864864452[1] = delta_x[1] + nom_x[1];
   out_8975246762864864452[2] = delta_x[2] + nom_x[2];
   out_8975246762864864452[3] = delta_x[3] + nom_x[3];
   out_8975246762864864452[4] = delta_x[4] + nom_x[4];
   out_8975246762864864452[5] = delta_x[5] + nom_x[5];
   out_8975246762864864452[6] = delta_x[6] + nom_x[6];
   out_8975246762864864452[7] = delta_x[7] + nom_x[7];
   out_8975246762864864452[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4540036244876903697) {
   out_4540036244876903697[0] = -nom_x[0] + true_x[0];
   out_4540036244876903697[1] = -nom_x[1] + true_x[1];
   out_4540036244876903697[2] = -nom_x[2] + true_x[2];
   out_4540036244876903697[3] = -nom_x[3] + true_x[3];
   out_4540036244876903697[4] = -nom_x[4] + true_x[4];
   out_4540036244876903697[5] = -nom_x[5] + true_x[5];
   out_4540036244876903697[6] = -nom_x[6] + true_x[6];
   out_4540036244876903697[7] = -nom_x[7] + true_x[7];
   out_4540036244876903697[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3842189826305601254) {
   out_3842189826305601254[0] = 1.0;
   out_3842189826305601254[1] = 0;
   out_3842189826305601254[2] = 0;
   out_3842189826305601254[3] = 0;
   out_3842189826305601254[4] = 0;
   out_3842189826305601254[5] = 0;
   out_3842189826305601254[6] = 0;
   out_3842189826305601254[7] = 0;
   out_3842189826305601254[8] = 0;
   out_3842189826305601254[9] = 0;
   out_3842189826305601254[10] = 1.0;
   out_3842189826305601254[11] = 0;
   out_3842189826305601254[12] = 0;
   out_3842189826305601254[13] = 0;
   out_3842189826305601254[14] = 0;
   out_3842189826305601254[15] = 0;
   out_3842189826305601254[16] = 0;
   out_3842189826305601254[17] = 0;
   out_3842189826305601254[18] = 0;
   out_3842189826305601254[19] = 0;
   out_3842189826305601254[20] = 1.0;
   out_3842189826305601254[21] = 0;
   out_3842189826305601254[22] = 0;
   out_3842189826305601254[23] = 0;
   out_3842189826305601254[24] = 0;
   out_3842189826305601254[25] = 0;
   out_3842189826305601254[26] = 0;
   out_3842189826305601254[27] = 0;
   out_3842189826305601254[28] = 0;
   out_3842189826305601254[29] = 0;
   out_3842189826305601254[30] = 1.0;
   out_3842189826305601254[31] = 0;
   out_3842189826305601254[32] = 0;
   out_3842189826305601254[33] = 0;
   out_3842189826305601254[34] = 0;
   out_3842189826305601254[35] = 0;
   out_3842189826305601254[36] = 0;
   out_3842189826305601254[37] = 0;
   out_3842189826305601254[38] = 0;
   out_3842189826305601254[39] = 0;
   out_3842189826305601254[40] = 1.0;
   out_3842189826305601254[41] = 0;
   out_3842189826305601254[42] = 0;
   out_3842189826305601254[43] = 0;
   out_3842189826305601254[44] = 0;
   out_3842189826305601254[45] = 0;
   out_3842189826305601254[46] = 0;
   out_3842189826305601254[47] = 0;
   out_3842189826305601254[48] = 0;
   out_3842189826305601254[49] = 0;
   out_3842189826305601254[50] = 1.0;
   out_3842189826305601254[51] = 0;
   out_3842189826305601254[52] = 0;
   out_3842189826305601254[53] = 0;
   out_3842189826305601254[54] = 0;
   out_3842189826305601254[55] = 0;
   out_3842189826305601254[56] = 0;
   out_3842189826305601254[57] = 0;
   out_3842189826305601254[58] = 0;
   out_3842189826305601254[59] = 0;
   out_3842189826305601254[60] = 1.0;
   out_3842189826305601254[61] = 0;
   out_3842189826305601254[62] = 0;
   out_3842189826305601254[63] = 0;
   out_3842189826305601254[64] = 0;
   out_3842189826305601254[65] = 0;
   out_3842189826305601254[66] = 0;
   out_3842189826305601254[67] = 0;
   out_3842189826305601254[68] = 0;
   out_3842189826305601254[69] = 0;
   out_3842189826305601254[70] = 1.0;
   out_3842189826305601254[71] = 0;
   out_3842189826305601254[72] = 0;
   out_3842189826305601254[73] = 0;
   out_3842189826305601254[74] = 0;
   out_3842189826305601254[75] = 0;
   out_3842189826305601254[76] = 0;
   out_3842189826305601254[77] = 0;
   out_3842189826305601254[78] = 0;
   out_3842189826305601254[79] = 0;
   out_3842189826305601254[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4130278799096072198) {
   out_4130278799096072198[0] = state[0];
   out_4130278799096072198[1] = state[1];
   out_4130278799096072198[2] = state[2];
   out_4130278799096072198[3] = state[3];
   out_4130278799096072198[4] = state[4];
   out_4130278799096072198[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4130278799096072198[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4130278799096072198[7] = state[7];
   out_4130278799096072198[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2680083277103679996) {
   out_2680083277103679996[0] = 1;
   out_2680083277103679996[1] = 0;
   out_2680083277103679996[2] = 0;
   out_2680083277103679996[3] = 0;
   out_2680083277103679996[4] = 0;
   out_2680083277103679996[5] = 0;
   out_2680083277103679996[6] = 0;
   out_2680083277103679996[7] = 0;
   out_2680083277103679996[8] = 0;
   out_2680083277103679996[9] = 0;
   out_2680083277103679996[10] = 1;
   out_2680083277103679996[11] = 0;
   out_2680083277103679996[12] = 0;
   out_2680083277103679996[13] = 0;
   out_2680083277103679996[14] = 0;
   out_2680083277103679996[15] = 0;
   out_2680083277103679996[16] = 0;
   out_2680083277103679996[17] = 0;
   out_2680083277103679996[18] = 0;
   out_2680083277103679996[19] = 0;
   out_2680083277103679996[20] = 1;
   out_2680083277103679996[21] = 0;
   out_2680083277103679996[22] = 0;
   out_2680083277103679996[23] = 0;
   out_2680083277103679996[24] = 0;
   out_2680083277103679996[25] = 0;
   out_2680083277103679996[26] = 0;
   out_2680083277103679996[27] = 0;
   out_2680083277103679996[28] = 0;
   out_2680083277103679996[29] = 0;
   out_2680083277103679996[30] = 1;
   out_2680083277103679996[31] = 0;
   out_2680083277103679996[32] = 0;
   out_2680083277103679996[33] = 0;
   out_2680083277103679996[34] = 0;
   out_2680083277103679996[35] = 0;
   out_2680083277103679996[36] = 0;
   out_2680083277103679996[37] = 0;
   out_2680083277103679996[38] = 0;
   out_2680083277103679996[39] = 0;
   out_2680083277103679996[40] = 1;
   out_2680083277103679996[41] = 0;
   out_2680083277103679996[42] = 0;
   out_2680083277103679996[43] = 0;
   out_2680083277103679996[44] = 0;
   out_2680083277103679996[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2680083277103679996[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2680083277103679996[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2680083277103679996[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2680083277103679996[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2680083277103679996[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2680083277103679996[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2680083277103679996[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2680083277103679996[53] = -9.8000000000000007*dt;
   out_2680083277103679996[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2680083277103679996[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2680083277103679996[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2680083277103679996[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2680083277103679996[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2680083277103679996[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2680083277103679996[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2680083277103679996[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2680083277103679996[62] = 0;
   out_2680083277103679996[63] = 0;
   out_2680083277103679996[64] = 0;
   out_2680083277103679996[65] = 0;
   out_2680083277103679996[66] = 0;
   out_2680083277103679996[67] = 0;
   out_2680083277103679996[68] = 0;
   out_2680083277103679996[69] = 0;
   out_2680083277103679996[70] = 1;
   out_2680083277103679996[71] = 0;
   out_2680083277103679996[72] = 0;
   out_2680083277103679996[73] = 0;
   out_2680083277103679996[74] = 0;
   out_2680083277103679996[75] = 0;
   out_2680083277103679996[76] = 0;
   out_2680083277103679996[77] = 0;
   out_2680083277103679996[78] = 0;
   out_2680083277103679996[79] = 0;
   out_2680083277103679996[80] = 1;
}
void h_25(double *state, double *unused, double *out_3861349082676883418) {
   out_3861349082676883418[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4949154357715001298) {
   out_4949154357715001298[0] = 0;
   out_4949154357715001298[1] = 0;
   out_4949154357715001298[2] = 0;
   out_4949154357715001298[3] = 0;
   out_4949154357715001298[4] = 0;
   out_4949154357715001298[5] = 0;
   out_4949154357715001298[6] = 1;
   out_4949154357715001298[7] = 0;
   out_4949154357715001298[8] = 0;
}
void h_24(double *state, double *unused, double *out_6369036304708791866) {
   out_6369036304708791866[0] = state[4];
   out_6369036304708791866[1] = state[5];
}
void H_24(double *state, double *unused, double *out_202645030748492264) {
   out_202645030748492264[0] = 0;
   out_202645030748492264[1] = 0;
   out_202645030748492264[2] = 0;
   out_202645030748492264[3] = 0;
   out_202645030748492264[4] = 1;
   out_202645030748492264[5] = 0;
   out_202645030748492264[6] = 0;
   out_202645030748492264[7] = 0;
   out_202645030748492264[8] = 0;
   out_202645030748492264[9] = 0;
   out_202645030748492264[10] = 0;
   out_202645030748492264[11] = 0;
   out_202645030748492264[12] = 0;
   out_202645030748492264[13] = 0;
   out_202645030748492264[14] = 1;
   out_202645030748492264[15] = 0;
   out_202645030748492264[16] = 0;
   out_202645030748492264[17] = 0;
}
void h_30(double *state, double *unused, double *out_4006868906897538389) {
   out_4006868906897538389[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7467487316222249925) {
   out_7467487316222249925[0] = 0;
   out_7467487316222249925[1] = 0;
   out_7467487316222249925[2] = 0;
   out_7467487316222249925[3] = 0;
   out_7467487316222249925[4] = 1;
   out_7467487316222249925[5] = 0;
   out_7467487316222249925[6] = 0;
   out_7467487316222249925[7] = 0;
   out_7467487316222249925[8] = 0;
}
void h_26(double *state, double *unused, double *out_994804360763307256) {
   out_994804360763307256[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1207651038840945074) {
   out_1207651038840945074[0] = 0;
   out_1207651038840945074[1] = 0;
   out_1207651038840945074[2] = 0;
   out_1207651038840945074[3] = 0;
   out_1207651038840945074[4] = 0;
   out_1207651038840945074[5] = 0;
   out_1207651038840945074[6] = 0;
   out_1207651038840945074[7] = 1;
   out_1207651038840945074[8] = 0;
}
void h_27(double *state, double *unused, double *out_8863022350337729063) {
   out_8863022350337729063[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8755662686303358474) {
   out_8755662686303358474[0] = 0;
   out_8755662686303358474[1] = 0;
   out_8755662686303358474[2] = 0;
   out_8755662686303358474[3] = 1;
   out_8755662686303358474[4] = 0;
   out_8755662686303358474[5] = 0;
   out_8755662686303358474[6] = 0;
   out_8755662686303358474[7] = 0;
   out_8755662686303358474[8] = 0;
}
void h_29(double *state, double *unused, double *out_6635995733016337592) {
   out_6635995733016337592[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7977718660536642109) {
   out_7977718660536642109[0] = 0;
   out_7977718660536642109[1] = 1;
   out_7977718660536642109[2] = 0;
   out_7977718660536642109[3] = 0;
   out_7977718660536642109[4] = 0;
   out_7977718660536642109[5] = 0;
   out_7977718660536642109[6] = 0;
   out_7977718660536642109[7] = 0;
   out_7977718660536642109[8] = 0;
}
void h_28(double *state, double *unused, double *out_7919512497436066628) {
   out_7919512497436066628[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2895319643467111535) {
   out_2895319643467111535[0] = 1;
   out_2895319643467111535[1] = 0;
   out_2895319643467111535[2] = 0;
   out_2895319643467111535[3] = 0;
   out_2895319643467111535[4] = 0;
   out_2895319643467111535[5] = 0;
   out_2895319643467111535[6] = 0;
   out_2895319643467111535[7] = 0;
   out_2895319643467111535[8] = 0;
}
void h_31(double *state, double *unused, double *out_223899554422861068) {
   out_223899554422861068[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4979800319591961726) {
   out_4979800319591961726[0] = 0;
   out_4979800319591961726[1] = 0;
   out_4979800319591961726[2] = 0;
   out_4979800319591961726[3] = 0;
   out_4979800319591961726[4] = 0;
   out_4979800319591961726[5] = 0;
   out_4979800319591961726[6] = 0;
   out_4979800319591961726[7] = 0;
   out_4979800319591961726[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8975246762864864452) {
  err_fun(nom_x, delta_x, out_8975246762864864452);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4540036244876903697) {
  inv_err_fun(nom_x, true_x, out_4540036244876903697);
}
void car_H_mod_fun(double *state, double *out_3842189826305601254) {
  H_mod_fun(state, out_3842189826305601254);
}
void car_f_fun(double *state, double dt, double *out_4130278799096072198) {
  f_fun(state,  dt, out_4130278799096072198);
}
void car_F_fun(double *state, double dt, double *out_2680083277103679996) {
  F_fun(state,  dt, out_2680083277103679996);
}
void car_h_25(double *state, double *unused, double *out_3861349082676883418) {
  h_25(state, unused, out_3861349082676883418);
}
void car_H_25(double *state, double *unused, double *out_4949154357715001298) {
  H_25(state, unused, out_4949154357715001298);
}
void car_h_24(double *state, double *unused, double *out_6369036304708791866) {
  h_24(state, unused, out_6369036304708791866);
}
void car_H_24(double *state, double *unused, double *out_202645030748492264) {
  H_24(state, unused, out_202645030748492264);
}
void car_h_30(double *state, double *unused, double *out_4006868906897538389) {
  h_30(state, unused, out_4006868906897538389);
}
void car_H_30(double *state, double *unused, double *out_7467487316222249925) {
  H_30(state, unused, out_7467487316222249925);
}
void car_h_26(double *state, double *unused, double *out_994804360763307256) {
  h_26(state, unused, out_994804360763307256);
}
void car_H_26(double *state, double *unused, double *out_1207651038840945074) {
  H_26(state, unused, out_1207651038840945074);
}
void car_h_27(double *state, double *unused, double *out_8863022350337729063) {
  h_27(state, unused, out_8863022350337729063);
}
void car_H_27(double *state, double *unused, double *out_8755662686303358474) {
  H_27(state, unused, out_8755662686303358474);
}
void car_h_29(double *state, double *unused, double *out_6635995733016337592) {
  h_29(state, unused, out_6635995733016337592);
}
void car_H_29(double *state, double *unused, double *out_7977718660536642109) {
  H_29(state, unused, out_7977718660536642109);
}
void car_h_28(double *state, double *unused, double *out_7919512497436066628) {
  h_28(state, unused, out_7919512497436066628);
}
void car_H_28(double *state, double *unused, double *out_2895319643467111535) {
  H_28(state, unused, out_2895319643467111535);
}
void car_h_31(double *state, double *unused, double *out_223899554422861068) {
  h_31(state, unused, out_223899554422861068);
}
void car_H_31(double *state, double *unused, double *out_4979800319591961726) {
  H_31(state, unused, out_4979800319591961726);
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
