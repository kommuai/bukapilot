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
void err_fun(double *nom_x, double *delta_x, double *out_7166728709141313111) {
   out_7166728709141313111[0] = delta_x[0] + nom_x[0];
   out_7166728709141313111[1] = delta_x[1] + nom_x[1];
   out_7166728709141313111[2] = delta_x[2] + nom_x[2];
   out_7166728709141313111[3] = delta_x[3] + nom_x[3];
   out_7166728709141313111[4] = delta_x[4] + nom_x[4];
   out_7166728709141313111[5] = delta_x[5] + nom_x[5];
   out_7166728709141313111[6] = delta_x[6] + nom_x[6];
   out_7166728709141313111[7] = delta_x[7] + nom_x[7];
   out_7166728709141313111[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7904298925836893155) {
   out_7904298925836893155[0] = -nom_x[0] + true_x[0];
   out_7904298925836893155[1] = -nom_x[1] + true_x[1];
   out_7904298925836893155[2] = -nom_x[2] + true_x[2];
   out_7904298925836893155[3] = -nom_x[3] + true_x[3];
   out_7904298925836893155[4] = -nom_x[4] + true_x[4];
   out_7904298925836893155[5] = -nom_x[5] + true_x[5];
   out_7904298925836893155[6] = -nom_x[6] + true_x[6];
   out_7904298925836893155[7] = -nom_x[7] + true_x[7];
   out_7904298925836893155[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2723439132888064356) {
   out_2723439132888064356[0] = 1.0;
   out_2723439132888064356[1] = 0.0;
   out_2723439132888064356[2] = 0.0;
   out_2723439132888064356[3] = 0.0;
   out_2723439132888064356[4] = 0.0;
   out_2723439132888064356[5] = 0.0;
   out_2723439132888064356[6] = 0.0;
   out_2723439132888064356[7] = 0.0;
   out_2723439132888064356[8] = 0.0;
   out_2723439132888064356[9] = 0.0;
   out_2723439132888064356[10] = 1.0;
   out_2723439132888064356[11] = 0.0;
   out_2723439132888064356[12] = 0.0;
   out_2723439132888064356[13] = 0.0;
   out_2723439132888064356[14] = 0.0;
   out_2723439132888064356[15] = 0.0;
   out_2723439132888064356[16] = 0.0;
   out_2723439132888064356[17] = 0.0;
   out_2723439132888064356[18] = 0.0;
   out_2723439132888064356[19] = 0.0;
   out_2723439132888064356[20] = 1.0;
   out_2723439132888064356[21] = 0.0;
   out_2723439132888064356[22] = 0.0;
   out_2723439132888064356[23] = 0.0;
   out_2723439132888064356[24] = 0.0;
   out_2723439132888064356[25] = 0.0;
   out_2723439132888064356[26] = 0.0;
   out_2723439132888064356[27] = 0.0;
   out_2723439132888064356[28] = 0.0;
   out_2723439132888064356[29] = 0.0;
   out_2723439132888064356[30] = 1.0;
   out_2723439132888064356[31] = 0.0;
   out_2723439132888064356[32] = 0.0;
   out_2723439132888064356[33] = 0.0;
   out_2723439132888064356[34] = 0.0;
   out_2723439132888064356[35] = 0.0;
   out_2723439132888064356[36] = 0.0;
   out_2723439132888064356[37] = 0.0;
   out_2723439132888064356[38] = 0.0;
   out_2723439132888064356[39] = 0.0;
   out_2723439132888064356[40] = 1.0;
   out_2723439132888064356[41] = 0.0;
   out_2723439132888064356[42] = 0.0;
   out_2723439132888064356[43] = 0.0;
   out_2723439132888064356[44] = 0.0;
   out_2723439132888064356[45] = 0.0;
   out_2723439132888064356[46] = 0.0;
   out_2723439132888064356[47] = 0.0;
   out_2723439132888064356[48] = 0.0;
   out_2723439132888064356[49] = 0.0;
   out_2723439132888064356[50] = 1.0;
   out_2723439132888064356[51] = 0.0;
   out_2723439132888064356[52] = 0.0;
   out_2723439132888064356[53] = 0.0;
   out_2723439132888064356[54] = 0.0;
   out_2723439132888064356[55] = 0.0;
   out_2723439132888064356[56] = 0.0;
   out_2723439132888064356[57] = 0.0;
   out_2723439132888064356[58] = 0.0;
   out_2723439132888064356[59] = 0.0;
   out_2723439132888064356[60] = 1.0;
   out_2723439132888064356[61] = 0.0;
   out_2723439132888064356[62] = 0.0;
   out_2723439132888064356[63] = 0.0;
   out_2723439132888064356[64] = 0.0;
   out_2723439132888064356[65] = 0.0;
   out_2723439132888064356[66] = 0.0;
   out_2723439132888064356[67] = 0.0;
   out_2723439132888064356[68] = 0.0;
   out_2723439132888064356[69] = 0.0;
   out_2723439132888064356[70] = 1.0;
   out_2723439132888064356[71] = 0.0;
   out_2723439132888064356[72] = 0.0;
   out_2723439132888064356[73] = 0.0;
   out_2723439132888064356[74] = 0.0;
   out_2723439132888064356[75] = 0.0;
   out_2723439132888064356[76] = 0.0;
   out_2723439132888064356[77] = 0.0;
   out_2723439132888064356[78] = 0.0;
   out_2723439132888064356[79] = 0.0;
   out_2723439132888064356[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_9222521799993261203) {
   out_9222521799993261203[0] = state[0];
   out_9222521799993261203[1] = state[1];
   out_9222521799993261203[2] = state[2];
   out_9222521799993261203[3] = state[3];
   out_9222521799993261203[4] = state[4];
   out_9222521799993261203[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_9222521799993261203[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_9222521799993261203[7] = state[7];
   out_9222521799993261203[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5859762924884839019) {
   out_5859762924884839019[0] = 1;
   out_5859762924884839019[1] = 0;
   out_5859762924884839019[2] = 0;
   out_5859762924884839019[3] = 0;
   out_5859762924884839019[4] = 0;
   out_5859762924884839019[5] = 0;
   out_5859762924884839019[6] = 0;
   out_5859762924884839019[7] = 0;
   out_5859762924884839019[8] = 0;
   out_5859762924884839019[9] = 0;
   out_5859762924884839019[10] = 1;
   out_5859762924884839019[11] = 0;
   out_5859762924884839019[12] = 0;
   out_5859762924884839019[13] = 0;
   out_5859762924884839019[14] = 0;
   out_5859762924884839019[15] = 0;
   out_5859762924884839019[16] = 0;
   out_5859762924884839019[17] = 0;
   out_5859762924884839019[18] = 0;
   out_5859762924884839019[19] = 0;
   out_5859762924884839019[20] = 1;
   out_5859762924884839019[21] = 0;
   out_5859762924884839019[22] = 0;
   out_5859762924884839019[23] = 0;
   out_5859762924884839019[24] = 0;
   out_5859762924884839019[25] = 0;
   out_5859762924884839019[26] = 0;
   out_5859762924884839019[27] = 0;
   out_5859762924884839019[28] = 0;
   out_5859762924884839019[29] = 0;
   out_5859762924884839019[30] = 1;
   out_5859762924884839019[31] = 0;
   out_5859762924884839019[32] = 0;
   out_5859762924884839019[33] = 0;
   out_5859762924884839019[34] = 0;
   out_5859762924884839019[35] = 0;
   out_5859762924884839019[36] = 0;
   out_5859762924884839019[37] = 0;
   out_5859762924884839019[38] = 0;
   out_5859762924884839019[39] = 0;
   out_5859762924884839019[40] = 1;
   out_5859762924884839019[41] = 0;
   out_5859762924884839019[42] = 0;
   out_5859762924884839019[43] = 0;
   out_5859762924884839019[44] = 0;
   out_5859762924884839019[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5859762924884839019[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5859762924884839019[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5859762924884839019[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5859762924884839019[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5859762924884839019[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5859762924884839019[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5859762924884839019[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5859762924884839019[53] = -9.8000000000000007*dt;
   out_5859762924884839019[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5859762924884839019[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5859762924884839019[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5859762924884839019[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5859762924884839019[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5859762924884839019[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5859762924884839019[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5859762924884839019[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5859762924884839019[62] = 0;
   out_5859762924884839019[63] = 0;
   out_5859762924884839019[64] = 0;
   out_5859762924884839019[65] = 0;
   out_5859762924884839019[66] = 0;
   out_5859762924884839019[67] = 0;
   out_5859762924884839019[68] = 0;
   out_5859762924884839019[69] = 0;
   out_5859762924884839019[70] = 1;
   out_5859762924884839019[71] = 0;
   out_5859762924884839019[72] = 0;
   out_5859762924884839019[73] = 0;
   out_5859762924884839019[74] = 0;
   out_5859762924884839019[75] = 0;
   out_5859762924884839019[76] = 0;
   out_5859762924884839019[77] = 0;
   out_5859762924884839019[78] = 0;
   out_5859762924884839019[79] = 0;
   out_5859762924884839019[80] = 1;
}
void h_25(double *state, double *unused, double *out_3573010150517271071) {
   out_3573010150517271071[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3773583827001965595) {
   out_3773583827001965595[0] = 0;
   out_3773583827001965595[1] = 0;
   out_3773583827001965595[2] = 0;
   out_3773583827001965595[3] = 0;
   out_3773583827001965595[4] = 0;
   out_3773583827001965595[5] = 0;
   out_3773583827001965595[6] = 1;
   out_3773583827001965595[7] = 0;
   out_3773583827001965595[8] = 0;
}
void h_24(double *state, double *unused, double *out_4865118116114068912) {
   out_4865118116114068912[0] = state[4];
   out_4865118116114068912[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8642398692029672447) {
   out_8642398692029672447[0] = 0;
   out_8642398692029672447[1] = 0;
   out_8642398692029672447[2] = 0;
   out_8642398692029672447[3] = 0;
   out_8642398692029672447[4] = 1;
   out_8642398692029672447[5] = 0;
   out_8642398692029672447[6] = 0;
   out_8642398692029672447[7] = 0;
   out_8642398692029672447[8] = 0;
   out_8642398692029672447[9] = 0;
   out_8642398692029672447[10] = 0;
   out_8642398692029672447[11] = 0;
   out_8642398692029672447[12] = 0;
   out_8642398692029672447[13] = 0;
   out_8642398692029672447[14] = 1;
   out_8642398692029672447[15] = 0;
   out_8642398692029672447[16] = 0;
   out_8642398692029672447[17] = 0;
}
void h_30(double *state, double *unused, double *out_2057634506830890826) {
   out_2057634506830890826[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8301280157129573793) {
   out_8301280157129573793[0] = 0;
   out_8301280157129573793[1] = 0;
   out_8301280157129573793[2] = 0;
   out_8301280157129573793[3] = 0;
   out_8301280157129573793[4] = 1;
   out_8301280157129573793[5] = 0;
   out_8301280157129573793[6] = 0;
   out_8301280157129573793[7] = 0;
   out_8301280157129573793[8] = 0;
}
void h_26(double *state, double *unused, double *out_2944038760829954819) {
   out_2944038760829954819[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7515087145876021819) {
   out_7515087145876021819[0] = 0;
   out_7515087145876021819[1] = 0;
   out_7515087145876021819[2] = 0;
   out_7515087145876021819[3] = 0;
   out_7515087145876021819[4] = 0;
   out_7515087145876021819[5] = 0;
   out_7515087145876021819[6] = 0;
   out_7515087145876021819[7] = 1;
   out_7515087145876021819[8] = 0;
}
void h_27(double *state, double *unused, double *out_8574683418178116716) {
   out_8574683418178116716[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7970700604779552912) {
   out_7970700604779552912[0] = 0;
   out_7970700604779552912[1] = 0;
   out_7970700604779552912[2] = 0;
   out_7970700604779552912[3] = 1;
   out_7970700604779552912[4] = 0;
   out_7970700604779552912[5] = 0;
   out_7970700604779552912[6] = 0;
   out_7970700604779552912[7] = 0;
   out_7970700604779552912[8] = 0;
}
void h_29(double *state, double *unused, double *out_7124052025549233549) {
   out_7124052025549233549[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7791048812815181609) {
   out_7791048812815181609[0] = 0;
   out_7791048812815181609[1] = 1;
   out_7791048812815181609[2] = 0;
   out_7791048812815181609[3] = 0;
   out_7791048812815181609[4] = 0;
   out_7791048812815181609[5] = 0;
   out_7791048812815181609[6] = 0;
   out_7791048812815181609[7] = 0;
   out_7791048812815181609[8] = 0;
}
void h_28(double *state, double *unused, double *out_4870387387870589255) {
   out_4870387387870589255[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5573296243824839433) {
   out_5573296243824839433[0] = 1;
   out_5573296243824839433[1] = 0;
   out_5573296243824839433[2] = 0;
   out_5573296243824839433[3] = 0;
   out_5573296243824839433[4] = 0;
   out_5573296243824839433[5] = 0;
   out_5573296243824839433[6] = 0;
   out_5573296243824839433[7] = 0;
   out_5573296243824839433[8] = 0;
}
void h_31(double *state, double *unused, double *out_288673325911230581) {
   out_288673325911230581[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8141295248109373295) {
   out_8141295248109373295[0] = 0;
   out_8141295248109373295[1] = 0;
   out_8141295248109373295[2] = 0;
   out_8141295248109373295[3] = 0;
   out_8141295248109373295[4] = 0;
   out_8141295248109373295[5] = 0;
   out_8141295248109373295[6] = 0;
   out_8141295248109373295[7] = 0;
   out_8141295248109373295[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7166728709141313111) {
  err_fun(nom_x, delta_x, out_7166728709141313111);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7904298925836893155) {
  inv_err_fun(nom_x, true_x, out_7904298925836893155);
}
void car_H_mod_fun(double *state, double *out_2723439132888064356) {
  H_mod_fun(state, out_2723439132888064356);
}
void car_f_fun(double *state, double dt, double *out_9222521799993261203) {
  f_fun(state,  dt, out_9222521799993261203);
}
void car_F_fun(double *state, double dt, double *out_5859762924884839019) {
  F_fun(state,  dt, out_5859762924884839019);
}
void car_h_25(double *state, double *unused, double *out_3573010150517271071) {
  h_25(state, unused, out_3573010150517271071);
}
void car_H_25(double *state, double *unused, double *out_3773583827001965595) {
  H_25(state, unused, out_3773583827001965595);
}
void car_h_24(double *state, double *unused, double *out_4865118116114068912) {
  h_24(state, unused, out_4865118116114068912);
}
void car_H_24(double *state, double *unused, double *out_8642398692029672447) {
  H_24(state, unused, out_8642398692029672447);
}
void car_h_30(double *state, double *unused, double *out_2057634506830890826) {
  h_30(state, unused, out_2057634506830890826);
}
void car_H_30(double *state, double *unused, double *out_8301280157129573793) {
  H_30(state, unused, out_8301280157129573793);
}
void car_h_26(double *state, double *unused, double *out_2944038760829954819) {
  h_26(state, unused, out_2944038760829954819);
}
void car_H_26(double *state, double *unused, double *out_7515087145876021819) {
  H_26(state, unused, out_7515087145876021819);
}
void car_h_27(double *state, double *unused, double *out_8574683418178116716) {
  h_27(state, unused, out_8574683418178116716);
}
void car_H_27(double *state, double *unused, double *out_7970700604779552912) {
  H_27(state, unused, out_7970700604779552912);
}
void car_h_29(double *state, double *unused, double *out_7124052025549233549) {
  h_29(state, unused, out_7124052025549233549);
}
void car_H_29(double *state, double *unused, double *out_7791048812815181609) {
  H_29(state, unused, out_7791048812815181609);
}
void car_h_28(double *state, double *unused, double *out_4870387387870589255) {
  h_28(state, unused, out_4870387387870589255);
}
void car_H_28(double *state, double *unused, double *out_5573296243824839433) {
  H_28(state, unused, out_5573296243824839433);
}
void car_h_31(double *state, double *unused, double *out_288673325911230581) {
  h_31(state, unused, out_288673325911230581);
}
void car_H_31(double *state, double *unused, double *out_8141295248109373295) {
  H_31(state, unused, out_8141295248109373295);
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
