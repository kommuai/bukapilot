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
void err_fun(double *nom_x, double *delta_x, double *out_6665965332270932763) {
   out_6665965332270932763[0] = delta_x[0] + nom_x[0];
   out_6665965332270932763[1] = delta_x[1] + nom_x[1];
   out_6665965332270932763[2] = delta_x[2] + nom_x[2];
   out_6665965332270932763[3] = delta_x[3] + nom_x[3];
   out_6665965332270932763[4] = delta_x[4] + nom_x[4];
   out_6665965332270932763[5] = delta_x[5] + nom_x[5];
   out_6665965332270932763[6] = delta_x[6] + nom_x[6];
   out_6665965332270932763[7] = delta_x[7] + nom_x[7];
   out_6665965332270932763[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5612242369879271369) {
   out_5612242369879271369[0] = -nom_x[0] + true_x[0];
   out_5612242369879271369[1] = -nom_x[1] + true_x[1];
   out_5612242369879271369[2] = -nom_x[2] + true_x[2];
   out_5612242369879271369[3] = -nom_x[3] + true_x[3];
   out_5612242369879271369[4] = -nom_x[4] + true_x[4];
   out_5612242369879271369[5] = -nom_x[5] + true_x[5];
   out_5612242369879271369[6] = -nom_x[6] + true_x[6];
   out_5612242369879271369[7] = -nom_x[7] + true_x[7];
   out_5612242369879271369[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6173598752306844576) {
   out_6173598752306844576[0] = 1.0;
   out_6173598752306844576[1] = 0.0;
   out_6173598752306844576[2] = 0.0;
   out_6173598752306844576[3] = 0.0;
   out_6173598752306844576[4] = 0.0;
   out_6173598752306844576[5] = 0.0;
   out_6173598752306844576[6] = 0.0;
   out_6173598752306844576[7] = 0.0;
   out_6173598752306844576[8] = 0.0;
   out_6173598752306844576[9] = 0.0;
   out_6173598752306844576[10] = 1.0;
   out_6173598752306844576[11] = 0.0;
   out_6173598752306844576[12] = 0.0;
   out_6173598752306844576[13] = 0.0;
   out_6173598752306844576[14] = 0.0;
   out_6173598752306844576[15] = 0.0;
   out_6173598752306844576[16] = 0.0;
   out_6173598752306844576[17] = 0.0;
   out_6173598752306844576[18] = 0.0;
   out_6173598752306844576[19] = 0.0;
   out_6173598752306844576[20] = 1.0;
   out_6173598752306844576[21] = 0.0;
   out_6173598752306844576[22] = 0.0;
   out_6173598752306844576[23] = 0.0;
   out_6173598752306844576[24] = 0.0;
   out_6173598752306844576[25] = 0.0;
   out_6173598752306844576[26] = 0.0;
   out_6173598752306844576[27] = 0.0;
   out_6173598752306844576[28] = 0.0;
   out_6173598752306844576[29] = 0.0;
   out_6173598752306844576[30] = 1.0;
   out_6173598752306844576[31] = 0.0;
   out_6173598752306844576[32] = 0.0;
   out_6173598752306844576[33] = 0.0;
   out_6173598752306844576[34] = 0.0;
   out_6173598752306844576[35] = 0.0;
   out_6173598752306844576[36] = 0.0;
   out_6173598752306844576[37] = 0.0;
   out_6173598752306844576[38] = 0.0;
   out_6173598752306844576[39] = 0.0;
   out_6173598752306844576[40] = 1.0;
   out_6173598752306844576[41] = 0.0;
   out_6173598752306844576[42] = 0.0;
   out_6173598752306844576[43] = 0.0;
   out_6173598752306844576[44] = 0.0;
   out_6173598752306844576[45] = 0.0;
   out_6173598752306844576[46] = 0.0;
   out_6173598752306844576[47] = 0.0;
   out_6173598752306844576[48] = 0.0;
   out_6173598752306844576[49] = 0.0;
   out_6173598752306844576[50] = 1.0;
   out_6173598752306844576[51] = 0.0;
   out_6173598752306844576[52] = 0.0;
   out_6173598752306844576[53] = 0.0;
   out_6173598752306844576[54] = 0.0;
   out_6173598752306844576[55] = 0.0;
   out_6173598752306844576[56] = 0.0;
   out_6173598752306844576[57] = 0.0;
   out_6173598752306844576[58] = 0.0;
   out_6173598752306844576[59] = 0.0;
   out_6173598752306844576[60] = 1.0;
   out_6173598752306844576[61] = 0.0;
   out_6173598752306844576[62] = 0.0;
   out_6173598752306844576[63] = 0.0;
   out_6173598752306844576[64] = 0.0;
   out_6173598752306844576[65] = 0.0;
   out_6173598752306844576[66] = 0.0;
   out_6173598752306844576[67] = 0.0;
   out_6173598752306844576[68] = 0.0;
   out_6173598752306844576[69] = 0.0;
   out_6173598752306844576[70] = 1.0;
   out_6173598752306844576[71] = 0.0;
   out_6173598752306844576[72] = 0.0;
   out_6173598752306844576[73] = 0.0;
   out_6173598752306844576[74] = 0.0;
   out_6173598752306844576[75] = 0.0;
   out_6173598752306844576[76] = 0.0;
   out_6173598752306844576[77] = 0.0;
   out_6173598752306844576[78] = 0.0;
   out_6173598752306844576[79] = 0.0;
   out_6173598752306844576[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2303534004413667847) {
   out_2303534004413667847[0] = state[0];
   out_2303534004413667847[1] = state[1];
   out_2303534004413667847[2] = state[2];
   out_2303534004413667847[3] = state[3];
   out_2303534004413667847[4] = state[4];
   out_2303534004413667847[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2303534004413667847[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2303534004413667847[7] = state[7];
   out_2303534004413667847[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2404614061955785468) {
   out_2404614061955785468[0] = 1;
   out_2404614061955785468[1] = 0;
   out_2404614061955785468[2] = 0;
   out_2404614061955785468[3] = 0;
   out_2404614061955785468[4] = 0;
   out_2404614061955785468[5] = 0;
   out_2404614061955785468[6] = 0;
   out_2404614061955785468[7] = 0;
   out_2404614061955785468[8] = 0;
   out_2404614061955785468[9] = 0;
   out_2404614061955785468[10] = 1;
   out_2404614061955785468[11] = 0;
   out_2404614061955785468[12] = 0;
   out_2404614061955785468[13] = 0;
   out_2404614061955785468[14] = 0;
   out_2404614061955785468[15] = 0;
   out_2404614061955785468[16] = 0;
   out_2404614061955785468[17] = 0;
   out_2404614061955785468[18] = 0;
   out_2404614061955785468[19] = 0;
   out_2404614061955785468[20] = 1;
   out_2404614061955785468[21] = 0;
   out_2404614061955785468[22] = 0;
   out_2404614061955785468[23] = 0;
   out_2404614061955785468[24] = 0;
   out_2404614061955785468[25] = 0;
   out_2404614061955785468[26] = 0;
   out_2404614061955785468[27] = 0;
   out_2404614061955785468[28] = 0;
   out_2404614061955785468[29] = 0;
   out_2404614061955785468[30] = 1;
   out_2404614061955785468[31] = 0;
   out_2404614061955785468[32] = 0;
   out_2404614061955785468[33] = 0;
   out_2404614061955785468[34] = 0;
   out_2404614061955785468[35] = 0;
   out_2404614061955785468[36] = 0;
   out_2404614061955785468[37] = 0;
   out_2404614061955785468[38] = 0;
   out_2404614061955785468[39] = 0;
   out_2404614061955785468[40] = 1;
   out_2404614061955785468[41] = 0;
   out_2404614061955785468[42] = 0;
   out_2404614061955785468[43] = 0;
   out_2404614061955785468[44] = 0;
   out_2404614061955785468[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2404614061955785468[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2404614061955785468[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2404614061955785468[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2404614061955785468[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2404614061955785468[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2404614061955785468[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2404614061955785468[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2404614061955785468[53] = -9.8100000000000005*dt;
   out_2404614061955785468[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2404614061955785468[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2404614061955785468[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2404614061955785468[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2404614061955785468[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2404614061955785468[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2404614061955785468[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2404614061955785468[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2404614061955785468[62] = 0;
   out_2404614061955785468[63] = 0;
   out_2404614061955785468[64] = 0;
   out_2404614061955785468[65] = 0;
   out_2404614061955785468[66] = 0;
   out_2404614061955785468[67] = 0;
   out_2404614061955785468[68] = 0;
   out_2404614061955785468[69] = 0;
   out_2404614061955785468[70] = 1;
   out_2404614061955785468[71] = 0;
   out_2404614061955785468[72] = 0;
   out_2404614061955785468[73] = 0;
   out_2404614061955785468[74] = 0;
   out_2404614061955785468[75] = 0;
   out_2404614061955785468[76] = 0;
   out_2404614061955785468[77] = 0;
   out_2404614061955785468[78] = 0;
   out_2404614061955785468[79] = 0;
   out_2404614061955785468[80] = 1;
}
void h_25(double *state, double *unused, double *out_8916844757207971229) {
   out_8916844757207971229[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2650618102317352899) {
   out_2650618102317352899[0] = 0;
   out_2650618102317352899[1] = 0;
   out_2650618102317352899[2] = 0;
   out_2650618102317352899[3] = 0;
   out_2650618102317352899[4] = 0;
   out_2650618102317352899[5] = 0;
   out_2650618102317352899[6] = 1;
   out_2650618102317352899[7] = 0;
   out_2650618102317352899[8] = 0;
}
void h_24(double *state, double *unused, double *out_3582409430036900214) {
   out_3582409430036900214[0] = state[4];
   out_3582409430036900214[1] = state[5];
}
void H_24(double *state, double *unused, double *out_907307496484970237) {
   out_907307496484970237[0] = 0;
   out_907307496484970237[1] = 0;
   out_907307496484970237[2] = 0;
   out_907307496484970237[3] = 0;
   out_907307496484970237[4] = 1;
   out_907307496484970237[5] = 0;
   out_907307496484970237[6] = 0;
   out_907307496484970237[7] = 0;
   out_907307496484970237[8] = 0;
   out_907307496484970237[9] = 0;
   out_907307496484970237[10] = 0;
   out_907307496484970237[11] = 0;
   out_907307496484970237[12] = 0;
   out_907307496484970237[13] = 0;
   out_907307496484970237[14] = 1;
   out_907307496484970237[15] = 0;
   out_907307496484970237[16] = 0;
   out_907307496484970237[17] = 0;
}
void h_30(double *state, double *unused, double *out_9192038819492477118) {
   out_9192038819492477118[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4266072239174263856) {
   out_4266072239174263856[0] = 0;
   out_4266072239174263856[1] = 0;
   out_4266072239174263856[2] = 0;
   out_4266072239174263856[3] = 0;
   out_4266072239174263856[4] = 1;
   out_4266072239174263856[5] = 0;
   out_4266072239174263856[6] = 0;
   out_4266072239174263856[7] = 0;
   out_4266072239174263856[8] = 0;
}
void h_26(double *state, double *unused, double *out_3966131603657152171) {
   out_3966131603657152171[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6392121421191409123) {
   out_6392121421191409123[0] = 0;
   out_6392121421191409123[1] = 0;
   out_6392121421191409123[2] = 0;
   out_6392121421191409123[3] = 0;
   out_6392121421191409123[4] = 0;
   out_6392121421191409123[5] = 0;
   out_6392121421191409123[6] = 0;
   out_6392121421191409123[7] = 1;
   out_6392121421191409123[8] = 0;
}
void h_27(double *state, double *unused, double *out_3651613828942865299) {
   out_3651613828942865299[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4954720361261017880) {
   out_4954720361261017880[0] = 0;
   out_4954720361261017880[1] = 0;
   out_4954720361261017880[2] = 0;
   out_4954720361261017880[3] = 1;
   out_4954720361261017880[4] = 0;
   out_4954720361261017880[5] = 0;
   out_4954720361261017880[6] = 0;
   out_4954720361261017880[7] = 0;
   out_4954720361261017880[8] = 0;
}
void h_29(double *state, double *unused, double *out_4412521683673211077) {
   out_4412521683673211077[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6668083088130568913) {
   out_6668083088130568913[0] = 0;
   out_6668083088130568913[1] = 1;
   out_6668083088130568913[2] = 0;
   out_6668083088130568913[3] = 0;
   out_6668083088130568913[4] = 0;
   out_6668083088130568913[5] = 0;
   out_6668083088130568913[6] = 0;
   out_6668083088130568913[7] = 0;
   out_6668083088130568913[8] = 0;
}
void h_28(double *state, double *unused, double *out_3048297944266387782) {
   out_3048297944266387782[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4704452816565242662) {
   out_4704452816565242662[0] = 1;
   out_4704452816565242662[1] = 0;
   out_4704452816565242662[2] = 0;
   out_4704452816565242662[3] = 0;
   out_4704452816565242662[4] = 0;
   out_4704452816565242662[5] = 0;
   out_4704452816565242662[6] = 0;
   out_4704452816565242662[7] = 0;
   out_4704452816565242662[8] = 0;
}
void h_31(double *state, double *unused, double *out_890021839783038503) {
   out_890021839783038503[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2619972140440392471) {
   out_2619972140440392471[0] = 0;
   out_2619972140440392471[1] = 0;
   out_2619972140440392471[2] = 0;
   out_2619972140440392471[3] = 0;
   out_2619972140440392471[4] = 0;
   out_2619972140440392471[5] = 0;
   out_2619972140440392471[6] = 0;
   out_2619972140440392471[7] = 0;
   out_2619972140440392471[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6665965332270932763) {
  err_fun(nom_x, delta_x, out_6665965332270932763);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5612242369879271369) {
  inv_err_fun(nom_x, true_x, out_5612242369879271369);
}
void car_H_mod_fun(double *state, double *out_6173598752306844576) {
  H_mod_fun(state, out_6173598752306844576);
}
void car_f_fun(double *state, double dt, double *out_2303534004413667847) {
  f_fun(state,  dt, out_2303534004413667847);
}
void car_F_fun(double *state, double dt, double *out_2404614061955785468) {
  F_fun(state,  dt, out_2404614061955785468);
}
void car_h_25(double *state, double *unused, double *out_8916844757207971229) {
  h_25(state, unused, out_8916844757207971229);
}
void car_H_25(double *state, double *unused, double *out_2650618102317352899) {
  H_25(state, unused, out_2650618102317352899);
}
void car_h_24(double *state, double *unused, double *out_3582409430036900214) {
  h_24(state, unused, out_3582409430036900214);
}
void car_H_24(double *state, double *unused, double *out_907307496484970237) {
  H_24(state, unused, out_907307496484970237);
}
void car_h_30(double *state, double *unused, double *out_9192038819492477118) {
  h_30(state, unused, out_9192038819492477118);
}
void car_H_30(double *state, double *unused, double *out_4266072239174263856) {
  H_30(state, unused, out_4266072239174263856);
}
void car_h_26(double *state, double *unused, double *out_3966131603657152171) {
  h_26(state, unused, out_3966131603657152171);
}
void car_H_26(double *state, double *unused, double *out_6392121421191409123) {
  H_26(state, unused, out_6392121421191409123);
}
void car_h_27(double *state, double *unused, double *out_3651613828942865299) {
  h_27(state, unused, out_3651613828942865299);
}
void car_H_27(double *state, double *unused, double *out_4954720361261017880) {
  H_27(state, unused, out_4954720361261017880);
}
void car_h_29(double *state, double *unused, double *out_4412521683673211077) {
  h_29(state, unused, out_4412521683673211077);
}
void car_H_29(double *state, double *unused, double *out_6668083088130568913) {
  H_29(state, unused, out_6668083088130568913);
}
void car_h_28(double *state, double *unused, double *out_3048297944266387782) {
  h_28(state, unused, out_3048297944266387782);
}
void car_H_28(double *state, double *unused, double *out_4704452816565242662) {
  H_28(state, unused, out_4704452816565242662);
}
void car_h_31(double *state, double *unused, double *out_890021839783038503) {
  h_31(state, unused, out_890021839783038503);
}
void car_H_31(double *state, double *unused, double *out_2619972140440392471) {
  H_31(state, unused, out_2619972140440392471);
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
