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
void err_fun(double *nom_x, double *delta_x, double *out_7807722455415062785) {
   out_7807722455415062785[0] = delta_x[0] + nom_x[0];
   out_7807722455415062785[1] = delta_x[1] + nom_x[1];
   out_7807722455415062785[2] = delta_x[2] + nom_x[2];
   out_7807722455415062785[3] = delta_x[3] + nom_x[3];
   out_7807722455415062785[4] = delta_x[4] + nom_x[4];
   out_7807722455415062785[5] = delta_x[5] + nom_x[5];
   out_7807722455415062785[6] = delta_x[6] + nom_x[6];
   out_7807722455415062785[7] = delta_x[7] + nom_x[7];
   out_7807722455415062785[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_403857763333909611) {
   out_403857763333909611[0] = -nom_x[0] + true_x[0];
   out_403857763333909611[1] = -nom_x[1] + true_x[1];
   out_403857763333909611[2] = -nom_x[2] + true_x[2];
   out_403857763333909611[3] = -nom_x[3] + true_x[3];
   out_403857763333909611[4] = -nom_x[4] + true_x[4];
   out_403857763333909611[5] = -nom_x[5] + true_x[5];
   out_403857763333909611[6] = -nom_x[6] + true_x[6];
   out_403857763333909611[7] = -nom_x[7] + true_x[7];
   out_403857763333909611[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4427032663813456926) {
   out_4427032663813456926[0] = 1.0;
   out_4427032663813456926[1] = 0.0;
   out_4427032663813456926[2] = 0.0;
   out_4427032663813456926[3] = 0.0;
   out_4427032663813456926[4] = 0.0;
   out_4427032663813456926[5] = 0.0;
   out_4427032663813456926[6] = 0.0;
   out_4427032663813456926[7] = 0.0;
   out_4427032663813456926[8] = 0.0;
   out_4427032663813456926[9] = 0.0;
   out_4427032663813456926[10] = 1.0;
   out_4427032663813456926[11] = 0.0;
   out_4427032663813456926[12] = 0.0;
   out_4427032663813456926[13] = 0.0;
   out_4427032663813456926[14] = 0.0;
   out_4427032663813456926[15] = 0.0;
   out_4427032663813456926[16] = 0.0;
   out_4427032663813456926[17] = 0.0;
   out_4427032663813456926[18] = 0.0;
   out_4427032663813456926[19] = 0.0;
   out_4427032663813456926[20] = 1.0;
   out_4427032663813456926[21] = 0.0;
   out_4427032663813456926[22] = 0.0;
   out_4427032663813456926[23] = 0.0;
   out_4427032663813456926[24] = 0.0;
   out_4427032663813456926[25] = 0.0;
   out_4427032663813456926[26] = 0.0;
   out_4427032663813456926[27] = 0.0;
   out_4427032663813456926[28] = 0.0;
   out_4427032663813456926[29] = 0.0;
   out_4427032663813456926[30] = 1.0;
   out_4427032663813456926[31] = 0.0;
   out_4427032663813456926[32] = 0.0;
   out_4427032663813456926[33] = 0.0;
   out_4427032663813456926[34] = 0.0;
   out_4427032663813456926[35] = 0.0;
   out_4427032663813456926[36] = 0.0;
   out_4427032663813456926[37] = 0.0;
   out_4427032663813456926[38] = 0.0;
   out_4427032663813456926[39] = 0.0;
   out_4427032663813456926[40] = 1.0;
   out_4427032663813456926[41] = 0.0;
   out_4427032663813456926[42] = 0.0;
   out_4427032663813456926[43] = 0.0;
   out_4427032663813456926[44] = 0.0;
   out_4427032663813456926[45] = 0.0;
   out_4427032663813456926[46] = 0.0;
   out_4427032663813456926[47] = 0.0;
   out_4427032663813456926[48] = 0.0;
   out_4427032663813456926[49] = 0.0;
   out_4427032663813456926[50] = 1.0;
   out_4427032663813456926[51] = 0.0;
   out_4427032663813456926[52] = 0.0;
   out_4427032663813456926[53] = 0.0;
   out_4427032663813456926[54] = 0.0;
   out_4427032663813456926[55] = 0.0;
   out_4427032663813456926[56] = 0.0;
   out_4427032663813456926[57] = 0.0;
   out_4427032663813456926[58] = 0.0;
   out_4427032663813456926[59] = 0.0;
   out_4427032663813456926[60] = 1.0;
   out_4427032663813456926[61] = 0.0;
   out_4427032663813456926[62] = 0.0;
   out_4427032663813456926[63] = 0.0;
   out_4427032663813456926[64] = 0.0;
   out_4427032663813456926[65] = 0.0;
   out_4427032663813456926[66] = 0.0;
   out_4427032663813456926[67] = 0.0;
   out_4427032663813456926[68] = 0.0;
   out_4427032663813456926[69] = 0.0;
   out_4427032663813456926[70] = 1.0;
   out_4427032663813456926[71] = 0.0;
   out_4427032663813456926[72] = 0.0;
   out_4427032663813456926[73] = 0.0;
   out_4427032663813456926[74] = 0.0;
   out_4427032663813456926[75] = 0.0;
   out_4427032663813456926[76] = 0.0;
   out_4427032663813456926[77] = 0.0;
   out_4427032663813456926[78] = 0.0;
   out_4427032663813456926[79] = 0.0;
   out_4427032663813456926[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6512143296045430298) {
   out_6512143296045430298[0] = state[0];
   out_6512143296045430298[1] = state[1];
   out_6512143296045430298[2] = state[2];
   out_6512143296045430298[3] = state[3];
   out_6512143296045430298[4] = state[4];
   out_6512143296045430298[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6512143296045430298[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6512143296045430298[7] = state[7];
   out_6512143296045430298[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4489943215438999676) {
   out_4489943215438999676[0] = 1;
   out_4489943215438999676[1] = 0;
   out_4489943215438999676[2] = 0;
   out_4489943215438999676[3] = 0;
   out_4489943215438999676[4] = 0;
   out_4489943215438999676[5] = 0;
   out_4489943215438999676[6] = 0;
   out_4489943215438999676[7] = 0;
   out_4489943215438999676[8] = 0;
   out_4489943215438999676[9] = 0;
   out_4489943215438999676[10] = 1;
   out_4489943215438999676[11] = 0;
   out_4489943215438999676[12] = 0;
   out_4489943215438999676[13] = 0;
   out_4489943215438999676[14] = 0;
   out_4489943215438999676[15] = 0;
   out_4489943215438999676[16] = 0;
   out_4489943215438999676[17] = 0;
   out_4489943215438999676[18] = 0;
   out_4489943215438999676[19] = 0;
   out_4489943215438999676[20] = 1;
   out_4489943215438999676[21] = 0;
   out_4489943215438999676[22] = 0;
   out_4489943215438999676[23] = 0;
   out_4489943215438999676[24] = 0;
   out_4489943215438999676[25] = 0;
   out_4489943215438999676[26] = 0;
   out_4489943215438999676[27] = 0;
   out_4489943215438999676[28] = 0;
   out_4489943215438999676[29] = 0;
   out_4489943215438999676[30] = 1;
   out_4489943215438999676[31] = 0;
   out_4489943215438999676[32] = 0;
   out_4489943215438999676[33] = 0;
   out_4489943215438999676[34] = 0;
   out_4489943215438999676[35] = 0;
   out_4489943215438999676[36] = 0;
   out_4489943215438999676[37] = 0;
   out_4489943215438999676[38] = 0;
   out_4489943215438999676[39] = 0;
   out_4489943215438999676[40] = 1;
   out_4489943215438999676[41] = 0;
   out_4489943215438999676[42] = 0;
   out_4489943215438999676[43] = 0;
   out_4489943215438999676[44] = 0;
   out_4489943215438999676[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4489943215438999676[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4489943215438999676[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4489943215438999676[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4489943215438999676[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4489943215438999676[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4489943215438999676[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4489943215438999676[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4489943215438999676[53] = -9.8100000000000005*dt;
   out_4489943215438999676[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4489943215438999676[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4489943215438999676[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4489943215438999676[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4489943215438999676[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4489943215438999676[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4489943215438999676[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4489943215438999676[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4489943215438999676[62] = 0;
   out_4489943215438999676[63] = 0;
   out_4489943215438999676[64] = 0;
   out_4489943215438999676[65] = 0;
   out_4489943215438999676[66] = 0;
   out_4489943215438999676[67] = 0;
   out_4489943215438999676[68] = 0;
   out_4489943215438999676[69] = 0;
   out_4489943215438999676[70] = 1;
   out_4489943215438999676[71] = 0;
   out_4489943215438999676[72] = 0;
   out_4489943215438999676[73] = 0;
   out_4489943215438999676[74] = 0;
   out_4489943215438999676[75] = 0;
   out_4489943215438999676[76] = 0;
   out_4489943215438999676[77] = 0;
   out_4489943215438999676[78] = 0;
   out_4489943215438999676[79] = 0;
   out_4489943215438999676[80] = 1;
}
void h_25(double *state, double *unused, double *out_6013164719541088623) {
   out_6013164719541088623[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5302341408152459906) {
   out_5302341408152459906[0] = 0;
   out_5302341408152459906[1] = 0;
   out_5302341408152459906[2] = 0;
   out_5302341408152459906[3] = 0;
   out_5302341408152459906[4] = 0;
   out_5302341408152459906[5] = 0;
   out_5302341408152459906[6] = 1;
   out_5302341408152459906[7] = 0;
   out_5302341408152459906[8] = 0;
}
void h_24(double *state, double *unused, double *out_2616353451905830894) {
   out_2616353451905830894[0] = state[4];
   out_2616353451905830894[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3129691809146960340) {
   out_3129691809146960340[0] = 0;
   out_3129691809146960340[1] = 0;
   out_3129691809146960340[2] = 0;
   out_3129691809146960340[3] = 0;
   out_3129691809146960340[4] = 1;
   out_3129691809146960340[5] = 0;
   out_3129691809146960340[6] = 0;
   out_3129691809146960340[7] = 0;
   out_3129691809146960340[8] = 0;
   out_3129691809146960340[9] = 0;
   out_3129691809146960340[10] = 0;
   out_3129691809146960340[11] = 0;
   out_3129691809146960340[12] = 0;
   out_3129691809146960340[13] = 0;
   out_3129691809146960340[14] = 1;
   out_3129691809146960340[15] = 0;
   out_3129691809146960340[16] = 0;
   out_3129691809146960340[17] = 0;
}
void h_30(double *state, double *unused, double *out_3793701877449921109) {
   out_3793701877449921109[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7820674366659708533) {
   out_7820674366659708533[0] = 0;
   out_7820674366659708533[1] = 0;
   out_7820674366659708533[2] = 0;
   out_7820674366659708533[3] = 0;
   out_7820674366659708533[4] = 1;
   out_7820674366659708533[5] = 0;
   out_7820674366659708533[6] = 0;
   out_7820674366659708533[7] = 0;
   out_7820674366659708533[8] = 0;
}
void h_26(double *state, double *unused, double *out_6520286259268309355) {
   out_6520286259268309355[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1560838089278403682) {
   out_1560838089278403682[0] = 0;
   out_1560838089278403682[1] = 0;
   out_1560838089278403682[2] = 0;
   out_1560838089278403682[3] = 0;
   out_1560838089278403682[4] = 0;
   out_1560838089278403682[5] = 0;
   out_1560838089278403682[6] = 0;
   out_1560838089278403682[7] = 1;
   out_1560838089278403682[8] = 0;
}
void h_27(double *state, double *unused, double *out_4998435919739762180) {
   out_4998435919739762180[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8402475635865899866) {
   out_8402475635865899866[0] = 0;
   out_8402475635865899866[1] = 0;
   out_8402475635865899866[2] = 0;
   out_8402475635865899866[3] = 1;
   out_8402475635865899866[4] = 0;
   out_8402475635865899866[5] = 0;
   out_8402475635865899866[6] = 0;
   out_8402475635865899866[7] = 0;
   out_8402475635865899866[8] = 0;
}
void h_29(double *state, double *unused, double *out_5273629982024268069) {
   out_5273629982024268069[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8330905710974100717) {
   out_8330905710974100717[0] = 0;
   out_8330905710974100717[1] = 1;
   out_8330905710974100717[2] = 0;
   out_8330905710974100717[3] = 0;
   out_8330905710974100717[4] = 0;
   out_8330905710974100717[5] = 0;
   out_8330905710974100717[6] = 0;
   out_8330905710974100717[7] = 0;
   out_8330905710974100717[8] = 0;
}
void h_28(double *state, double *unused, double *out_4427764741529312994) {
   out_4427764741529312994[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3248506693904570143) {
   out_3248506693904570143[0] = 1;
   out_3248506693904570143[1] = 0;
   out_3248506693904570143[2] = 0;
   out_3248506693904570143[3] = 0;
   out_3248506693904570143[4] = 0;
   out_3248506693904570143[5] = 0;
   out_3248506693904570143[6] = 0;
   out_3248506693904570143[7] = 0;
   out_3248506693904570143[8] = 0;
}
void h_31(double *state, double *unused, double *out_757670506809262313) {
   out_757670506809262313[0] = state[8];
}
void H_31(double *state, double *unused, double *out_934629987045052206) {
   out_934629987045052206[0] = 0;
   out_934629987045052206[1] = 0;
   out_934629987045052206[2] = 0;
   out_934629987045052206[3] = 0;
   out_934629987045052206[4] = 0;
   out_934629987045052206[5] = 0;
   out_934629987045052206[6] = 0;
   out_934629987045052206[7] = 0;
   out_934629987045052206[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7807722455415062785) {
  err_fun(nom_x, delta_x, out_7807722455415062785);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_403857763333909611) {
  inv_err_fun(nom_x, true_x, out_403857763333909611);
}
void car_H_mod_fun(double *state, double *out_4427032663813456926) {
  H_mod_fun(state, out_4427032663813456926);
}
void car_f_fun(double *state, double dt, double *out_6512143296045430298) {
  f_fun(state,  dt, out_6512143296045430298);
}
void car_F_fun(double *state, double dt, double *out_4489943215438999676) {
  F_fun(state,  dt, out_4489943215438999676);
}
void car_h_25(double *state, double *unused, double *out_6013164719541088623) {
  h_25(state, unused, out_6013164719541088623);
}
void car_H_25(double *state, double *unused, double *out_5302341408152459906) {
  H_25(state, unused, out_5302341408152459906);
}
void car_h_24(double *state, double *unused, double *out_2616353451905830894) {
  h_24(state, unused, out_2616353451905830894);
}
void car_H_24(double *state, double *unused, double *out_3129691809146960340) {
  H_24(state, unused, out_3129691809146960340);
}
void car_h_30(double *state, double *unused, double *out_3793701877449921109) {
  h_30(state, unused, out_3793701877449921109);
}
void car_H_30(double *state, double *unused, double *out_7820674366659708533) {
  H_30(state, unused, out_7820674366659708533);
}
void car_h_26(double *state, double *unused, double *out_6520286259268309355) {
  h_26(state, unused, out_6520286259268309355);
}
void car_H_26(double *state, double *unused, double *out_1560838089278403682) {
  H_26(state, unused, out_1560838089278403682);
}
void car_h_27(double *state, double *unused, double *out_4998435919739762180) {
  h_27(state, unused, out_4998435919739762180);
}
void car_H_27(double *state, double *unused, double *out_8402475635865899866) {
  H_27(state, unused, out_8402475635865899866);
}
void car_h_29(double *state, double *unused, double *out_5273629982024268069) {
  h_29(state, unused, out_5273629982024268069);
}
void car_H_29(double *state, double *unused, double *out_8330905710974100717) {
  H_29(state, unused, out_8330905710974100717);
}
void car_h_28(double *state, double *unused, double *out_4427764741529312994) {
  h_28(state, unused, out_4427764741529312994);
}
void car_H_28(double *state, double *unused, double *out_3248506693904570143) {
  H_28(state, unused, out_3248506693904570143);
}
void car_h_31(double *state, double *unused, double *out_757670506809262313) {
  h_31(state, unused, out_757670506809262313);
}
void car_H_31(double *state, double *unused, double *out_934629987045052206) {
  H_31(state, unused, out_934629987045052206);
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
